#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

#include "mpu6500.h"

#define AP_SSID "VIMES_CONFIG_WIFI"
#define AP_PASS "12345678"
#define MAX_STR_LEN 64
#define WIFI_CONNECTED_BIT BIT0

#define LED_GPIO      GPIO_NUM_2  
#define BUTTON_GPIO   GPIO_NUM_0  

static const char* TAG = "VIMES";

char config_wifi_ssid[MAX_STR_LEN] = {0};
char config_wifi_pass[MAX_STR_LEN] = {0};
char config_mqtt_ip[MAX_STR_LEN] = {0};
char config_mqtt_port[10] = {0};
char config_mqtt_user[MAX_STR_LEN] = {0};
char config_mqtt_pass[MAX_STR_LEN] = {0};
char config_mqtt_topic[MAX_STR_LEN] = {0};
uint8_t configured = 0;
float gravity_x = 0;
float gravity_y = 0;
float gravity_z = 0;

void get_form_value(char *src, const char *key, char *dest, size_t dest_len);
esp_err_t root_get_handler(httpd_req_t *req);
esp_err_t save_post_handler(httpd_req_t *req);
void start_webserver(void);
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void init_ap_mode();
void init_sta_mode();
void led_mode_config();
void led_mode_connected();
void led_mode_off();
void check_boot_button(void* params);
void led_blink_task(void *pvParameter);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void start_mqtt(void* params);
float vibration_rms();

const char* html_form = 
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>VIMES Configuration</title>"
    "<style>body{font-family:sans-serif;padding:20px;} input{width:100%;padding:10px;margin:5px 0;} input[type=submit]{background:#007BFF;color:white;border:none;cursor:pointer;}</style></head>"
    "<body><h2>VIMES Parameters</h2>"
    "<form action='/save' method='post'>"
    "<label>WiFi SSID:</label><input type='text' name='ssid' placeholder='Nombre de la red Wifi' required>"
    "<label>WiFi Pass:</label><input type='password' name='pass' placeholder='Clave Wifi'>"
    "<hr>"
    "<label>MQTT Broker IP:</label><input type='text' name='mip' placeholder='Ej: 192.168.1.50' required>"
    "<label>MQTT Port:</label><input type='text' name='mport' value='1883' required>"
    "<label>MQTT User:</label><input type='text' name='muser' required>"
    "<label>MQTT Pass:</label><input type='password' name='mpass' required>"
    "<label>MQTT Topic:</label><input type='text' name='mtopic' placeholder='sensores/dato' required>"
    "<br><br><input type='submit' value='Guardar y Reiniciar'>"
    "</form></body></html>";

EventGroupHandle_t s_wifi_event_group;
esp_mqtt_client_handle_t client;
TaskHandle_t s_led_task_handle = NULL;

void app_main(void) {
    nvs_handle_t handle;
    size_t required_size = MAX_STR_LEN;
    esp_err_t ret;

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u8(handle, "configured", &configured);
        if (configured == 1) {
            nvs_get_str(handle, "ssid", config_wifi_ssid, &required_size);
            required_size = MAX_STR_LEN; nvs_get_str(handle, "pass", config_wifi_pass, &required_size);
            required_size = MAX_STR_LEN; nvs_get_str(handle, "mip", config_mqtt_ip, &required_size);
            required_size = 10;          nvs_get_str(handle, "mport", config_mqtt_port, &required_size);
            required_size = MAX_STR_LEN; nvs_get_str(handle, "muser", config_mqtt_user, &required_size);
            required_size = MAX_STR_LEN; nvs_get_str(handle, "mpass", config_mqtt_pass, &required_size);
            required_size = MAX_STR_LEN; nvs_get_str(handle, "mtopic", config_mqtt_topic, &required_size);
        }
        nvs_close(handle);
    }
    if (configured == 1) {
        xTaskCreatePinnedToCore(check_boot_button,"CLEAR CONFIG",1024,NULL,5,NULL,1);
        init_sta_mode();
    } else {
        init_ap_mode();
    }
}

void get_form_value(char *src, const char *key, char *dest, size_t dest_len)
{
    char key_pattern[32];
    char hex[3]; 
    char* start;
    char* end;
    size_t len;
    int idx;
    int val;
    int j = 0;

    snprintf(key_pattern, sizeof(key_pattern), "%s=", key);
    start = strstr(src, key_pattern);
    
    while (start != NULL) {
        if (start == src || *(start - 1) == '&' || *(start - 1) == '?') {
            break;
        }
        start = strstr(start + 1, key_pattern);
    }

    if (start)
    {
        start += strlen(key_pattern);
        end = strchr(start, '&');
        len = end ? (end - start) : strlen(start);
        if (len >= dest_len)
            len = dest_len - 1;

        j = 0;
        for (idx = 0; idx < len; idx++)
        {
            if (start[idx] == '+')
                dest[j++] = ' ';
            else if (start[idx] == '%' && idx + 2 < len)
            {
                hex[0] = start[idx + 1]; 
                hex[1] = start[idx + 2];
                hex[2] = 0;
                
                val = (int)strtol(hex, NULL, 16);
                dest[j++] = (char)val;
                idx += 2;
            }
            else
            {
                dest[j++] = start[idx];
            }
        }
        dest[j] = '\0';
    }
}

esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_send(req, html_form, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t save_post_handler(httpd_req_t* req)
{
    nvs_handle_t handle;
    char buf[512];
    int ret, remaining = req->content_len;
    if (remaining >= sizeof(buf))
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0)
        return ESP_FAIL;
    buf[ret] = '\0';

    get_form_value(buf, "ssid", config_wifi_ssid, sizeof(config_wifi_ssid));
    get_form_value(buf, "pass", config_wifi_pass, sizeof(config_wifi_pass));
    get_form_value(buf, "mip", config_mqtt_ip, sizeof(config_mqtt_ip));
    get_form_value(buf, "mport", config_mqtt_port, sizeof(config_mqtt_port));
    get_form_value(buf, "muser", config_mqtt_user, sizeof(config_mqtt_user));
    get_form_value(buf, "mpass", config_mqtt_pass, sizeof(config_mqtt_pass));
    get_form_value(buf, "mtopic", config_mqtt_topic, sizeof(config_mqtt_topic));

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &handle));
    nvs_set_str(handle, "ssid", config_wifi_ssid);
    nvs_set_str(handle, "pass", config_wifi_pass);
    nvs_set_str(handle, "mip", config_mqtt_ip);
    nvs_set_str(handle, "mport", config_mqtt_port);
    nvs_set_str(handle, "muser", config_mqtt_user);
    nvs_set_str(handle, "mpass", config_mqtt_pass);
    nvs_set_str(handle, "mtopic", config_mqtt_topic);
    nvs_set_u8(handle, "configured", 1);
    nvs_commit(handle);
    nvs_close(handle);

    httpd_resp_send(req, "Guardado. Reiniciando...", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    esp_restart();
    return ESP_OK;
}

void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
        httpd_register_uri_handler(server, &root);
        httpd_uri_t save = {.uri = "/save", .method = HTTP_POST, .handler = save_post_handler};
        httpd_register_uri_handler(server, &save);
    }
}

void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        led_mode_off();
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reintentando conectar al WiFi...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        led_mode_connected();
        ESP_LOGI(TAG, "Conectado al WiFi. IP obtenida.");
    }
}

void init_ap_mode()
{
    led_mode_config();

    ESP_LOGI(TAG, "Modo AP Iniciado");
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID), 
            .password = AP_PASS, 
            .max_connection = 1, 
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    start_webserver();
}

void init_sta_mode()
{
    EventBits_t bits;
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, config_wifi_ssid);
    strcpy((char *)wifi_config.sta.password, config_wifi_pass);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
        xTaskCreatePinnedToCore(start_mqtt,"MQTT",4096,NULL,10,NULL,1);
    }
}

void led_mode_config()
{
    if (s_led_task_handle == NULL)
    {
        xTaskCreate(led_blink_task, "blink_task", 2048, NULL, 5, &s_led_task_handle);
    }
}

void led_mode_connected()
{
    if (s_led_task_handle != NULL)
    {
        vTaskDelete(s_led_task_handle);
        s_led_task_handle = NULL;
    }
    gpio_set_level(LED_GPIO, 1);
}

void led_mode_off()
{
    if (s_led_task_handle != NULL)
    {
        vTaskDelete(s_led_task_handle);
        s_led_task_handle = NULL;
    }
    gpio_set_level(LED_GPIO, 0);
}

void check_boot_button(void* params)
{
    while(1){
        if (gpio_get_level(BUTTON_GPIO) == 0)
        {
            vTaskDelay(100/portTICK_PERIOD_MS);
            if (gpio_get_level(BUTTON_GPIO) == 0)
            {
                ESP_LOGW(TAG, "Boton presionado. Manten 5s para borrar datos...");
                int count = 0;
                while (gpio_get_level(BUTTON_GPIO) == 0 && count < 50)
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    count++;
                    gpio_set_level(LED_GPIO, count % 2);
                }

                if (count >= 50)
                {
                    ESP_LOGE(TAG, "BORRANDO DATOS NVS...");
                    led_mode_off();
                    nvs_flash_erase();
                    nvs_flash_init();
                    esp_restart();
                }
                else
                {
                    ESP_LOGI(TAG, "Cancelado borrado de datos.");
                }
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void led_blink_task(void *pvParameter)
{
    while (1)
    {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(500/portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Conectado");
        esp_mqtt_client_subscribe(event->client, config_mqtt_topic, 0);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Mensaje MQTT: %.*s", event->data_len, event->data);
        ESP_LOGI(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
        break;
    default:
        break;
    }
}

void start_mqtt(void* params)
{
    char uri[128];
    char payload[64];
    float vibration_value;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado correctamente");

    ESP_ERROR_CHECK(mpu6500_write_byte(MPU6500_PWR_MGMT_1, 0x00));
    ESP_LOGI(TAG, "Sensor MPU6500 despertado");

    ESP_ERROR_CHECK(mpu6500_write_byte(0x1C, 0x00));

    snprintf(uri, sizeof(uri), "mqtt://%s:%s", config_mqtt_ip, config_mqtt_port);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .credentials.username = config_mqtt_user,
        .credentials.authentication.password = config_mqtt_pass,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    while(1)
    {
        vibration_value = vibration_rms();
        sprintf(payload, "{\"vibracion\": %.6f}", vibration_value);
        esp_mqtt_client_publish(client, config_mqtt_topic, payload, strlen(payload), 1, 0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

float vibration_rms()
{
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    uint8_t raw_data[14];
    
    float sum_sq_x = 0;
    float sum_sq_y = 0;
    float sum_sq_z = 0;
    float vib_x, vib_y, vib_z;
    float rms_x_lsb, rms_y_lsb, rms_z_lsb;
    float rms_x_ms2, rms_y_ms2, rms_z_ms2;
    float rms_global;

    for (int i = 0; i < SAMPLES_COUNT; i++)
    {
        if (mpu6500_read_bytes(MPU6500_ACCEL_XOUT_H, raw_data, 14) == ESP_OK) 
        {
            accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
            accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
            accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);
             
            //Filtro Pasa-Baja Exponencial
            gravity_x = (ALPHA * accel_x_raw) + ((1.0 - ALPHA) * gravity_x);
            gravity_y = (ALPHA * accel_y_raw) + ((1.0 - ALPHA) * gravity_y);
            gravity_z = (ALPHA * accel_z_raw) + ((1.0 - ALPHA) * gravity_z);

            //cálculo de la vibración (componente AC)
            vib_x = accel_x_raw - gravity_x;
            vib_y = accel_y_raw - gravity_y;
            vib_z = accel_z_raw - gravity_z;

            sum_sq_x += (vib_x * vib_x);
            sum_sq_y += (vib_y * vib_y);
            sum_sq_z += (vib_z * vib_z);
        }
        else 
        {
            ESP_LOGE(TAG, "Error leyendo muestra %d", i);
        }
    }

    //CÁLCULO RMS EN LSB
    rms_x_lsb = sqrtf(sum_sq_x / SAMPLES_COUNT);
    rms_y_lsb = sqrtf(sum_sq_y / SAMPLES_COUNT);
    rms_z_lsb = sqrtf(sum_sq_z / SAMPLES_COUNT);

    //CONVERSIÓN A UNIDADES FÍSICAS (m/s^2)
    rms_x_ms2 = (rms_x_lsb / ACCEL_SCALE_FACTOR) * GRAVITY_EARTH;
    rms_y_ms2 = (rms_y_lsb / ACCEL_SCALE_FACTOR) * GRAVITY_EARTH;
    rms_z_ms2 = (rms_z_lsb / ACCEL_SCALE_FACTOR) * GRAVITY_EARTH;

    //CÁLCULO RMS GLOBAL (Suma Vectorial)
    rms_global = sqrtf((rms_x_ms2 * rms_x_ms2) + 
                             (rms_y_ms2 * rms_y_ms2) + 
                             (rms_z_ms2 * rms_z_ms2));

    ESP_LOGI(TAG, "Vibración Global RMS: %.6f m/s^2", rms_global);
    return rms_global;
}