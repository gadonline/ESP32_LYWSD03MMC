/* Simple HTTP + SSL Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"

#include <esp_http_server.h>
#include "esp_tls.h"


#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "cJSON.h"


#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"

#include "esp_http_client.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0
#define TELEGRAM_TOKEN CONFIG_TELEGRAM_TOKEN
#define TELEGRAM_CHAT_ID_ACCESS_LIST CONFIG_TELEGRAM_CHAT_ID_ACCESS_LIST
#define TELEGRAM_HTTP_PROXY_SERVER CONFIG_TELEGRAM_HTTP_PROXY_SERVER
#define LYWSD03MMC_LOCATIONS_LIST CONFIG_LYWSD03MMC_LOCATIONS_LIST

//START BT CODE

struct device_struct {
    uint8_t mac[6];
    uint8_t name[12];
    char location[12];
    int rssi;
    float temp;
    int hum;
    int bat_pct;
    int bat_v;
};
struct device_struct device_list[10];

int device_count = 0;

// scan parameters
static esp_ble_scan_params_t ble_scan_params = {
		.scan_type              = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval          = 0x50,
		.scan_window            = 0x30
	};

// GAP callback
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    cJSON* cjson_location_name = NULL;
    cJSON* cjson_locations = cJSON_Parse(LYWSD03MMC_LOCATIONS_LIST);
    switch (event) {
		
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 
				
			printf("ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT\n");
			if(param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan parameters set, start scanning for 10 seconds\n\n");
				esp_ble_gap_start_scanning(10);
			}
			else printf("Unable to set scan parameters, error code %d\n\n", param->scan_param_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
			
			printf("ESP_GAP_BLE_SCAN_START_COMPLETE_EVT\n");
			if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan started\n\n");
			}
			else printf("Unable to start scan process, error code %d\n\n", param->scan_start_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
				if(param->scan_rst.ble_adv[4] == 0xA4 && param->scan_rst.ble_adv[5] == 0xC1 && param->scan_rst.ble_adv[6] == 0x38) {
                    bool add = true;
                    uint8_t mac[6];
                    uint8_t *name = NULL;
                    uint8_t name_len = 0;
                    char location[12];
                    
                    mac[0] = param->scan_rst.ble_adv[4];
                    mac[1] = param->scan_rst.ble_adv[5];
                    mac[2] = param->scan_rst.ble_adv[6];
                    mac[3] = param->scan_rst.ble_adv[7];
                    mac[4] = param->scan_rst.ble_adv[8];
                    mac[5] = param->scan_rst.ble_adv[9];
                    int rssi = param->scan_rst.rssi;
                    float temp = param->scan_rst.ble_adv[11]/10.;
                    
                    if (param->scan_rst.ble_adv[10] == 255) {
                        temp = temp - 25.6;
                    }
                    
                    int hum = param->scan_rst.ble_adv[12];
                    int bat_pct = param->scan_rst.ble_adv[13];
                    int bat_v = param->scan_rst.ble_adv[14] * 256 + param->scan_rst.ble_adv[15];
                    name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
                    
                    cjson_location_name = cJSON_GetObjectItem(cjson_locations, (char*)name);
                    
                    if (cJSON_IsString(cjson_location_name)) {
                        sprintf(location, "%s", cjson_location_name->valuestring);
                    } else {
                        sprintf(location, "%s", name);
                    }
                    
                    int i;
                    if (device_count != 0) {
                        for (i = 0; i<device_count; i++)
                        {
                            add = true;
                             if (memcmp(device_list[i].mac, mac, 6) == 0) {
                                add = false;
                                break;
                            }
                        }
                    }
                    
                    if (add == true) {
                        printf("add %d: %x:%x:%x:%x:%x:%x\n", device_count, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                        device_list[device_count].rssi = rssi;
                        memcpy(device_list[device_count].mac, mac, 6);
                        device_list[device_count].temp = temp;
                        device_list[device_count].hum = hum;
                        memcpy(device_list[device_count].name, name, name_len);
                        strcpy(device_list[device_count].location, location);
                        device_list[device_count].bat_pct = bat_pct;
                        device_list[device_count].bat_v = bat_v;
                        device_count++;
                    } else {
                        printf("update %d: %x:%x:%x:%x:%x:%x\n", i, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                        device_list[i].temp = temp;
                        device_list[i].hum = hum;
                        device_list[i].bat_pct = bat_pct;
                        device_list[i].bat_v = bat_v;
                    }
                }
			} else if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT){
                
				printf("Scan complete\n\n");
				
			}
			break;
		
		default:
		
			printf("Event %d unhandled\n\n", event);
			break;
	}
    cJSON_Delete(cjson_locations);
}

//END BT CODE

/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
*/

static const char *TAG = "example";

/* An HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    cJSON* device_list_json = cJSON_CreateObject();
    cJSON* device_json = NULL;
    char mac_str[18];
    char adv_name_str[12];
    char bat_str[14];
    
    int i;
    for (i = 0; i<device_count; i++)
    {
        sprintf(mac_str, "%x:%x:%x:%x:%x:%x",
            device_list[i].mac[0],
            device_list[i].mac[1],
            device_list[i].mac[2],
            device_list[i].mac[3],
            device_list[i].mac[4],
            device_list[i].mac[5]
        );
        sprintf(adv_name_str, "%s", device_list[i].name);
        sprintf(bat_str, "%d%% (%d mV)", device_list[i].bat_pct, device_list[i].bat_v);
        
        device_json = cJSON_CreateObject();
        cJSON_AddStringToObject(device_json, "adv_name", adv_name_str);
        cJSON_AddStringToObject(device_json, "location", device_list[i].location);
        cJSON_AddStringToObject(device_json, "mac", mac_str);
        cJSON_AddNumberToObject(device_json, "rssi", device_list[i].rssi);
        cJSON_AddNumberToObject(device_json, "temp", device_list[i].temp);
        cJSON_AddNumberToObject(device_json, "hum", device_list[i].hum);
        cJSON_AddStringToObject(device_json, "bat", bat_str);
        cJSON_AddItemToObject(device_list_json, adv_name_str, device_json);
    }
    
    char *device_list_json_print = cJSON_PrintUnformatted(device_list_json);
    cJSON_Delete(device_list_json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, device_list_json_print, HTTPD_RESP_USE_STRLEN);
    
    free(device_list_json_print);
    
    return ESP_OK;
}

static esp_err_t telegram_post_handler(httpd_req_t *req)
{   
    char *content;
    content = malloc(500);
    size_t recv_size = MIN(req->content_len, 500);

    int ret = httpd_req_recv(req, content, recv_size);
    printf("recv_size: %d\n", recv_size);
    printf("ret: %d\n", ret);
    if (ret != 0) {
        cJSON* cjson_content = NULL;
        cJSON* cjson_content_message = NULL;
        cJSON* cjson_content_message_chat = NULL;
        cJSON* cjson_content_message_chat_id = NULL;
        
        cjson_content = cJSON_Parse(content);
        free(content);
        
        if(cjson_content != NULL)
        {
            cjson_content_message = cJSON_GetObjectItem(cjson_content, "message");
            cjson_content_message_chat = cJSON_GetObjectItem(cjson_content_message, "chat");
            cjson_content_message_chat_id = cJSON_GetObjectItemCaseSensitive(cjson_content_message_chat, "id");
            bool send_message = false;
            int chat_id;
            
            if (cJSON_IsNumber(cjson_content_message_chat_id)) {
                chat_id = cjson_content_message_chat_id->valueint;
                char str[] = TELEGRAM_CHAT_ID_ACCESS_LIST;
                char *istr = strtok(str, ",");
                
                while (istr != NULL)
                {
                    if (atoi(istr) == chat_id) {
                        send_message = true;
                        break;
                    }
                    istr = strtok(NULL, ",");
                }
            }
            
            if (send_message) {
                printf("free_heap_size_start: %d\n", esp_get_free_heap_size());
                char *url;
                url = malloc(800);
                char *device;
                
                
                sprintf(url, "http://%s/bot%s/sendMessage?chat_id=%d&text=", TELEGRAM_HTTP_PROXY_SERVER, TELEGRAM_TOKEN, cjson_content_message_chat_id->valueint);
                
                int i;
                for (i = 0; i<device_count; i++)
                {
                    //: %%3A
                    //🌡 %%F0%%9F%%8C%%A1
                    //° %%C2%%B0
                    //💧 %%F0%%9F%%92%%A7
                    //\n %%0A
                    //% %%25
                    device = malloc(100);
                    sprintf(device, "%s%%3A%%20%%F0%%9F%%8C%%A1%.1f%%C2%%B0%%20%%F0%%9F%%92%%A7%d%%25%%0A", device_list[i].location, device_list[i].temp, device_list[i].hum);
                    strcat(url, device);
                    free(device);
                }
                
                printf("url: %s\n", url);
                
                esp_http_client_config_t config = {
                    .url = url,
                    .crt_bundle_attach = esp_crt_bundle_attach,
                };
                esp_http_client_handle_t client = esp_http_client_init(&config);
                free(url);
                esp_http_client_set_header(client, "Host", "api.telegram.org");
                esp_err_t err = esp_http_client_perform(client);
            
                if (err == ESP_OK) {
                    printf("HTTPS Status = %d, content_length = %lld\n",
                            esp_http_client_get_status_code(client),
                            esp_http_client_get_content_length(client));
                } else {
                    printf("Error perform http request %s", esp_err_to_name(err));
                }
                
                esp_http_client_cleanup(client);
                printf("free_heap_size_stop: %d\n", esp_get_free_heap_size());
            }
        }
        cJSON_Delete(cjson_content);
    } else {
        free(content);
    }
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "ESP32_LYWSD03MMC\n", HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler
};

static const httpd_uri_t telegram = {
    .uri       = "/ESP32_LYWSD03MMC",
    .method    = HTTP_POST,
    .handler   = telegram_post_handler
};


static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();

    esp_err_t ret = httpd_start(&server, &conf);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &telegram);
    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_webserver();
    }
}

void app_main(void)
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register event handlers to start server when Wi-Fi or Ethernet is connected,
     * and stop server when disconnection happens.
     */

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    
    //START BT CODE
    
    printf("BT scan\n\n");
	
	// set components to log only errors
	esp_log_level_set("*", ESP_LOG_ERROR);
	
	// initialize nvs
	ESP_ERROR_CHECK(nvs_flash_init());
	printf("- NVS init ok\n");
	
	// release memory reserved for classic BT (not used)
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	printf("- Memory for classic BT released\n");
	
	// initialize the BT controller with the default config
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	printf("- BT controller init ok\n");
	
	// enable the BT controller in BLE mode
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
	printf("- BT controller enabled in BLE mode\n");
	
	// initialize Bluedroid library
	ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
	printf("- Bluedroid initialized and enabled\n");
	
	// register GAP callback function
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	printf("- GAP callback registered\n\n");
    
	while(true){
		// configure scan parameters
		esp_ble_gap_set_scan_params(&ble_scan_params);
		vTaskDelay(30000 / portTICK_PERIOD_MS);
        
        cJSON* device_list_json = cJSON_CreateObject();
        cJSON* device_json = NULL;
        char mac_str[18];
        char adv_name_str[12];
        char bat_str[14];
        
        int i;
        for (i = 0; i<device_count; i++)
        {
            sprintf(mac_str, "%x:%x:%x:%x:%x:%x",
                device_list[i].mac[0],
                device_list[i].mac[1],
                device_list[i].mac[2],
                device_list[i].mac[3],
                device_list[i].mac[4],
                device_list[i].mac[5]
            );
            sprintf(adv_name_str, "%s", device_list[i].name);
            sprintf(bat_str, "%d%% (%d mV)", device_list[i].bat_pct, device_list[i].bat_v);
            device_json = cJSON_CreateObject();
            cJSON_AddStringToObject(device_json, "adv_name", adv_name_str);
            cJSON_AddStringToObject(device_json, "location", device_list[i].location);
            cJSON_AddStringToObject(device_json, "mac", mac_str);
            cJSON_AddNumberToObject(device_json, "rssi", device_list[i].rssi);
            cJSON_AddNumberToObject(device_json, "temp", device_list[i].temp);
            cJSON_AddNumberToObject(device_json, "hum", device_list[i].hum);
            cJSON_AddStringToObject(device_json, "bat", bat_str);
            cJSON_AddItemToObject(device_list_json, adv_name_str, device_json);
        }
        
        char *device_list_json_print = cJSON_Print(device_list_json);
        printf("%s\n", device_list_json_print);
        
        free(device_list_json_print);
        cJSON_Delete(device_list_json);
        
        printf("free_heap_size: %d\n", esp_get_free_heap_size());
	}
    
    //END BT CODE
}
