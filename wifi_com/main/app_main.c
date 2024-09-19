#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "esp_err.h"
#include "mqtt_client.h"

#include "driver/uart.h"
#include "sdkconfig.h"

/*---------------------------------------------------------------
        UART General Macros
---------------------------------------------------------------*/
#define ECHO_TEST_TXD (17) //UART2 TX
#define ECHO_TEST_RXD (16) //UART2 RX
#define ECHO_TEST_RTS (-1) 
#define ECHO_TEST_CTS (-1)

#define ECHO_UART_PORT_NUM      (2)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (4096)

#define BUF_SIZE (1024)

/*****************************************MQTT VARIABLES***************************************************************************/
static esp_mqtt_client_handle_t client;
static int msg_id;
char read_topic[20], read_data[20];
esp_mqtt_event_handle_t event;
static const char *TAG = "WIFI_COM";
/*****************************************LED DRIVER GENERAL VARIABLES***************************************************************************/
uint8_t ctrlData[1] = {'0'}; //default init value - biến gửi tín hiệu điều khiển

//biến tín hiệu
//tín hiệu enable được phân chia 4 bit đầu là tín hiệu enable/ disable, 4 bit sau là id của thiết bị
//bên các con thiết bị còn lại sẽ gửi data thông qua wifi nên ta cần một biến để lưu trữ chúng


/*****************************************UART FUNCT BEGIN***************************************************************************/
/*---------------------------------------------------------------
        UART General Functions
---------------------------------------------------------------*/
void uart_init(void)
{
    //UART driver
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}
/*---------------------------------------------------------------
        UART FreeRTOS Task Function
---------------------------------------------------------------*/
static void uart_task(void *arg) //recieve data from handmodule
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        if (len) {
            data[len] = '\0';

            if(data[0] == '0'||data[0] == '1'||data[0] == '2'||data[0] == '3'||data[0] == '4'||data[0] == '5'||data[0] == '6'||data[0] == '7'||data[0] == '8')
            {
                strcpy((char*)ctrlData,(char*)data);
                ESP_LOGI(TAG, "Ctrl data: %s", (char*)ctrlData);

                //msg_id = esp_mqtt_client_publish(client, "/device/oled/",(const char*)ctrlData, 0, 0, 0); //test  transmission
                
                if(strcmp((const char*)read_topic,"/device/signal/")==0) //nhận tín hiệu từ thiết bị điều khiển
                {   
                    if(strcmp((const char*)read_data,"1")==0) //chuẩn tín hiệu disable chung cho tất cả các thiết bị
                    {
                        //do nothing
                    } else if (strcmp((const char*)read_data,"2")==0)
                    {
                        //thực hiện gửi lệnh điều khiển đến xe
                        msg_id = esp_mqtt_client_publish(client, "/device/car/",(const char*)ctrlData, 0, 0, 0);
                    } else if(strcmp((const char*)read_data,"3")==0)
                    {
                        //thực hiện gửi lệnh điều khiển đến đèn
                        msg_id = esp_mqtt_client_publish(client, "/device/ledrgb/",(const char*)ctrlData, 0, 0, 0);
                    } else if (strcmp((const char*)read_data,"4")==0)
                    {
                        msg_id = esp_mqtt_client_publish(client, "/device/oled/",(const char*)ctrlData, 0, 0, 0);
                        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    }
                   
                }       
            }
        }
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}
/*****************************************UART FUNCT END***************************************************************************/
/*****************************************MQTT FUNCT BEGIN***************************************************************************/
static void mqttInit(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    event = event_data;
    client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "/device/signal/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        
        break;
    case MQTT_EVENT_DISCONNECTED:
        //ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        //printf("DATA=%.*s\r\n", event->data_len, event->data);

        sprintf(read_topic,"%.*s",event->topic_len,event->topic);
        sprintf(read_data,"%.*s",event->data_len,event->data);

        ESP_LOGI(TAG, "Topic: %s", (const char*)read_topic);
        ESP_LOGI(TAG,"Data: %s", (const char*)read_data);

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.hostname = "mqtt.flespi.io",
            .address.transport = MQTT_TRANSPORT_OVER_TCP,
            .address.port = 1883,
        },
        .credentials = {
            .username = "Jw92hp0jnKbNhP0aN4wcJMQUE5emGzVdVU618Ualu9YzONzpHTsev6ZY218yqTA9",
            .client_id = "Pi_Wifi_esp32",
        }, 
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{   

    uart_init();
    mqttInit();

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    xTaskCreate(uart_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}


