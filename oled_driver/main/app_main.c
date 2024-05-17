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
#include "driver/ledc.h"

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
#include "driver/gpio.h"
#include "sdkconfig.h"

#include <inttypes.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
//------I2C LCD lib -----
#include "esp_log.h"
#include "driver/i2c.h"
#include "ssd1306.h"

//Chân được cấu hình dựa trên KIT ESP được sử dụng (Current: ESP32-S2 AI Thinker)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

//----- I2C LCD define -----
#define CONFIG_SSD1306_OPMODE 0x3C

#define I2C_MASTER_SCL_IO GPIO_NUM_4           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_5              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1     /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define SSD1306_OLED_ADDR   0x3C  /*!< slave address for OLED SSD1306 */
#define SSD1306_CMD_START CONFIG_SSD1306_OPMODE   /*!< Operation mode */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
//----- I2C LCD define -----


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

uint8_t ctrlData[1] = {'0'}; //default init value 
static const char *TAG = "OLED-DRIVER";


uint8_t clrFlg = 0;

uint8_t bufferText1[100] = "The only true wisdom is in knowing you know nothing.";
uint8_t bufferText2[100] = "Chao buoi trua!";
uint8_t bufferText3[100] = "Chao buoi toi!";
uint8_t bufferText4[100] = "Ban an sang chua?";
uint8_t bufferText5[100] = "Ban an trua chua?";
uint8_t bufferText6[100] = "Ban an toi chua?";
uint8_t bufferText7[100] = "Di an chung khong?";
uint8_t bufferText8[100] = "Di choi khong?";


//----- I2C LCD essential function -----
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
//----- I2C LCD essential function -----

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

            //ESP_LOGI(TAG, "Recv str: %s", (char*)data);

            if(data[0] == '0'||data[0] == '1'||data[0] == '2'||data[0] == '3'||data[0] == '4'||data[0] == '5'||data[0] == '6'||data[0] == '7'||data[0] == '8')
            {
                strcpy((char*)ctrlData,(char*)data);
                ESP_LOGI(TAG, "Ctrl data: %s", (char*)ctrlData);
            }
        }
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

void handleText (const void *arg_text) {
    char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);
    int strIndex = 0;
    uint8_t spaceCount = 0;

    for (strIndex = 0;strIndex < text_len;strIndex++)
    {   
        if(bufferText1[strIndex] == ' ')
        {
            if(spaceCount < 2)
            {
                spaceCount++;
            } else if (spaceCount==2)
            {
                bufferText1[strIndex] = '\n';
                spaceCount = 0;
            }
        }
    }
   //ESP_LOGI(TAG, "Data after handle: %s", (char*)bufferText1);
    return;
}

static void displayText_task(void *arg)
{
    while(1)
    {   
        switch(ctrlData[0])
        {   
            case '0':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                break;
            case '1':
                handleText(bufferText1);
                task_ssd1306_display_text(bufferText1,I2C_MASTER_NUM);
                break;
            case '2':
                handleText(bufferText2);
                task_ssd1306_display_text(bufferText2,I2C_MASTER_NUM);
                break;
            case '3':
                handleText(bufferText3);
                task_ssd1306_display_text(bufferText3,I2C_MASTER_NUM);
                break;
            case '4':
                handleText(bufferText4);
                task_ssd1306_display_text(bufferText4,I2C_MASTER_NUM);
                break;
            case '5':
                handleText(bufferText5);
                task_ssd1306_display_text(bufferText5,I2C_MASTER_NUM);
                break;
            case '6':
                handleText(bufferText6);
                task_ssd1306_display_text(bufferText6,I2C_MASTER_NUM);
                break;
            case '7':
                handleText(bufferText7);
                task_ssd1306_display_text(bufferText7,I2C_MASTER_NUM);
                break;
            case '8':
                handleText(bufferText8);
                task_ssd1306_display_text(bufferText8,I2C_MASTER_NUM);
                break;
            default:
                break;
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

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

        //msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode", "data_3", 0, 1, 0);
        //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        
        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
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
            .client_id = "LED-driver_esp32",
        }, 
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
/*****************************************MQTT FUNCT END***************************************************************************/

void app_main(void)
{   
    uart_init();
    i2c_master_init();
    ssd1306_init(I2C_MASTER_NUM);

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    task_ssd1306_display_clear(I2C_MASTER_NUM);
    xTaskCreate(uart_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(displayText_task,"display_test_task",1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}