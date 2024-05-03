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

/*---------------------------------------------------------------
        LED RGB General Macros
---------------------------------------------------------------*/
typedef enum{
    RGBLED_OFF, //0 
    RGBLED_RED, //1
    RGBLED_GREEN, //2
    RGBLED_BLUE, //3
    RGBLED_YELLOW, //4
    RGBLED_CYAN, //5 
    RGBLED_PURPLE, //6
    RGBLED_WHITE, //7
    /*3 chân đèn màu chủ đạo: Red, Green, Blue ===> tối đa 8 trạng thái đèn*/
}ledRGB_Status;

typedef struct {
    gpio_num_t redGPIONum;
    gpio_num_t blueGPIONum;
    gpio_num_t greenGPIONum;
    ledRGB_Status ledState;
}rgbLedTypDef;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE //Default: Low speed

#define LEDC_CHANNEL_RED            LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN          LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE           LEDC_CHANNEL_2
//independence: Channel Analog, co the dung chung timer 
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
    * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

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

/*---------------------------------------------------------------
        GPIO General Macros
---------------------------------------------------------------*/
#define GPIO_REDLED GPIO_NUM_23
#define GPIO_GREENLED GPIO_NUM_21
#define GPIO_BLUELED GPIO_NUM_18
#define GPIO_VOLTAGE GPIO_NUM_5

#define GPIO_NRLED GPIO_NUM_25
#define GPIO_NBLED GPIO_NUM_26
#define GPIO_NGLED GPIO_NUM_27

/*****************************************MQTT VARIABLES***************************************************************************/
static esp_mqtt_client_handle_t client;
static int msg_id;
char read_topic[20], read_data[20];
esp_mqtt_event_handle_t event;
static const char *TAG = "LEDDRIVER";
/*****************************************LED DRIVER GENERAL VARIABLES***************************************************************************/
uint8_t ctrlData[1] = {'0'}; //default init value 

TaskHandle_t dashBoardTask= NULL;
TaskHandle_t colorOpTask = NULL;
TaskHandle_t uartTask = NULL;

uint32_t LedNState = 0;
uint8_t chooseLedFlag = 0;
uint8_t tempFlag = 0;

rgbLedTypDef rgbLed1;

uint8_t ledState = 0;
uint8_t countState = 0;
uint8_t ledDrvOpt = 1;
uint8_t stopClrSptrFlg = 0;

/*****************************************LED DRIVER GENERAL VARIABLES***************************************************************************/
/*****************************************FUNCTIONS PROTOTYPE***************************************************************************/
//OPTIONAL
long map(long x, long in_min, long in_max, long out_min, long out_max);
/*****************************************FUNCTIONS PROTOTYPE***************************************************************************/

/*****************************************GPIO FUNCT BEGIN***************************************************************************/

void gpio_init(void)
{   
   //notice led gpio init 
   gpio_reset_pin(GPIO_NRLED);
   gpio_reset_pin(GPIO_NBLED);
   gpio_reset_pin(GPIO_NGLED);
   //set the gpio push/ pull output
   gpio_set_direction(GPIO_NRLED,GPIO_MODE_OUTPUT);
   gpio_set_direction(GPIO_NBLED,GPIO_MODE_OUTPUT);
   gpio_set_direction(GPIO_NGLED,GPIO_MODE_OUTPUT);
}

/*****************************************GPIO FUNCT END***************************************************************************/
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

        msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
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
/*****************************************LED RGB FUNCT BEGIN***************************************************************************/
void ledc_init(uint8_t channelCtrl, int gpio_numCtrl)
{   
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = channelCtrl,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_numCtrl,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void genPWM_RGB(ledc_channel_t channel, uint32_t duty)
{   
    // Set duty to x%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}

void showRGB(int color)
{
    int redPWM;
	int greenPWM;
	int bluePWM;
 
	if (color <= 255)          // phân vùng 1
	{
		redPWM = 255 - color;    // red đi từ sáng về tắt
		greenPWM = color;        // green đi từ tắt thành sáng
		bluePWM = 0;             // blue luôn tắt
	}
	else if (color <= 511)     // phân vùng 2
	{
		redPWM = 0;                     // đỏ luôn tắt
		greenPWM = 255 - (color - 256); // green đi từ sáng về tắt
		bluePWM = (color - 256);        // blue đi từ tắt thành sáng
	}
	else // color >= 512       // phân vùng 3
	{
		redPWM = (color - 512);         // red tắt rồi lại sáng
		greenPWM = 0;                   // green luôn tắt nhé
		bluePWM = 255 - (color - 512);  // blue sáng rồi lại tắt
	}

    redPWM = map(redPWM,0,255,8192,0);
    greenPWM = map(greenPWM,0,255,8192,0);
    bluePWM = map(bluePWM,0,255,8192,0);
 
	//xuất xung ra 
    genPWM_RGB(LEDC_CHANNEL_RED, redPWM);
    genPWM_RGB(LEDC_CHANNEL_GREEN, greenPWM);
    genPWM_RGB(LEDC_CHANNEL_BLUE, bluePWM);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

esp_err_t initLedRGB(rgbLedTypDef* rgbLedX)
{   
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(rgbLedX->redGPIONum, GPIO_MODE_OUTPUT);
    gpio_set_direction(rgbLedX->greenGPIONum, GPIO_MODE_OUTPUT);
    gpio_set_direction(rgbLedX->blueGPIONum, GPIO_MODE_OUTPUT);

    //rgbLedX->ledState = RGBLED_OFF; //default led state init

    return ESP_OK;
}

esp_err_t resetGPIOInit(rgbLedTypDef* rgbLedX,gpio_num_t red_gpio, gpio_num_t green_gpio, gpio_num_t blue_gpio)
{
    rgbLedX->redGPIONum = red_gpio;
    rgbLedX->greenGPIONum = green_gpio;
    rgbLedX->blueGPIONum = blue_gpio;

    gpio_reset_pin(rgbLedX->redGPIONum);
    gpio_reset_pin(rgbLedX->greenGPIONum);
    gpio_reset_pin(rgbLedX->blueGPIONum);

    return ESP_OK;
}

esp_err_t ledc_initRGB(rgbLedTypDef* rgbLedX)
{   
    ledc_init(LEDC_CHANNEL_RED,rgbLedX->redGPIONum); //init angalog PWM pinn
    ledc_init(LEDC_CHANNEL_GREEN,rgbLedX->greenGPIONum);
    ledc_init(LEDC_CHANNEL_BLUE,rgbLedX->blueGPIONum);

    return ESP_OK;
}

void showSpectrum (void) //auto color spectrum funct
    {   
    uint16_t i = 0;
    for (; i < 768; i++)
	{
		showRGB(i);  // Call RGBspectrum() with our new x
        vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void handle_RgbLedState(rgbLedTypDef* rgbLedX, ledRGB_Status rgbLedState)
{   
    rgbLedX->ledState = rgbLedState;
    switch (rgbLedX->ledState)
    {
        case RGBLED_OFF:
            gpio_set_level(rgbLedX->redGPIONum,1);
            gpio_set_level(rgbLedX->greenGPIONum,1);
            gpio_set_level(rgbLedX->blueGPIONum,1);
            break;
        case RGBLED_RED:
            gpio_set_level(rgbLedX->redGPIONum,0);
            gpio_set_level(rgbLedX->greenGPIONum,1);
            gpio_set_level(rgbLedX->blueGPIONum,1);
            break;
        case RGBLED_GREEN:
            gpio_set_level(rgbLedX->redGPIONum,1);
            gpio_set_level(rgbLedX->greenGPIONum,0);
            gpio_set_level(rgbLedX->blueGPIONum,1);
            break;
        case RGBLED_BLUE:
            gpio_set_level(rgbLedX->redGPIONum,1);
            gpio_set_level(rgbLedX->greenGPIONum,1);
            gpio_set_level(rgbLedX->blueGPIONum,0);
            break;
        case RGBLED_YELLOW:
            gpio_set_level(rgbLedX->redGPIONum,0);
            gpio_set_level(rgbLedX->greenGPIONum,0);
            gpio_set_level(rgbLedX->blueGPIONum,1);
            break;
        case RGBLED_CYAN:
            gpio_set_level(rgbLedX->redGPIONum,1);
            gpio_set_level(rgbLedX->greenGPIONum,0);
            gpio_set_level(rgbLedX->blueGPIONum,0);
            break;
        case RGBLED_PURPLE:
            gpio_set_level(rgbLedX->redGPIONum,0);
            gpio_set_level(rgbLedX->greenGPIONum,1);
            gpio_set_level(rgbLedX->blueGPIONum,0);
            break;
        case RGBLED_WHITE:
            gpio_set_level(rgbLedX->redGPIONum,0);
            gpio_set_level(rgbLedX->greenGPIONum,0);
            gpio_set_level(rgbLedX->blueGPIONum,0);
            break;
        default:    //trường hợp error, giá trị trạng thái đèn rác
            break;
    }
}
/*****************************************LED RGB FUNCT END***************************************************************************/
/*****************************************LED DRIVER TASKS BEGIN***************************************************************************/
uint8_t ledOptTaskFlg = 0;

static void blink_led(void) {      
    gpio_set_level(GPIO_NRLED,LedNState);
    gpio_set_level(GPIO_NGLED,LedNState);
    gpio_set_level(GPIO_NBLED,LedNState);
    LedNState = !LedNState;
}

static void handleColorOpt(void) {

    if(tempFlag==0)
    {   
        resetGPIOInit(&rgbLed1,GPIO_REDLED, GPIO_GREENLED,GPIO_BLUELED);
        initLedRGB(&rgbLed1);
        handle_RgbLedState(&rgbLed1, RGBLED_OFF);
        tempFlag = 1;
    }

    switch(ctrlData[0])
    {   
        case '0':
            ledOptTaskFlg = 1;
            chooseLedFlag = 1;
            break;
        case '1':
            if(chooseLedFlag==1) {
                ledState = 1;
                chooseLedFlag = 0;
            }
            break;
        case '2':
            if(chooseLedFlag==1) {
                ledState = 2;
                chooseLedFlag = 0;
            }
            break;
        case '3':
            if(chooseLedFlag==1) {
                ledState = 3;
                chooseLedFlag = 0;
            }
            break;
        case '4':
            if(chooseLedFlag==1) {
                ledState = 4;
                chooseLedFlag = 0;
            }
            break;
        case '5':
            if(chooseLedFlag==1) {
                ledState = 5;
                chooseLedFlag = 0;
            }
            break;
        case '6':
            if(chooseLedFlag==1) {
                ledState = 6;
                chooseLedFlag = 0;
            }
            break;
        case '7':
            if(chooseLedFlag==1) {
                ledState = 7;
                chooseLedFlag = 0;
            }
            break;
        case '8':
            if(chooseLedFlag == 1) {
                handle_RgbLedState(&rgbLed1,RGBLED_OFF);
                //quay ve trang thai chon che do
                countState = 0;
                ESP_LOGI(TAG,"comebacktodashboard");
                chooseLedFlag = 0;
                tempFlag = 0;
                ledState = 0;
                ledOptTaskFlg = 0;
            }
        default:
            break;
    }
}

static void ledRGB_task(void *arg)
{
    while(1)
    {   
        if(ledOptTaskFlg == 1) {
            switch(ledState)
            {   
                case 0:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "8", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_OFF);
                    break;
                case 1:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "1", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_RED);
                    break;
                case 2:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "2", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_GREEN);
                    break;
                case 3:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "3", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_BLUE);
                    break;
                case 4:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "4", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_YELLOW);
                    break;
                case 5:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "5", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_CYAN);
                    break;
                case 6:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "6", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_PURPLE);
                    break;
                case 7:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "7", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    handle_RgbLedState(&rgbLed1,RGBLED_WHITE);
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

static void handleColorSpectr(void) {        
    gpio_set_level(GPIO_NRLED,0);
    gpio_set_level(GPIO_NBLED,1);
    gpio_set_level(GPIO_NGLED,0);

    if(tempFlag==0)
    {   
        handle_RgbLedState(&rgbLed1, RGBLED_OFF);
        resetGPIOInit(&rgbLed1,GPIO_REDLED, GPIO_GREENLED,GPIO_BLUELED);
        ledc_initRGB(&rgbLed1);

        genPWM_RGB(LEDC_CHANNEL_RED, 8192); //off led trước khi thoát chế độ
        genPWM_RGB(LEDC_CHANNEL_GREEN, 8192);
        genPWM_RGB(LEDC_CHANNEL_BLUE, 8192);
        tempFlag = 1;
    }

    if(ctrlData[0]=='8'&&stopClrSptrFlg==1) //cu chi tay thoat che do
    {   
        genPWM_RGB(LEDC_CHANNEL_RED, 8192); //off led trước khi thoát chế độ
        genPWM_RGB(LEDC_CHANNEL_GREEN, 8192);
        genPWM_RGB(LEDC_CHANNEL_BLUE, 8192);
        //ve dashboard
        //chay task dasboard
        countState = 0;
        stopClrSptrFlg = 0;
    }

    //operation led
    uint16_t i = 0;
    for (; i < 768; i++)
	{   
        if(ctrlData[0] == '8')  {
            stopClrSptrFlg = 1;
            break;
        }
		showRGB(i);  // Call RGBspectrum() with our new x
        vTaskDelay(10 / portTICK_PERIOD_MS);
	} //show spectrum
    
}


static void handleLedSwMode_task(void *arg) //control led driver using data from hand module
{  
    while(1)
    {   
        nvs_handle_t countState_handle;
        nvs_handle_t ledState_handle;
        nvs_handle_t ledDrvOpt_handle;
                                            
        esp_err_t err_countState;
        esp_err_t err_ledState;
        esp_err_t err_ledDrvOpt;

        // Open NVS Flash before write/ read
        err_countState = nvs_open("storage", NVS_READWRITE, &countState_handle);
        if (err_countState != ESP_OK) {
            ESP_LOGI(TAG,"Open NVS CountState Flash Error\n");
        }

        err_ledState = nvs_open("storage", NVS_READWRITE, &ledState_handle);
        if (err_ledState != ESP_OK) {
            ESP_LOGI(TAG,"Open NVS Led State Flash Error\n");
        }

        err_ledDrvOpt = nvs_open("storage", NVS_READWRITE, &ledDrvOpt_handle);
        if (err_ledDrvOpt != ESP_OK) {
            ESP_LOGI(TAG,"Open NVS Led State Flash Error\n");
        }

        // Read
        err_countState = nvs_get_u8(countState_handle, "count_state", &countState);

        err_ledState = nvs_get_u8(ledState_handle, "led_state", &ledState);

        err_ledDrvOpt = nvs_get_u8(ledDrvOpt_handle, "leddriver_opt", &ledDrvOpt);
      
        if(countState == 0)
        {   
            blink_led();

            msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "0", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "8", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            switch(ctrlData[0])
            {   
                case '1':
                    tempFlag = 0;
                    ledDrvOpt = 1;
                    countState = 1;
                    break;
                case '2':
                    ledDrvOpt = 2;
                    countState = 1;
                    break;
                default:
                    break;
            }
        } else if(countState == 1)
        {
            switch (ledDrvOpt)
            {   
                case 1: 
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "1", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    gpio_set_level(GPIO_NRLED,1);
                    gpio_set_level(GPIO_NBLED,0);
                    gpio_set_level(GPIO_NGLED,0);

                    //thuc hien chon mau tai day
                    handleColorOpt(); //callback funct chon mau
                    break;
                case 2:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "2", 0, 0, 0);
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "9", 0, 0, 0); //change app to rgb picture
                    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //thuc hien color spectrum tai day
                    handleColorSpectr(); //callback funct color spectrum
                    break;
                default:
                    break;
            }
        }
    

        err_countState = nvs_set_u8(countState_handle, "count_state", countState);
        err_countState = nvs_commit(countState_handle);

        err_ledState = nvs_set_u8(ledState_handle, "led_state", ledState);
        err_ledState = nvs_commit(ledState_handle);

        err_ledDrvOpt = nvs_set_u8(ledDrvOpt_handle, "leddriver_opt", ledDrvOpt);
        err_ledDrvOpt = nvs_commit(ledDrvOpt_handle);
      
        // Close
        nvs_close(countState_handle);
        nvs_close(ledState_handle);
        nvs_close(ledDrvOpt);

        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}

//Ngay code va khong lam gi ngoai dev + test Flash: 01/05/2024
//Test va kiem tra dieu chinh truyen nhan giua Raspberry Pi + ESP32: 28/04/2024 - 30/04/2024
/*****************************************LED DRIVER TASKS END***************************************************************************/


void app_main(void)
{   
    /* Initialize NVS. */
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    mqttInit();
    uart_init();
    gpio_init();
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
    xTaskCreate (ledRGB_task,"ledRgb_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(uart_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(handleLedSwMode_task, "handle_LEDdriver_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}


