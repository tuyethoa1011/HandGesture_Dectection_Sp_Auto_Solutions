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
//#include "driver/ledc.h"

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

#include "esp_timer.h"

#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#include "led_strip_rmt.h"
#include "led_strip_types.h"
#include "led_strip.h"


// GPIO assignment
#define LED_STRIP_GPIO  0
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 60
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution) 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define EXAMPLE_CHASE_SPEED_MS      500

//static uint8_t led_strip_pixels[LED_STRIP_LED_NUMBERS * 3]; - khi nao code toi kia thi open ra

#define LED_CTRL_PIN 27
#define EN_BTN_PIN 33

//button sẽ có task thay đổi biến trạng thái nút bấm nhờ vào việc nhấn cứ mỗi lần nhận lệnh bấm nút thì đảo trạng thái 
//enable - disable

uint8_t buttonState = 0;
uint8_t btn_current = 1;
uint8_t btn_last = 1;

uint8_t btn_filter = 1;
uint8_t is_debouncing;
uint32_t time_deboune;
uint32_t time_start_press;
uint8_t is_press_timeout;

uint8_t ctrlSwitch = 0; //0 disable 1 enable
uint8_t enableNoticeData = '3'; //dieu chinh lai ma ID data cho led RGB
uint8_t disableNoticeData = '1';

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
#define GPIO_NRLED GPIO_NUM_25
#define GPIO_NBLED GPIO_NUM_26

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


uint8_t ledState = 0;
uint8_t countState = 0;
uint8_t ledDrvOpt = 1;

uint8_t ledOptTaskFlg = 0;

led_strip_handle_t led_strip; //init ledstrip var

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
};

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
   //set the gpio push/ pull output
   gpio_set_direction(GPIO_NRLED,GPIO_MODE_OUTPUT);
   gpio_set_direction(GPIO_NBLED,GPIO_MODE_OUTPUT);
}

/*****************************************GPIO FUNCT END***************************************************************************/
/*---------------------------------------------------------------
        Button Functions
---------------------------------------------------------------*/
void btn_pressing_callback()
{
	ctrlSwitch = !ctrlSwitch;
    ESP_LOGI(TAG, "Switch value: %d\n", ctrlSwitch);
}
void btn_press_short_callback()
{
	//do nothing 
}
void btn_release_callback()
{   
    //thực hiện gửi tín hiệu mqtt đến wifi module hotspot
    //publish mqtt tại đây
    if(ctrlSwitch == 1)
    {
        msg_id = esp_mqtt_client_publish(client, "/device/signal/", "3", 0,0,0); //send en   able message
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);    
    } else if (ctrlSwitch == 0 )
    {
        msg_id = esp_mqtt_client_publish(client, "/device/signal/", "1", 0,0,0); //send disable message
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);  
    }
    gpio_set_level(LED_CTRL_PIN, ctrlSwitch); 
}
void btn_press_timeout_callback()
{
	//do nothing 
}

static void handleButton(void *arg)
{
    while(1)
    {
        //------------------ Loc nhieu ------------------------
        //đọc pin nút nhấn cần xử lý
	    uint8_t sta = gpio_get_level(EN_BTN_PIN);
        //ESP_LOGI(TAG, "button state: %d\n", sta);
	    if(sta != btn_filter)
	    {   
            //ESP_LOGI(TAG, "hi\n");
		    btn_filter = sta;
		    is_debouncing = 1;
		    time_deboune = esp_timer_get_time(); // ở đây đang lấy thời gian hiện tại
	    }
	    //------------------ Tin hieu da xac lap------------------------
	    if(is_debouncing && (esp_timer_get_time() - time_deboune >= 15))
	    {
		    btn_current = btn_filter;
		    is_debouncing = 0;
	    }
	    //---------------------Xu li nhan nha------------------------
	    if(btn_current != btn_last)
	    {	
		    if(btn_current == 0)//nhan xuong
		    {	
			    //printf("1 Button current: %d\n", btn_current);
			    is_press_timeout = 1;
                //hàm xử lý khi nhấn nút ở tốc độ bình thường
			    btn_pressing_callback();
			    time_start_press = esp_timer_get_time();
		    }
		    else //nha nut
		    {
			    if(esp_timer_get_time() - time_start_press <= 1000) 
			    {   
                    //hàm xử lý khi nhấn nút nhanh
				    //btn_press_short_callback();
			    }
			    //printf("0 Button current: %d\n", btn_current);
			    //printf("1 Button last: %d\n", btn_last);
                //hàm xử lý khi nhả nút
			    btn_release_callback(); //khi nhả nút ra thì hiện tại không làm gì
		    }
		    btn_last = btn_current;
	    }
	    //-------------Xu li nhan giu----------------
	    if(is_press_timeout && (esp_timer_get_time() - time_start_press >= 3000))
	    {   
            //hàm xử lý khi nhấn đè nút
		    //btn_press_timeout_callback();
		    is_press_timeout =0;
	    }
        vTaskDelay(10/portTICK_PERIOD_MS);
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

        msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode", "data_3", 0, 1, 0);
        //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        
        msg_id = esp_mqtt_client_subscribe(client, "/device/ledrgb/", 0);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

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
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        sprintf(read_topic,"%.*s",event->topic_len,event->topic);
        sprintf(read_data,"%.*s",event->data_len,event->data);
        
        if(strcmp((const char*)read_topic,"/device/ledrgb/")==0) //nhận tín hiệu từ thiết bị điều khiển
        {   
            switch (ctrlSwitch)
            {
                case 0:
                    /* code - viec enable/disable nhan tin hieu tu ben thiet bi dieu khien de bao hieu
                        1. init gpio chan den
                        2. on off den tai day
                    */
                    //off den enable
                    break;
                case 1:
                    if(strcmp((const char*)read_data,"0")==0 || strcmp((const char*)read_data,"1")==0 ||
                    strcmp((const char*)read_data,"2")==0 || strcmp((const char*)read_data,"3")==0 || 
                    strcmp((const char*)read_data,"4")==0 || strcmp((const char*)read_data,"5")==0 ||
                    strcmp((const char*)read_data,"6")==0 || strcmp((const char*)read_data,"7")==0 ||
                    strcmp((const char*)read_data,"8")==0)
                    {
                        //copy chuỗi vừa nhận được vào control Data
                        strcpy((char*)ctrlData,(char*)read_data);
                        ESP_LOGI(TAG, "Ctrl data recieve: %s", (char*)ctrlData);
                        //in ra debug thử xem nhận được đúng dữ liệu không
                    } 
                    /* code */
                    break;
                default: //error case
                    break;
            }
        }
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
/*****************************************LED STRIP FUNCT START ***************************************************************************/
led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

uint8_t ledStrip_init (void)
{
    led_strip_handle_t led_strip = configure_led(); //init led driver ws2812
    if(!led_strip)
    {
        return 1; //error case
    } else return 0; //normal case
}
/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
/*****************************************LED STRIP FUNCT END ***************************************************************************/
void ledStrip_ColorCtrl(int ledNum, int red, int green, int blue)
{   
    int i = 0;
    for (; i < ledNum; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue)); //neu gia tri khong dung thi them &
    }
}
/*****************************************LED RGB FUNCT END***************************************************************************/
/*****************************************LED DRIVER TASKS BEGIN***************************************************************************/


static void blink_led(void) {      
    gpio_set_level(GPIO_NRLED,LedNState);
    gpio_set_level(GPIO_NBLED,LedNState);
    LedNState = !LedNState;
}

static void handleColorOpt(void) {

    if(tempFlag==0)
    {   
        gpio_set_level(GPIO_NRLED,1);
        gpio_set_level(GPIO_NBLED,0);
        led_strip = configure_led(); //init led driver ws2812
        if(!led_strip)
        {
            ESP_LOGE(TAG,"install WS2812 driver failed\n");
        } else if (led_strip)
        {
            ESP_LOGE(TAG,"install WS2812 driver success\n");
        }
        tempFlag = 1;
        //if để init đèn của chế độ 1 lần 
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
                //clear tắt đèn
                //ESP_ERROR_CHECK(led_strip_clear(led_strip));
                //quay ve trang thai chon che do
                ESP_LOGI(TAG,"comebacktodashboard");
                //chooseLedFlag = 0;
                tempFlag = 0;
                ledOptTaskFlg = 0;
                //ledState = 0;
                countState = 0;
                //ledState = 0;
                ledDrvOpt = 1;
                for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
            }
        default:
            break;
    }
}

const uint8_t lights[360]={
  0,   0,   0,   0,   0,   1,   1,   2, 
  2,   3,   4,   5,   6,   7,   8,   9, 
 11,  12,  13,  15,  17,  18,  20,  22, 
 24,  26,  28,  30,  32,  35,  37,  39, 
 42,  44,  47,  49,  52,  55,  58,  60, 
 63,  66,  69,  72,  75,  78,  81,  85, 
 88,  91,  94,  97, 101, 104, 107, 111, 
114, 117, 121, 124, 127, 131, 134, 137, 
141, 144, 147, 150, 154, 157, 160, 163, 
167, 170, 173, 176, 179, 182, 185, 188, 
191, 194, 197, 200, 202, 205, 208, 210, 
213, 215, 217, 220, 222, 224, 226, 229, 
231, 232, 234, 236, 238, 239, 241, 242, 
244, 245, 246, 248, 249, 250, 251, 251, 
252, 253, 253, 254, 254, 255, 255, 255, 
255, 255, 255, 255, 254, 254, 253, 253, 
252, 251, 251, 250, 249, 248, 246, 245, 
244, 242, 241, 239, 238, 236, 234, 232, 
231, 229, 226, 224, 222, 220, 217, 215, 
213, 210, 208, 205, 202, 200, 197, 194, 
191, 188, 185, 182, 179, 176, 173, 170, 
167, 163, 160, 157, 154, 150, 147, 144, 
141, 137, 134, 131, 127, 124, 121, 117, 
114, 111, 107, 104, 101,  97,  94,  91, 
 88,  85,  81,  78,  75,  72,  69,  66, 
 63,  60,  58,  55,  52,  49,  47,  44, 
 42,  39,  37,  35,  32,  30,  28,  26, 
 24,  22,  20,  18,  17,  15,  13,  12, 
 11,   9,   8,   7,   6,   5,   4,   3, 
  2,   2,   1,   1,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0};

int rgb_red=0;
int rgb_green=120;
int rgb_blue=240;

int ledColor_red = 5;
int ledColor_green = 0;
int ledColor_blue = 0;

static void ledRGB_task(void *arg)
{
    while(1)
    {   
        if(ledOptTaskFlg == 1) {
            switch(ledState)
            {   
                case 0:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "8", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //tắt đèn - clear den
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    ESP_ERROR_CHECK(led_strip_clear(led_strip));
                    break;
                case 1:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "1", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu đỏ
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                case 2:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "2", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu xanh lá
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 0));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                case 3:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "3", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu xanh biển
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 255));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                case 4:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "4", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu vàng
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 255, 0));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                case 5:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "5", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu xanh lam
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 255));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                case 6:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "6", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu tím
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 160, 32, 240));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                case 7:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "7", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //chuyển đèn sang màu cam
                    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 254, 80,0));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

void rainbowLedStrip_handle(void)
{
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i,ledColor_red, ledColor_green, ledColor_blue));
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    ledColor_red   = lights[rgb_red];
    ledColor_green = lights[rgb_green];
    ledColor_blue = lights[rgb_blue];

    rgb_red += 1;
    rgb_green += 1;
    rgb_blue += 1;

    if (rgb_red >= 360) rgb_red=0;
    if (rgb_green >= 360) rgb_green=0;
    if (rgb_blue >= 360) rgb_blue=0;

}
static uint8_t s_led_state = 0;
static void handleLedSwMode_task(void *arg) //control led driver using data from hand module
{  
    while(1)
    {   
       /*  nvs_handle_t countState_handle;
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

        err_ledDrvOpt = nvs_get_u8(ledDrvOpt_handle, "leddriver_opt", &ledDrvOpt); */
      
        if(countState == 0)
        {   
            blink_led();

            msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "0", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "8", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            //chừng nào nút bấm nhận thì mới cho phép nhận Control Datas

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
                case '3':
                    ledDrvOpt = 3;
                    countState = 1;
                default:
                    break;
            }
        } else if(countState == 1)
        {   
            //ESP_LOGI(TAG,"Counstate current value: %d and counstate error val: %d\n", countState, err_countState);
            //ESP_LOGI(TAG,"Led driver option value: %d\n", ledDrvOpt);
            switch (ledDrvOpt)
            {  
                case 1: 
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "1", 0, 0, 0);
                    //thuc hien chon mau tai day
                    handleColorOpt(); //callback funct chon mau
                    if (ctrlData[0] == '8')
                    {
                        countState = 0;
                        //ledState = 0;
                        ledDrvOpt = 1;
                        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                        }
                        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                        ESP_ERROR_CHECK(led_strip_clear(led_strip));
                    }
                    break;
                case 2:
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "2", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/color/", "9", 0, 0, 0); //change app to rgb picture
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    //gọi hàm chạy led chaser led dây RGB
                    //operation led
                    //countState = 0;
                    //ledState = 0;
                    //ledDrvOpt = 1;
                    //gpio_set_level(GPIO_NRLED,0);
                    //gpio_set_level(GPIO_NBLED,1 );
                    if(tempFlag==0)
                    {   
                        gpio_set_level(GPIO_NRLED,0);
                        gpio_set_level(GPIO_NBLED,1 );
                        led_strip = configure_led(); //init led driver ws2812
                        if(!led_strip)
                        {
                            ESP_LOGE(TAG,"install WS2812 driver failed\n");
                        }
                        tempFlag = 1;
                        //if để init đèn của chế độ 1 lần 
                    }
                    rainbowLedStrip_handle();

                    if (ctrlData[0] == '8')
                    {
                        countState = 0;
                        //ledState = 0;
                        ledDrvOpt = 1;
                        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                        }
                        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                        ESP_ERROR_CHECK(led_strip_clear(led_strip));
                    }
                    break;
                case 3: //blink
                    msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode/", "3", 0, 0, 0);
                    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    if(tempFlag==0)
                    {   
                        gpio_set_level(GPIO_NRLED,1);
                        gpio_set_level(GPIO_NBLED,1 );
                        led_strip = configure_led(); //init led driver ws2812
                        if(!led_strip)
                        {
                            ESP_LOGE(TAG,"install WS2812 driver failed\n");
                        }
                        tempFlag = 1;
                    }
                    /* If the addressable LED is enabled */
                    if (s_led_state) {
                        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
                        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                            //ESP_LOGI(TAG,"Led State: %d", ledState);
                            switch(ledState)
                            {
                                case 1: //red
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0));
                                    break;
                                case 2: //green
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 0));
                                    break;
                                case 3: //blue
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 255));
                                    break;
                                case 4: //yellow
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 255, 0));
                                    break;
                                case 5: //cyan
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 255));
                                    break;
                                case 6: //purple
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 160, 32, 240));
                                    break;
                                case 7: //orange
                                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 254, 80,0));
                                    break;
                                default:
                                    break;
                            }
                            /* Refresh the strip to send data */
                            led_strip_refresh(led_strip);
                        }             
                    } else {
                        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                            /* Set all LED off to clear all pixels */
                            led_strip_clear(led_strip);
                        }   
                    }
                    s_led_state = !s_led_state;
                    if (ctrlData[0] == '8')
                    {
                        countState = 0;
                        //ledState = 0;
                        ledDrvOpt = 1;
                        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                        }
                        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                        ESP_ERROR_CHECK(led_strip_clear(led_strip));
                    }
                    break;
                default:
                    break;
            }
        }
    
/* 
        err_countState = nvs_set_u8(countState_handle, "count_state", countState);
        err_countState = nvs_commit(countState_handle);

        err_ledState = nvs_set_u8(ledState_handle, "led_state", ledState);
        err_ledState = nvs_commit(ledState_handle);

        err_ledDrvOpt = nvs_set_u8(ledDrvOpt_handle, "leddriver_opt", ledDrvOpt);
        err_ledDrvOpt = nvs_commit(ledDrvOpt_handle);
      
        // Close
        nvs_close(countState_handle);
        nvs_close(ledState_handle);
        nvs_close(ledDrvOpt); */ //đảm bảo chức năng rồi thêm flash sau, thêm ko được nữa thì bỏ

        vTaskDelay(250/portTICK_PERIOD_MS);
    }
}


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
    
    gpio_set_direction(LED_CTRL_PIN, GPIO_MODE_OUTPUT);   //init led 
    gpio_set_direction(EN_BTN_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(EN_BTN_PIN,  GPIO_PULLUP_ONLY);

    mqttInit();
    gpio_init();

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    xTaskCreate(handleButton, "button_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate (ledRGB_task,"ledRgb_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(handleLedSwMode_task, "handle_LEDdriver_task", ECHO_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL); //nguyen nhan
}


