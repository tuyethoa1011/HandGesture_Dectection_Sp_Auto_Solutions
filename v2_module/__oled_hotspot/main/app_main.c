/*
    Lưu ý: OLED driver không hỗ trợ lưu dialog vào Flash vì nó không phải mục tiêu chính hướng đến cho đề tài (Driver chỉ là phụ có chức năng là được)
    Ý cần nếu phản biện hỏi sao không làm Flash như driver kia:
    1. Ko phải mục tiêu chính của mô hình, driver sinh ra chỉ để chứng minh việc nhận lệnh từ thiết bị nhận diện mà thực hiện hành vi.
    2. Lưu vào Flash sẽ gây tốn bộ nhớ, đọc/ ghi flash gây tốn năng lượng, việc lưu trữ đoạn hội thoại người dùng điều chỉnh là ko cần thiết vì nó có thể đúng tại thời điểm đó nhưng tại thời điểm khác thì không. Vẫn nên giữ các câu thoại thường gặp thông thường mặc định sau khi tắt hệ thống.
*/
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
#include "driver/gpio.h"
#include "sdkconfig.h"

#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_timer.h"

#define LED_CTRL_PIN 27
#define EN_BTN_PIN 33

//------I2C LCD lib -----

#include "esp_log.h"
#include "driver/i2c.h"
#include "ssd1306.h"

//Chân được cấu hình dựa trên KIT ESP được sử dụng (Current: ESP32-S2 AI Thinker)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

//----- I2C LCD define -----
#define CONFIG_SSD1306_OPMODE 0x3C

#define I2C_MASTER_SCL_IO GPIO_NUM_21           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_22              /*!< gpio number for I2C master data  */
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
char read_topic[20], read_data[100];
esp_mqtt_event_handle_t event;
static const char *TAG = "OLED_WIFI_MODULE";
uint8_t ctrlData[1] = {'0'}; //default init value 
uint8_t ctrlData_serial[1] = {'0'}; //default init value 

uint8_t clrFlg = 0;

uint8_t bufferText1[100] = "Ngu ngon!";
uint8_t bufferText2[100] = "Chao buoi trua!";
uint8_t bufferText3[100] = "Chao buoi toi!";
uint8_t bufferText4[100] = "Ban an sang chua?";
uint8_t bufferText5[100] = "Ban an trua chua?";
uint8_t bufferText6[100] = "Ban an toi chua?";
uint8_t bufferText7[100] = "Di an chung khong?";
uint8_t bufferText8[100] = "Di choi khong?";

#define LED_PIN  27
#define EN_BUTTON_PIN  33

//button sẽ có task thay đổi biến trạng thái nút bấm nhờ vào việc nhấn cứ mỗi lần nhận lệnh bấm nút thì đảo trạng thái 
//enable - disable

uint8_t buttonState = 0;
uint8_t btn_current = 0;
uint8_t btn_last = 0;
uint8_t btn_filter = 0;
uint8_t is_debouncing;
uint32_t time_deboune;
uint32_t time_start_press;
uint8_t is_press_timeout;

uint8_t ctrlSwitch = 0; //0 disable 1 enable
uint8_t enableNoticeData = '4';
uint8_t disableNoticeData = '1';
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
                strcpy((char*)ctrlData_serial,(char*)data);
                ESP_LOGI(TAG, "Ctrl data serial: %s", (char*)ctrlData_serial);

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
                        //copy chuỗi vừa nhận được vào control Data
                        strcpy((char*)ctrlData,(char*)ctrlData_serial);
                        ESP_LOGI(TAG, "Ctrl data send switch: %s", (char*)ctrlData);
                         //in ra debug thử xem nhận được đúng dữ liệu không
                        /* code */
                        break;
                    default: //error case
                        break;
                }
                //msg_id = esp_mqtt_client_publish(client, "/device/oled/",(const char*)ctrlData, 0, 0, 0); //test  transmission
                if(strcmp((const char*)read_topic,"/device/signal/")==0) //nhận tín hiệu từ thiết bị điều khiển
                {   
                    if(strcmp((const char*)read_data,"1")==0) //chuẩn tín hiệu disable chung cho tất cả các thiết bị
                    {
                        //do nothing
                    } else if (strcmp((const char*)read_data,"2")==0)
                    {
                        //thực hiện gửi lệnh điều khiển đến xe
                        msg_id = esp_mqtt_client_publish(client, "/device/car/",(const char*)ctrlData_serial, 0, 0, 0);
                    } else if(strcmp((const char*)read_data,"3")==0)
                    {
                        //thực hiện gửi lệnh điều khiển đến đèn
                        msg_id = esp_mqtt_client_publish(client, "/device/ledrgb/",(const char*)ctrlData_serial, 0, 0, 0);
                    }
                   
                }       
            }
        }
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}
/*****************************************UART FUNCT END***************************************************************************/
/*****************************************MQTT FUNCT BEGIN***************************************************************************/
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

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

        //đăng ký topic nhận dữ liệu từ thiết bị wifi phát gói tin
        msg_id = esp_mqtt_client_subscribe(client, "/device/oled/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        //msg_id = esp_mqtt_client_publish(client, "/ledrgb/mode", "data_3", 0, 1, 0);
        //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog1/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog2/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog3/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog4/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog5/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog6/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog7/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog8/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/screen/dialog9/", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        //msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        //msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
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
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

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
            .client_id = "OLED_Pi_Wifi_esp32",
        }, 
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
/*****************************************MQTT FUNCT END***************************************************************************/
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
        msg_id = esp_mqtt_client_publish(client, "/device/signal/", (const char*)&enableNoticeData, 0,0,0); //send en   able message
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);    
    } else if (ctrlSwitch == 0)
    {
        msg_id = esp_mqtt_client_publish(client, "/device/signal/", (const char*)&disableNoticeData, 0,0,0); //send disable message
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
		    if(btn_current == 1)//nhan xuong
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

void handleText (const void *arg_text) {
    char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);
    int strIndex = 0;
    uint8_t spaceCount = 0;

    for (strIndex = 0;strIndex < text_len;strIndex++)
    {   
        if(text[strIndex] == ' ')                                                                                                                                                                                                  
        {
            if(spaceCount < 2)
            {
                spaceCount++;
            } else if (spaceCount==2)
            {
                text[strIndex] = '\n';
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
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText1);
                task_ssd1306_display_text(bufferText1,I2C_MASTER_NUM);
                break;
            case '2':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText2);
                task_ssd1306_display_text(bufferText2,I2C_MASTER_NUM);
                break;
            case '3':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText3);
                task_ssd1306_display_text(bufferText3,I2C_MASTER_NUM);
                break;
            case '4':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText4);
                task_ssd1306_display_text(bufferText4,I2C_MASTER_NUM);
                break;
            case '5':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText5);
                task_ssd1306_display_text(bufferText5,I2C_MASTER_NUM);
                break;
            case '6':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText6);
                task_ssd1306_display_text(bufferText6,I2C_MASTER_NUM);
                break;
            case '7':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText7);
                task_ssd1306_display_text(bufferText7,I2C_MASTER_NUM);
                break;
            case '8':
                task_ssd1306_display_clear(I2C_MASTER_NUM);
                handleText(bufferText8);
                task_ssd1306_display_text(bufferText8,I2C_MASTER_NUM);
                break;
            default:
                break;
        }

        //tam xong xu ly man buffer tu read data copy sang cac buffer chu, phai co task khac giu nhiem vu nhan buffer roi copy nua
        vTaskDelay(1500/portTICK_PERIOD_MS);
    }
}

static void getBufferChange_task(void *arg)
{
    while(1)
    {   
        //suy nghĩ phương án làm sao loại bỏ được vấn đề hiển thị text sau khi chỉnh sửa
        if(strcmp((const char*)read_topic,"/screen/dialog1/")==0) //d1
        {   
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText1,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText1,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL);
        } else if (strcmp((const char*)read_topic,"/screen/dialog2/")==0) //d2
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText2,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText2,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL); 
        } else if (strcmp((const char*)read_topic,"/screen/dialog3/")==0) //d3
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText3,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText3,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL); 
        } else if (strcmp((const char*)read_topic,"/screen/dialog4/")==0) //d4
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText4,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText4,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL);
        } else if(strcmp((const char*)read_topic,"/screen/dialog5/")==0) //d5
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText5,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText5,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL);
        }
        else if (strcmp((const char*)read_topic,"/screen/dialog6/")==0) //d6
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText6,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText6,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL);
        } else if (strcmp((const char*)read_topic,"/screen/dialog7/")==0) //d7
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText7,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText7,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL);
        } else if (strcmp((const char*)read_topic,"/screen/dialog8/")==0) //d8
        {
            //xóa dữ liệu hiện tại của buffer chữ
            memset(bufferText8,0,NULL);
            //copy dữ liệu từ "read_data" qua buffer chữ
            sprintf((char*)bufferText8,"%.*s",strlen((const char*)read_data),(const char*)read_data);
            //xóa dữ liệu của buffer vừa nhận được
            memset(read_data,0,NULL);
        } 
        //tam xong xu ly mjan buffer tu read data copy sang cac buffer chu, phai co task khac giu nhiem vu nhan buffer roi copy nua
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    gpio_set_direction(LED_CTRL_PIN, GPIO_MODE_OUTPUT);   //init led 
    gpio_set_direction(EN_BTN_PIN, GPIO_MODE_INPUT);

    uart_init();
    i2c_master_init();
    ssd1306_init(I2C_MASTER_NUM);
    task_ssd1306_display_clear(I2C_MASTER_NUM);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    task_ssd1306_display_clear(I2C_MASTER_NUM);
	
	xTaskCreate(uart_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(handleButton, "button_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(displayText_task,"display_test_task",1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(getBufferChange_task,"get_buffer_change",1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
