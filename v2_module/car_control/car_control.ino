#define ena   13     // ENA ==> GPIO 13     
#define enb   14     // ENB ==> GPIO 14    
#define in1  15      // IN1 ==> GPIO 15
#define in2  16      // IN2 ==> GPIO 16   
#define in3  17      // IN3 ==> GPIO 17     
#define in4  18      // IN4 ==> GPIO 18     
#define led 4        // LED ==> GPIO 4
#define servoPin 19 // Servo ==> GPIO 19
#include <PubSubClient.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <ESP32Servo.h>

/* WIFI _ MQTT settings */
#define MQTT_SERVER "mqtt.flespi.io" //ten server
#define MQTT_PORT 1883
#define MQTT_USER "Jw92hp0jnKbNhP0aN4wcJMQUE5emGzVdVU618Ualu9YzONzpHTsev6ZY218yqTA9" //username
#define MQTT_PASSWORD "" //password
#define MQTT_TOPIC_TX "/device/signal/" //topic to transmit enable/ disable control signal to hotspot module
#define MQTT_TOPIC_RX "/device/car" //topic to recieve data control after send enable data to hotspot module

#define EN_LED_PIN 27
#define EN_BTN_PIN 33

const char* ssid = "Ngo Van Hoa";
const char* password = "25021971";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

//button handle variable
bool is_debouncing = false;
unsigned long int time_debounce = 0;
int button_filter = 0;
int button_current = 0;
int button_last = 0;
unsigned long int previousMillis = 0;

bool switchCtrl = false;

int ledState = 0; //này thì có thể sẽ setup sau nếu còn dư thời gian - giờ chủ yếu tập trung motor và servo
Servo MyServo;
char data = '0'; /* data received from main hotspot */

//hàm xử lý nút bấm
void button_handle(void)
{ 
  int button_read = digitalRead(EN_BTN_PIN);
  //-------- Xu ly loc nhieu ------
  //cu co nhieu gan lai button filter
	if(button_read != button_filter) //trang thai doc duoc, khac voi trang thai truoc (button_filter)
	{ 
		//co su thay doi trang thai thi se vao day gan lien tuc vao button filter
		button_filter = button_read;
		is_debouncing = true;
		time_debounce = millis(); //moi lan co xung nhieu time debounce duoc cap nhat lai
	}
	//-------- Xac lap tin hieu ------
	if(is_debouncing == true && (millis()-time_debounce >= 25)) //check nhieu (standard vong check nhieu xap xi 25ms) - callib
	{
		button_current = button_filter;
		is_debouncing = false;
	}
	//-------- Xu ly nut nhan --------
	if(button_current != button_last)
	{
		if(button_current == 1) //press button case
		{
      //do something when button pressed
      switchCtrl = !switchCtrl;
      //Serial.println("press");
		} else { //release button case
      //do something when button released
      if(switchCtrl == true)
      {
        //Serial.println("release true");
        digitalWrite(EN_LED_PIN, HIGH);
        client.publish(MQTT_TOPIC_TX,String("2").c_str(),true); //car enable signal

      } else if (switchCtrl == false)
      {
        //Serial.println("release false");
        digitalWrite(EN_LED_PIN, LOW);
        client.publish(MQTT_TOPIC_TX,String("1").c_str(),true); 
      }   
		}
		button_last = button_current; 
	}
}

void setup_wifi(){
  Serial.print("Kết nối với ...");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");

  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("Wifi đã được kết nối");
  Serial.println("Địa chỉ IP: ");
  Serial.println(WiFi.localIP());
}

void connect_to_broker(){
  while(!client.connected()){
    Serial.print("Đang kết nối với MQTT Sever");
    String clientID = "Car Device-ESP32";
    //clientID += String(random(0xffff), HEX);
    if (client.connect(clientID.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Đã kết nối");
      client.subscribe(MQTT_TOPIC_RX); //subribe to mode topic to recieve signal from application control valve from user
    } else {
      Serial.print("Lỗi, rc=");
      Serial.print(client.state());
      Serial.println("Thử lại sau 2 giây");
      delay(2000);
    }
  }
  
}

//int ready = 1;
void callback(char* topic, byte *payload, unsigned int length) {
  //Serial.println("-------new message from broker-----");
  //Serial.print("topic: ");
  //Serial.print(topic);
  //Serial.print("\nmessage: ");
  //Serial.write(payload, length);

  //Debug section - Serial.print()..
  int countState = 0;
  //check payload
  if(String(topic) == "/device/car/")//check topic mode
  { 
    switch(countState)
    {
      case 0: //dashboard state
        if((char)payload[0] == '0')
        {
          countState = 1;
        } else if ((char)payload[0] == '1')
        {
          countState = 2;
        } else if ((char)payload[0] == '2')
        {
          countState = 3;
        }
        break;
      case 1: //control servo state
        if((char)payload[0] == '0')
        {
          Serial.println("S0");
          MyServo.write(0);
        } else if ((char)payload[0] == '1')
        {
          Serial.println("S1");
          MyServo.write(90);
        } else if ((char)payload[0] == '2')
        {
          Serial.println("S2");
          MyServo.write(180);
        }
        break;
      case 2: //conrol led state
        if((char)payload[0] == '0') //turn on led
        {
          Serial.println("off");
          //digitalWrite(led, LOW); - có thời gian thì add rồi enable hoạt động cho đèn xe sau
        } else if ((char)payload[0] == '1') //turn off led
        {
          Serial.println("on");
          //digitalWrite(led, HIGH);
        }
        break;
      default: //error motor state
        if((char)payload[0] == '0') //backward
        {
          Serial.print("B");
          Serial.println(ledState);
          BACKWARD();
        } else if ((char)payload[0] == '1') //forward
        {
          Serial.print("F");
          Serial.println(ledState);
          FORWARD();
        } else if ((char)payload[0] == '2') //turn left
        {
          Serial.print("L");
          //Serial.println(ledState);
          TURN_LEFT();
        } else if ((char)payload[0] == '3') //turn right
        {
          Serial.print("R");
          //Serial.println(ledState);
          TURN_RIGHT();
        } else if ((char)payload[0] == '4') //stop motor
        {
          Serial.print("S");
          Serial.println(ledState);
          STOP();
        }
        break;
    }
  }
}


void setup()
{
  pinMode(EN_BTN_PIN,INPUT); //0v on | >0v off
  pinMode(EN_LED_PIN,OUTPUT); //init led pin gpio
  /* initialize motor control pins as output */
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT); 
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(led, OUTPUT);
  MyServo.attach(servoPin);
  MyServo.write(90);
  Serial.begin(115200);
  
  setup_wifi();
  
  client.setServer(MQTT_SERVER, MQTT_PORT );
  client.setCallback(callback);
  connect_to_broker();
}


void loop()
{   
  client.loop();
  if (!client.connected()) {
    connect_to_broker();
  }
  button_handle();
} 

void STOP(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}
void BACKWARD(){
  analogWrite(ena, 100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW); 
  analogWrite(enb, 100); 
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void TURN_LEFT(){
  analogWrite(ena, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enb, 255); 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void TURN_RIGHT(){
  analogWrite(ena, 255); 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enb, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void FORWARD(){
  analogWrite(ena, 100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enb, 100); 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}


