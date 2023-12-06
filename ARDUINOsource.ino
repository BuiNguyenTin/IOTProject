#include <ESP8266WiFi.h>          //Wifi
#include <PubSubClient.h>         //Wifi
#include "DHT.h"                  //DHT
#include <LiquidCrystal_I2C.h>    //LCD
#include <Wire.h>                 //Set SDA, SCL of LCD
#include <Adafruit_Fingerprint.h> //Finger
#include <WiFi.h>                 //ifttt

//IFTTT
// const char* host = "maker.ifttt.com";
// const int port = 80;
// const char* request = "/trigger/SLnoti/with/key/l9Pu7kbjrh7bB-AxXeNssh_SVLmRjRtXFVDlfw-Q3U1"; //Thay đổi tên noti
// //?value1=4&value2=5 cộng chuỗi

// void sendRequest() {
//   WiFiClient client;
//   while(!client.connect(host, port)) {
//     Serial.println("connection fail");
//     delay(1000);
//   }
//   client.print(String("GET ") + request + " HTTP/1.1\r\n"
//               + "Host: " + host + "\r\n"
//               + "Connection: close\r\n\r\n");
//   delay(500);

//   while(client.available()) {
//     String line = client.readStringUntil('\r');
//     Serial.print(line);
//   }
// }

//DHT 
#define DHTPIN D5 // what digital pin we're connected to
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

//KK
int sensorValue;

//UltraSonic
const int trig = D8;  // trig of HC-SR04
const int echo = D7;  // echo of HC-SR04

// #i2c = I2C(scl=Pin(0), sda=Pin(2), freq=10000)
//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
int sdaPin = D3;
int sclPin = D4;

//Finger
volatile int finger_status = -1;

#define Finger_Rx 4
#define Finger_Tx 5

#if (defined(AVR) || defined(ESP8266)) && !defined(AVR_ATmega2560)
SoftwareSerial mySerial(Finger_Rx, Finger_Tx);
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

uint8_t id;

//Read number from Serial
uint8_t readnumber(void) { 
  uint8_t num = 0;

  while (num == 0) {
    while (! Serial.available());
    num = Serial.parseInt();
  }
  return num;
}

//***Set server***
const char* mqttServer = "broker.mqtt-dashboard.com"; 
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void wifiConnect() {
  WiFi.begin("THREE O'CLOCK", "3open24h");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("21127702/led");
    }
    else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

//***MQTT Receiver***
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);

    //***Insert code here to control other devices***
  if(String(topic) == "21127702/led") {
    if(strMsg == "on") {
      digitalWrite(2, HIGH);
    }
    else {
      digitalWrite(2, LOW);
    }
  }
}

// Temperature
void temperature(){
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  char buffer_tem[50];
  sprintf(buffer_tem, "%f", t);
  mqttClient.publish("21127583/tem", buffer_tem);

  char buffer_hum[50];
  sprintf(buffer_hum, "%f", h);
  mqttClient.publish("21127583/hum", buffer_hum);
  
  int distance = ultrasonic(); //Cal Distance when human come closely
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }

  //LCD displays Humidity
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Humidity: ");
  lcd.setCursor(0,1);
  lcd.print(h);

  //Cal Distance when human come closely
  distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  delay(1000);

  //LCD displays Temperature
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Temperature: ");
  lcd.setCursor(0,1);
  lcd.print(t);

  //Cal Distance when human come closely
  distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  delay(1000);
}

// KK
void air_quality(){
  sensorValue = analogRead(A0);  
  char buffer_air[50];
  sprintf(buffer_air, "%d", sensorValue);
  mqttClient.publish("21127583/air", buffer_air);

  //Cal Distance when human come closely
  int distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }

  //LCD displays Air quality
  lcd.setCursor(0,0);
  lcd.clear();     
  lcd.print("AirQua=");
  lcd.setCursor(0,1);
  lcd.print(sensorValue);

  //Cal Distance when human come closely
  int distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  delay(1000);
}

//Ultrasonic
long getDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH);
  long distanceCm = duration * 0.034 / 2;
  return distanceCm;
}

int ultrasonic(){ 
  int distanceCm = getDistance();
  if(distanceCm <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }

  /* In kết quả ra Serial Monitor *///
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Distance: ");
  lcd.setCursor(0,1);
  lcd.print(distanceCm);
  delay(1000);
  return distanceCm;
}

void setup() {
  Serial.begin(9600);
  //DHT
  dht.begin();
  //UltraSonic
  pinMode(trig, OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo, INPUT);    // chân echo sẽ nhận tín hiệu

  //LCD
  Wire.begin(sdaPin, sclPin);
  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("Hello Everyone");
  lcd.setCursor(0,1);
  lcd.print("Xin chao cac ban");

  //Wifi
  Serial.print("Connecting to WiFi");
  pinMode(2, OUTPUT);

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(90);

  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);

  //Finger
  Serial.println("\n\nAdafruit Fingerprint sensor enrollment");

  // set the data rate for the sensor serial port
  finger.begin(57600);

  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor");
    while (1) { delay(1); }
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);
  //connect
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor ");
    while (1) { delay(1); }
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }

  //IFTTT
  // sendRequest();
}

void loop() {
  //Connect MQTT
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  //Temparature
  temperature();

  //Air quality
  air_quality();
  
  // FINGER 
    //enroll
  FGenroll();

    //connect
  getFingerprintID();
  delay(1000);   
}

//FINGER enroll
void FGenroll() {
  Serial.println("Please type ID # (from 1 to 127) want to save...");
  Serial.println("Please hold FINGER on Fingeprint");

  //NHẬN DỮ LIỆU TỪ WEB (SỐ) KHI NGƯỜI DÙNG NHẬP
  id = readnumber();
  if (id == 0) {// ID #0 not allowed, try again!
     return;
  }
  /*
  THAY HÀM TRÊN BẰNG HÀM NHẬN DỮ LIỆU TỪ WEB
  */

  Serial.print("Enrolling ID #");
  Serial.println(id);

  while ( !getFingerprintEnroll() );
}


//ĐỐI VỚI HÀM ENROLL NÀY, GỬI CÁC HIỂN THỊ LÊN WEB RỒI HIỂN THỊ CHO NGƯỜI DÙNG XEM LUÔN NHA
uint8_t getFingerprintEnroll() { 

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id);
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println(".");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      //GỬI DỮ LIỆU
      break;
    default:
      Serial.println("Unknown error");
      //GỬI DỮ LIỆU
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      //GỬI DỮ LIỆU
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      //GỬI DỮ LIỆU
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      //GỬI DỮ LIỆU
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      //GỬI DỮ LIỆU
      return p;
    default:
      Serial.println("Unknown error");
      //GỬI DỮ LIỆU
      return p;
  }

  Serial.println("Remove finger");
  //GỬI DỮ LIỆU
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.println(id);
  p = -1;
  Serial.println("Place same finger again");
  //GỬI DỮ LIỆU
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      //GỬI DỮ LIỆU
      break;
    default:
      Serial.println("Unknown error");
      //GỬI DỮ LIỆU
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      //GỬI DỮ LIỆU
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      //GỬI DỮ LIỆU
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      //GỬI DỮ LIỆU
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      //GỬI DỮ LIỆU
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      //GỬI DỮ LIỆU
      return p;
    default:
      Serial.println("Unknown error");
      //GỬI DỮ LIỆU
      return p;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.println(id);
  //GỬI DỮ LIỆU

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Prints matched!");
    //GỬI DỮ LIỆU
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    //GỬI DỮ LIỆU
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("Fingerprints did not match");
    //GỬI DỮ LIỆU
    return p;
  } else {
    Serial.println("Unknown error");
    //GỬI DỮ LIỆU
    return p;
  }

  Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("Stored!");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI THÀNH CÔNG
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not store in that location");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  } else {
    Serial.println("Unknown error");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  }

  return true;
}

//FINGER connect
//HÀM NÀY XUẤT PHẦN DƯỚI THÔI (CÓ COMMAND R ĐÓ)
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI THÀNH CÔNG
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  } else {
    Serial.println("Unknown error");
    //GỬI DỮ LIỆU, XUẤT TRẠNG THÁI KHÔNG THÀNH CÔNG
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return finger.fingerID;
}