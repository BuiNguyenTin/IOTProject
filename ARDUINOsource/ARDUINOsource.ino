#include <Adafruit_Fingerprint.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h" //DHT
#include <LiquidCrystal_I2C.h> //LCD
#include <Adafruit_Fingerprint.h>
#include <Servo.h>
#include <Wire.h>

int servo_time = 0;
String servo_status = "Dong";
int buzzer = D0;
int warning = 0;
int count_open = 3;

//Servo
Servo myServo;

//DHT 
#define DHTPIN D5     // what digital pin we're connected to
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

//KK
int sensorValue;

//UltraSonic
const int trig = D8;     // chân trig của HC-SR04
const int echo = D7;     // chân echo của HC-SR04

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
int sdaPin = D3;  // Chân D3 là chân SDA
int sclPin = D4;  // Chân D4 là chân SCL

//Finger
volatile int finger_status = -1;

#define Finger_Rx 4
#define Finger_Tx 5

#if (defined(_AVR_) || defined(ESP8266)) && !defined(_AVR_ATmega2560_)
SoftwareSerial mySerial(Finger_Rx, Finger_Tx);
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

uint8_t id;
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
      mqttClient.subscribe("21127583/ser_web");
      mqttClient.subscribe("21127583/fin_web");
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
  if(String(topic) == "21127583/ser_web") {
    if(strMsg == "Mo") {
      servo_open();
    }
    else {
      servo_close();
    }
  }
  if(String(topic) == "21127583/fin_web") {
    int id = strMsg.toInt();
    bool result = fingerprint_registration(id);
    if(result == 1){
      char buffer_result[5] = "true";
      mqttClient.publish("21127583/fin", buffer_result);
    }
    else{
      char buffer_result[6] = "false";
      mqttClient.publish("21127583/fin", buffer_result);
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
  
  int distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Humidity: ");
  lcd.setCursor(0,1);
  lcd.print(h);
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  delay(1000);
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Temperature: ");
  lcd.setCursor(0,1);
  lcd.print(t);
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

  int distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  lcd.setCursor(0,0);
  lcd.clear();     
  lcd.print("AirQua=");
  lcd.setCursor(0,1);
  lcd.print(sensorValue);
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
  /* In kết quả ra Serial Monitor */
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
  myServo.attach(D6);
  servo_close_web();

  //Buzzer
  pinMode(buzzer, OUTPUT);

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
  Serial.print("Connecting to WiFi");
  // pinMode(2, OUTPUT);

  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
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

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(90);
}

void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  temperature();
  air_quality();
  
  // FINGER 
  
  //connect
  if(servo_status == "Mo" && millis()-servo_time>10000){
    servo_close_web();
  }

  int check = getFingerprintID();
  if(check >= 1 && check <= 127){
    servo_open_web();
  }
  else if(check == 254){
    warning += 1;
  }
  if(warning == count_open){
    digitalWrite(D0, HIGH);
    delay(3000);
    digitalWrite(D0, LOW);
    warning = 0;
  }
  rfid();
}

bool fingerprint_registration(int id){
  // enroll
  if (id == 0) {// ID #0 not allowed, try again!
     return false;
  }
  Serial.print("Enrolling ID #");
  Serial.println(id);

  int count = 0;
  while (!  getFingerprintEnroll(id) ){
    count++;
    if(count>=3){
      return false;
    }
  }
  return true;
}

void rfid(){
  if (Serial.available() > 0) {
    int id = Serial.parseInt(); // Đọc giá trị số từ cổng serial
    // Bây giờ bạn có thể sử dụng giá trị cảm biến (sensorValue) trong Arduino
    if(id == 1){
      servo_open_web();
    }
  }
}

void servo_open(){
  myServo.write(180);
  servo_status = "Mo";
  servo_time = millis();
}

void servo_close(){
  myServo.write(0);
  servo_status = "Dong";
}

void servo_open_web(){
  myServo.write(180);
  servo_status = "Mo";
  servo_time = millis();
  char buffer_ser[servo_status.length() + 1]; // +1 để chứa ký tự kết thúc chuỗi (\0)
  servo_status.toCharArray(buffer_ser, sizeof(buffer_ser));
  mqttClient.publish("21127583/ser", buffer_ser);
}

void servo_close_web(){
  myServo.write(0);
  servo_status = "Dong";
  char buffer_ser[servo_status.length() + 1]; // +1 để chứa ký tự kết thúc chuỗi (\0)
  servo_status.toCharArray(buffer_ser, sizeof(buffer_ser));
  mqttClient.publish("21127583/ser", buffer_ser);
}

//FINGER enroll
bool getFingerprintEnroll(int id) { 

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id);
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return false;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return false;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return false;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return false;
    default:
      Serial.println("Unknown error");
      return false;
  }

  Serial.println("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.println(id);
  p = -1;
  Serial.println("Place same finger again");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return false;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return false;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return false;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return false;
    default:
      Serial.println("Unknown error");
      return false;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return false;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("Fingerprints did not match");
    return false;
  } else {
    Serial.println("Unknown error");
    return false;
  }

  Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("Stored!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return false;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not store in that location");
    return false;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    return false;
  } else {
    Serial.println("Unknown error");
    return false;
  }

  return true;
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return 253;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return 254;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return 254;
    default:
      Serial.println("Unknown error");
      return 254;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return 254;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return 254;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return 254;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return 254;
    default:
      Serial.println("Unknown error");
      return 254;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return 254;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return 254;
  } else {
    Serial.println("Unknown error");
    return 254;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return finger.fingerID;
}

// // returns -1 if failed, otherwise returns ID #
// int getFingerprintIDez() {
//   uint8_t p = finger.getImage();
//   if (p != FINGERPRINT_OK)  return -1;

//   p = finger.image2Tz();
//   if (p != FINGERPRINT_OK)  return -1;

//   p = finger.fingerFastSearch();
//   if (p != FINGERPRINT_OK)  return -1;

//   // found a match!
//   Serial.print("Found ID #"); Serial.print(finger.fingerID);
//   Serial.print(" with confidence of "); Serial.println(finger.confidence);
//   return finger.fingerID;
// }
