#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//arm & car switching button
const int buttonPin = 0;
int lastButtonState = HIGH;
int buttonCount = -1  ;

//for arm
const int armBtnPin = 2;
int lastarmBtnState = HIGH;
int armBtnCount = 0;

Adafruit_MPU6050 mpu;
typedef struct struct_message {
  char carmove[20];
  char action[20];
  char armmove[20];
} struct_message;

struct_message gestureData;

// Replace with receiver ESP32 MAC Address
uint8_t receiverMac[] = { 0xF4, 0x65, 0x0B, 0x4A, 0x7C, 0x34 };  // example
void onSent(uint8_t *mac, uint8_t sendStatus) {
  Serial.print("Send Status: ");
  Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  // switching button
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(armBtnPin, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }
  Serial.println("MPU6050 ready!");

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: 8G");
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: 21Hz");

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(receiverMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_send_cb(onSent);
}

void loop() {
  int currentState = digitalRead(buttonPin);
  if (lastButtonState == HIGH && currentState == LOW) {
    buttonCount++;
    Serial.print("Button pressed: ");
    Serial.println(buttonCount);
    delay(200);  // debounce delay
  }

  lastButtonState = currentState;
  if (buttonCount == -1) {
    Serial.print("Robot Booting....");
  } else if (buttonCount % 2 == 0) {
    strcpy(gestureData.action, "car");

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print("/t, Y: ");
    Serial.println(a.acceleration.y);
    if (a.acceleration.y > 9) strcpy(gestureData.carmove, "L-125");
    else if (a.acceleration.y < -9) strcpy(gestureData.carmove, "R-125");
    else if (a.acceleration.x > 9) strcpy(gestureData.carmove, "B-225");
    else if (a.acceleration.x < -9) strcpy(gestureData.carmove, "F-225");
    else if (a.acceleration.y > 8) strcpy(gestureData.carmove, "L-125");
    else if (a.acceleration.y < -8) strcpy(gestureData.carmove, "R-125");
    else if (a.acceleration.x > 8) strcpy(gestureData.carmove, "B-150");
    else if (a.acceleration.x < -8) strcpy(gestureData.carmove, "F-150");
    else if (a.acceleration.y > 7) strcpy(gestureData.carmove, "L-100");
    else if (a.acceleration.y < -7) strcpy(gestureData.carmove, "R-100");
    else if (a.acceleration.x > 7) strcpy(gestureData.carmove, "B-100");
    else if (a.acceleration.x < -7) strcpy(gestureData.carmove, "F-100");
    else if (a.acceleration.y > 5) strcpy(gestureData.carmove, "L-100");
    else if (a.acceleration.y < -5) strcpy(gestureData.carmove, "R-100");
    else if (a.acceleration.x > 5) strcpy(gestureData.carmove, "B-80");
    else if (a.acceleration.x < -5) strcpy(gestureData.carmove, "F-80");
    else strcpy(gestureData.carmove, "STOP");

    Serial.print("Gesture: ");
    Serial.println(gestureData.carmove);
  } else {

    int armcurrentState = digitalRead(armBtnPin);
    if (lastarmBtnState == HIGH && armcurrentState == LOW) {
      armBtnCount++;
      Serial.print("Arm Button pressed: ");
      Serial.println(armBtnCount);
      delay(200);  // debounce delay
    }

    lastarmBtnState = armcurrentState;
    strcpy(gestureData.action, "arm");

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print("/t, Y: ");
    Serial.println(a.acceleration.y);
    if (armBtnCount % 2 == 0) {

      if (a.acceleration.y > 5) strcpy(gestureData.armmove, "LEFT");
      else if (a.acceleration.y < -5) strcpy(gestureData.armmove, "RIGHT");
      else if (a.acceleration.x > 5) strcpy(gestureData.armmove, "BACKWARD-S");
      else if (a.acceleration.x < -5) strcpy(gestureData.armmove, "FORWARD-S");
      else strcpy(gestureData.armmove, "STOP");
    } else {
      if (a.acceleration.y > 5) strcpy(gestureData.armmove, "OPEN");
      else if (a.acceleration.y < -5) strcpy(gestureData.armmove, "CLOSE");
      else if (a.acceleration.x > 5) strcpy(gestureData.armmove, "BACKWARD-E");
      else if (a.acceleration.x < -5) strcpy(gestureData.armmove, "FORWARD-E");
      else strcpy(gestureData.armmove, "STOP");
    }

    Serial.print("Gesture: ");
    Serial.println(gestureData.armmove);
  }

  esp_now_send(receiverMac, (uint8_t *)&gestureData, sizeof(gestureData));

  Serial.print("Action: ");
  Serial.println(gestureData.action);
}