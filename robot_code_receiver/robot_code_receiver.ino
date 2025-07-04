#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>

typedef struct struct_message {
  char carmove[20];
  char action[20];
  char armmove[20];
} struct_message;

struct_message gestureData;

//LEFT
int IN1 = 16;
int IN2 = 17;
int ENA = 4;
//RIGHT
int IN3 = 18;
int IN4 = 19;
int ENB = 5;

//arm pins
const int legServoPin = 27;  // You can use any PWM-capable GPIO
const int soulderServoPin = 26;
const int elboServoPin = 32;
const int gripServoPin = 33;

//ultrasonic sensor pin
const int trigPin = 23;
const int echoPin = 22;
const int servorpin = 13;
Servo ultrasonicSev;
//for arms
Servo legServo;
Servo SoulderServo;
Servo elboServo;
Servo gripServo;
// arm initial position
int legPos = 90;
int soulderPos = 80;
int elbowPos = 90;
int grapperPos = 2;

int ultraPos = 90;
//for battery percentage
const int analogPin = 34;
const float voltageDividerRatio = 4.74;

//for light
const int armLight = 25;
const int carLight = 14;

float globalDistance = 0.0;

unsigned long lastReceivedTime = 0;
bool isConnected = false;
#define TIMEOUT 500  // 2 seconds
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  memcpy(&gestureData, data, sizeof(gestureData));

  lastReceivedTime = millis();
  isConnected = true;

  String gesture = String(gestureData.carmove);
  String armGesture = String(gestureData.armmove);
  String action = String(gestureData.action);
  Serial.print("Action : ");
  Serial.print(action);

  if (action == "car") {
    Serial.print("\t Received Gesture: ");
    Serial.println(gesture);
    if (gesture == "R-225") {
      turnRight(100);
    } else if (gesture == "L-225") {
      turnLeft(100);
    } else if (gesture == "B-225") {
      moveBackward(100);
    } else if (gesture == "F-225") {
      moveForward(100);
    } else if (gesture == "R-150") {
      turnRight(100);
    } else if (gesture == "L-150") {
      turnLeft(100);
    } else if (gesture == "B-150") {
      moveBackward(100);
    } else if (gesture == "F-150") {
      moveForward(100);
    } else if (gesture == "R-125") {
      turnRight(100);
    } else if (gesture == "L-125") {
      turnLeft(100);
    } else if (gesture == "B-125") {
      moveBackward(100);
    } else if (gesture == "F-125") {
      moveForward(100);
    } else if (gesture == "R-100") {
      turnRight(80);
    } else if (gesture == "L-100") {
      turnLeft(80);
    } else if (gesture == "B-100") {
      moveBackward(80);
    } else if (gesture == "F-100") {
      moveForward(80);
    } else if (gesture == "R-80") {
      turnRight(50);
    } else if (gesture == "L-80") {
      turnLeft(50);
    } else if (gesture == "B-80") {
      moveBackward(50);
    } else if (gesture == "F-80") {
      moveForward(50);
    } else if (gesture == "STOP") {
      stopMotors();
    }
  } else if (action == "arm") {
    stopMotors();
    Serial.print("\t leg: ");
    Serial.print(legPos);
    Serial.print("\t soulder: ");
    Serial.print(soulderPos);
    Serial.print("\t elbo: ");
    Serial.print(elbowPos);
    Serial.print("\t grip: ");
    Serial.println(grapperPos);
    Serial.print("\t Received Gesture: ");
    Serial.println(armGesture);
    if (armGesture == "LEFT") {
      turnArmRight();
    } else if (armGesture == "RIGHT") {
      turnArmLeft();
    } else if (armGesture == "BACKWARD-S") {
      moveArmBackwardS();
    } else if (armGesture == "FORWARD-S") {
      moveArmForwardS();
    } else if (armGesture == "OPEN") {
      openGripper();
    } else if (armGesture == "CLOSE") {
      closeGripper();
    } else if (armGesture == "BACKWARD-E") {
      moveArmBackwardE();
    } else if (armGesture == "FORWARD-E") {
      moveArmForwardE();
    }
  } else {
    stopMotors();
  }
}

void setup() {
  Serial.begin(115200);
  //for car
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  //for ultrasonic sensro
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  //for arm
  legServo.setPeriodHertz(50);
  legServo.attach(legServoPin);

  SoulderServo.setPeriodHertz(50);
  SoulderServo.attach(soulderServoPin);

  elboServo.setPeriodHertz(50);
  elboServo.attach(elboServoPin);

  gripServo.setPeriodHertz(50);
  gripServo.attach(gripServoPin);
  //for ultrasonic servor
  ultrasonicSev.setPeriodHertz(50);
  ultrasonicSev.attach(servorpin);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");

    return;
  }
  // Start the battery monitor task
  xTaskCreate(
    batteryMonitorTask,
    "Battery Monitor",
    2048,
    NULL,
    1,
    NULL);

  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP-NOW Receiver Ready");
}

void loop() {
  if (millis() - lastReceivedTime > TIMEOUT && isConnected) {
    stopMotors();
    isConnected = false;
    Serial.println("ESP-NOW disconnect");
  }

  delay(100);  // small delay for efficiency
}
void batteryMonitorTask(void *pvParameters) {
  while (true) {
    int rawADC = analogRead(analogPin);
    float voltage = (rawADC / 4095.0) * 3.3 * voltageDividerRatio;

    int batteryPercent = getBatteryPercent(voltage);

    Serial.print("Battery Voltage: ");
    Serial.print(voltage, 2);
    Serial.print(" V\tBattery %: ");
    Serial.println(batteryPercent);

    vTaskDelay(60000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}


float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);  // 30ms timeout (~5 meters)
  float distance = duration * 0.034 / 2;


  // globalDistance = distance;

  Serial.print("Distance: ");
  Serial.print(globalDistance);
  Serial.println(" cm");

  return distance;
}


void moveForward(int speed) {
  ultrasonicSev.write(90);
  if (getDistance() > 30) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
  } else {
    stopMotors();
  }
}

void moveBackward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  ultraSonicRight();
  if (getDistance() > 30) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
  } else {
    stopMotors();
  }
}

void turnLeft(int speed) {
  ultraSonicLeft();
  if (getDistance() > 30) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
  } else {
    stopMotors();
  }
}

void stopMotors() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void ultraSonicRight() {

  ultrasonicSev.write(30);
}
void ultraSonicLeft() {

  ultrasonicSev.write(150);
}
int getBatteryPercent(float voltage) {
  if (voltage >= 16.8) return 100;
  if (voltage <= 12.0) return 0;
  return (int)((voltage - 12.0) * 100 / (16.8 - 12.0));
}

void turnArmRight() {
  if (legPos < 180) {
    legPos++;
    legServo.write(legPos);
    delay(30);
  }
}
void turnArmLeft() {
  if (legPos > 0) {
    legPos--;
    legServo.write(legPos);
    delay(30);
  }
}

void moveArmBackwardS() {
  if (soulderPos > 80) {
    soulderPos--;
    SoulderServo.write(soulderPos);
    delay(20);
  }
}
void moveArmForwardS() {
  if (soulderPos < 174) {
    soulderPos++;
    SoulderServo.write(soulderPos);
    delay(20);
  }
}
void moveArmBackwardE() {
  if (elbowPos < 180) {
    elbowPos++;
    elboServo.write(elbowPos);
    delay(20);
  }
}
void moveArmForwardE() {
  if (elbowPos > 90) {
    elbowPos--;
    elboServo.write(elbowPos);
    delay(20);
  }
}
void openGripper() {

  if (grapperPos < 58) {
    grapperPos++;
    gripServo.write(grapperPos);
    delay(20);
  }
}
void closeGripper() {
  if (grapperPos > 2) {
    grapperPos--;
    gripServo.write(grapperPos);
    delay(20);
  }
}
