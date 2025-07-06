#define BLYNK_TEMPLATE_ID "TMPL6ZTn_Zsqu"
#define BLYNK_TEMPLATE_NAME "GestureCar"
#define BLYNK_AUTH_TOKEN "k4bcB7giggrXs-e-yD1LjBGjOFYfamWk"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
// Blynk configuration

const char* WIFI_SSID = "Galaxy M31";
const char* WIFI_PASS = "12345678";
const int analogPin = 34;

float globalDistance = 0.0;

int IN1 = 16;
int IN2 = 17;
int ENA = 4;
//RIGHT
int IN3 = 18;
int IN4 = 19;
int ENB = 5;
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
/* ---------- Wi‑Fi first ---------- */

TaskHandle_t blynkHdle = NULL;
TaskHandle_t btryHdle = NULL;


volatile bool forwardPressed = false;
volatile bool backwardPressed = false;
volatile bool leftPressed = false;
volatile bool rightPressed = false;

/* -------- Arm command flags ------------------------------------------ */
volatile bool armLeftCmd = false;
volatile bool armRightCmd = false;
volatile bool armForwardSCmd = false;
volatile bool armBackwardSCmd = false;
volatile bool armForwardECmd = false;
volatile bool armBackwardECmd = false;
volatile bool gripperOpenCmd = false;
volatile bool gripperCloseCmd = false;
uint8_t CarSpeed = 100;


bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(250);
  Serial.printf("Wi‑Fi OK  (CH %d)\n", WiFi.channel());
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  return false;
}

/* ---------- Blynk service task on Core 0 ---------- */
void blynkTask(void*) {
  Blynk.config(BLYNK_AUTH_TOKEN);

  // First connection (blocking, but only once)
  Blynk.connect(5000);  // 5 s timeout

  for (;;) {
    Blynk.run();
    // Serial.println("Blynk run");               // < 10 ms
    if (!Blynk.connected()) Blynk.connect();  // auto‑reconnect
    vTaskDelay(5 / portTICK_PERIOD_MS);       // yield 5 ms
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
  if (connectWiFi()) {
    xTaskCreatePinnedToCore(blynkTask, "Blynk", 4096, NULL, 2, &blynkHdle, 0);
    xTaskCreate(batteryMonitorTask, "Battery Monitor", 4096, NULL, 1, &btryHdle);
  }

  xTaskCreatePinnedToCore(ultrasonicTask, "US", 4096, NULL, 1, NULL, 0);
}
void batteryMonitorTask(void* pvParameters) {
  Serial.print("Battery Voltage: ");
  while (true) {
    int rawADC = analogRead(analogPin);
    float voltage = (rawADC / 4095.0) * 3.3 * 4.74;

    int batteryPercent = getBatteryPercent(voltage);

    Serial.print("Battery Voltage: ");
    Serial.print(voltage, 2);
    Serial.print(" V\tBattery %: ");
    Serial.println(batteryPercent);
    if (Blynk.connected()) {
      // Blynk.virtualWrite(V0, batteryPercent);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}
int getBatteryPercent(float voltage) {
  if (voltage >= 16.8) return 100;
  if (voltage <= 12.0) return 0;
  return (int)((voltage - 12.0) * 100 / (16.8 - 12.0));
}
void ultrasonicTask(void* pv) {
  for (;;) {
    // Trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long dur = pulseIn(echoPin, HIGH, 30000);        // timeout 30 ms
    if (dur > 0) globalDistance = dur * 0.0343 / 2;  // cm
    if (Blynk.connected()) {
      // Blynk.virtualWrite(V1, globalDistance);
    }
    // Serial.print(" V\distance %: ");
    // Serial.println(globalDistance);
    vTaskDelay(50 / portTICK_PERIOD_MS);  // 20 Hz update
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (blynkHdle != NULL) {
      vTaskDelete(blynkHdle);
      blynkHdle = NULL;
      Serial.println(" deleted");
    }
  }

  BlynkOperate();
}

void BlynkOperate() {
  if (forwardPressed) {
    moveForward(CarSpeed);
  } else if (backwardPressed) {
    moveBackward(CarSpeed);
   } else if (leftPressed) {
    turnLeft(CarSpeed);
  } else if (rightPressed) {
    turnRight(CarSpeed);
  } else if (armRightCmd) {
    turnArmRight();
  } else if (armLeftCmd) {
    turnArmLeft();
  } else if (armForwardSCmd) {
    moveArmForwardS();
  } else if (armBackwardSCmd) {
    moveArmBackwardS();
  } else if (armForwardECmd) {
    moveArmForwardE();
  } else if (armBackwardECmd) {
    moveArmBackwardE();
  } else if (gripperOpenCmd) {
    openGripper();
  } else if (gripperCloseCmd) {
    closeGripper();
  } else {
        stopMotors();
  }
  // delay(500);
}
////////////////////////////////////////////
// blynk function IOT
////////////////////////////////////////////


BLYNK_WRITE(V3) {
  forwardPressed = param.asInt();
}
BLYNK_WRITE(V6) {
  backwardPressed = param.asInt();
}
BLYNK_WRITE(V4) {
  leftPressed = param.asInt();
}
BLYNK_WRITE(V5) {
  rightPressed = param.asInt();
}
BLYNK_WRITE(V8) {
  armRightCmd = param.asInt();
}
BLYNK_WRITE(V7) {
  armLeftCmd = param.asInt();
}
BLYNK_WRITE(V11) {
  armForwardSCmd = param.asInt();
}
BLYNK_WRITE(V12) {
  armBackwardSCmd = param.asInt();
}
BLYNK_WRITE(V13) {
  armForwardECmd = param.asInt();
}
BLYNK_WRITE(V14) {
  armBackwardECmd = param.asInt();
}
BLYNK_WRITE(V9) {
  gripperOpenCmd = param.asInt();
}
BLYNK_WRITE(V10) {
  gripperCloseCmd = param.asInt();
}
void moveForward(int speed) {
  ultrasonicSev.write(90);
  if (globalDistance > 30) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(10);
  } else {
    stopMotors();
  }
}

void moveBackward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  ultraSonicRight();
  if (globalDistance > 30) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

  } else {
    stopMotors();
  }
}

void turnLeft(int speed) {
  ultraSonicLeft();
  if (globalDistance > 30) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

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