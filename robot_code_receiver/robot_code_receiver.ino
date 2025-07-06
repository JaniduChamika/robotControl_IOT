#define BLYNK_TEMPLATE_ID "TMPL6ZTn_Zsqu"
#define BLYNK_TEMPLATE_NAME "GestureCar"
#define BLYNK_AUTH_TOKEN "k4bcB7giggrXs-e-yD1LjBGjOFYfamWk"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Blynk configuration

const char* WIFI_SSID = "Galaxy M31";
const char* WIFI_PASS = "12345678";
const int analogPin = 34;

float globalDistance = 0.0;
const int trigPin = 23;
const int echoPin = 22;
/* ---------- Wi‑Fi first ---------- */
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

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if (connectWiFi()) {
    xTaskCreatePinnedToCore(blynkTask, "Blynk", 4096, NULL, 2, NULL, 0);
    xTaskCreate(batteryMonitorTask, "Battery Monitor", 4096, NULL, 1, NULL);
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
      Blynk.virtualWrite(V0, batteryPercent);
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
      Blynk.virtualWrite(V1, globalDistance);
    }
    Serial.print(" V\distance %: ");
    Serial.println(globalDistance);
    vTaskDelay(50 / portTICK_PERIOD_MS);  // 20 Hz update
  }
}
void loop() {
  // other robot logic (motors / gesture) here
}