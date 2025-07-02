#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>       // required for OTA
#include "BasicOTA.hpp" // required for OTA
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
}
#include <AsyncMqttClient.h>
#include <secrets.h>

// address we will assign if dual sensor is present
#define LOX0_ADDRESS 0x30
#define LOX1_ADDRESS 0x31
// #define LOX2_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX0 5
#define SHT_LOX1 18
// #define SHT_LOX2 19

#define SDA_PIN 17
#define SCL_PIN 16

#define MQTT_PORT 1883

// objects for the vl53l0x
Adafruit_VL53L0X lox0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure0;
VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;

BasicOTA OTA; // required for OTA

const char *ssid = ssid_name;         // required for OTA
const char *password = ssid_password; // required for OTA

const char *debugTopic = secret_debugTopic;
const char *waterTopic = secret_waterTopic;
const char *coffeeTopic = secret_coffeeTopic;

int waterLevel;
int coffeeLevel;

String waterStr;
String coffeeStr;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer; // required for OTA

bool mqttConnected = false;
TickType_t disconnectTick = 0; // Track disconnection time

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0);
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // uint16_t packetIdSub = mqttClient.subscribe(subtopic, 2);
  // Serial.print("Subscribing at QoS 2, packetId: ");
  // Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  mqttConnected = false;
  disconnectTick = xTaskGetTickCount();
  Serial.println("Disconnected from MQTT.");
  Serial.print("Disconnect Tick: ");
  Serial.println(disconnectTick);

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{

  Serial.println("Message received:");
  Serial.printf("Topic: %s\n", topic);
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void setID()
{
  // all reset
  digitalWrite(SHT_LOX0, LOW);
  digitalWrite(SHT_LOX1, LOW);
  // digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, HIGH);
  // digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX0 LOX2
  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, LOW);
  // digitalWrite(SHT_LOX2, LOW);

  // initing LOX0
  if (!lox0.begin(LOX0_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX1 and resetting LOX0 LOX2
  digitalWrite(SHT_LOX1, HIGH);
  // digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // // activating LOX2
  // digitalWrite(SHT_LOX2, HIGH);
  // delay(10);

  // // initing LOX2
  // if (!lox2.begin(LOX2_ADDRESS))
  // {
  //   Serial.println(F("Failed to boot second VL53L0X"));
  //   while (1)
  //     ;
  // }
}

void readSensor()
{
  lox0.rangingTest(&measure0, false);
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  // lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // yg kepasang sekarang, coffee 0, water 1 sensornya
  waterLevel = measure1.RangeMilliMeter;
  coffeeLevel = measure0.RangeMilliMeter;

  // print sensor one reading
  Serial.print(F("0: "));
  if (measure0.RangeStatus != 4)
  { // if not out of range
    Serial.print(measure0.RangeMilliMeter);
  }
  else
  {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));
  // print sensor one reading
  Serial.print(F("1: "));
  if (measure1.RangeStatus != 4)
  { // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  }
  else
  {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  // Serial.print(F("2: "));
  // if (measure2.RangeStatus != 4)
  // {
  //   Serial.print(measure2.RangeMilliMeter);
  // }
  // else
  // {
  //   Serial.print(F("Out of range"));
  // }

  Serial.println();
}

void readSensorTask(void *parameter)
{
  readSensor();
  // String waterStr = String(waterLevel);
  // String coffeeStr = String(coffeeLevel);

  // mqttClient.publish(waterTopic, 2, false, waterStr.c_str());
  // mqttClient.publish(coffeeTopic, 2, false, coffeeStr.c_str());

  vTaskDelay(60000 / portTICK_PERIOD_MS);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // wait until serial port opens for native USB devices
  while (!Serial)
  {
    delay(1);
  }

  pinMode(SHT_LOX0, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  // pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX0, LOW);
  digitalWrite(SHT_LOX1, LOW);
  // digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));

  Serial.println(F("Starting..."));
  setID();

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  mqttReconnectTimer = xTimerCreate(
      "mqttTimer",
      pdMS_TO_TICKS(2000),
      pdFALSE, (void *)0,
      reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

  wifiReconnectTimer = xTimerCreate(
      "wifiTimer",
      pdMS_TO_TICKS(2000),
      pdFALSE, (void *)0,
      reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(mqttusername, mqttpassword);

  connectToWifi();
  // OTA.begin();
  // xTaskCreate(readSensorTask, "readSensorTask", 2048, NULL, 1, NULL);
}

void loop()
{
  // OTA.handle();
  readSensor();
  String waterStr = String(waterLevel);
  String coffeeStr = String(coffeeLevel);

  mqttClient.publish(waterTopic, 2, false, waterStr.c_str());
  mqttClient.publish(coffeeTopic, 2, false, coffeeStr.c_str());
  delay(60000);
}