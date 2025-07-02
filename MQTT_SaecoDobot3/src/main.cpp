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
#include <ArduinoJson.h> // Tambahkan untuk parsing JSON
#include <secrets.h>

#define MQTT_PORT 1883

BasicOTA OTA; // required for OTA

// GPIO sisa 32, 33, 34, 35
const int DobotDO1 = 32;
const int DobotDO2 = 33;
const int pins[] = {5, 16, 17, 18, 19, 21, 22, 23};
const int numPins = sizeof(pins) / sizeof(pins[0]);

const int DobotDI1 = pins[0];     // Start Coffee (GPIO 5)
const int DobotDI2 = pins[3];     // Coffee Done (GPIO 18)
const int espressoPin = pins[1];  // Relay Espresso (GPIO 16) // PB2
const int americanoPin = pins[2]; // Relay Americano (GPIO 17) // PB0

const char *ssid = ssid_name;         // required for OTA
const char *password = ssid_password; // required for OTA

const char *subtopic = secret_subtopic;
const char *pubtopic = secret_pubtopic; // Topik untuk publish status
const char *debugTopic = secret_debugTopic;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer; // required for OTA

bool mqttConnected = false;
TickType_t disconnectTick = 0; // Track disconnection time

bool brewingEspresso = false;
bool brewingAmericano = false;
String currentOrderId = ""; // Simpan orderId

long currentMillis = 0;
long brewStartMillis = 0;
int coffeeStart = 1;
bool waitingForDobot = false;
int currentCoffeeType = 0; 

bool publishCoffeeDone = false;
unsigned long doneMillis = 0;


void espresso()
{
  Serial.println("Making espresso");
  digitalWrite(espressoPin, LOW);
  delay(200);
  digitalWrite(espressoPin, HIGH);
  brewStartMillis = millis();
  brewingEspresso = true;
  Serial.println("Brew start millis: " + String(brewStartMillis));
}

void americano()
{
  Serial.println("Making americano");
  digitalWrite(americanoPin, LOW);
  delay(200);
  digitalWrite(americanoPin, HIGH);
  brewStartMillis = millis();
  mqttClient.publish(pubtopic, 2, false, String(brewStartMillis).c_str());
  brewingAmericano = true;
  Serial.println("Brew start millis: " + String(brewStartMillis));
}

void dobotStart(int coffeeType)
{
  Serial.println("Commanding Dobot to Start");

  digitalWrite(DobotDI1, LOW);
  delay(200);
  digitalWrite(DobotDI1, HIGH);

  currentCoffeeType = coffeeType;
  waitingForDobot = true;
}

void dobotCoffeeDone()
{
  Serial.println("Coffee done");
  digitalWrite(DobotDI2, LOW);
  delay(200);
  digitalWrite(DobotDI2, HIGH);
}

void updateCoffee()
{
  currentMillis = millis();
  mqttClient.publish(pubtopic, 2, false, String(currentMillis).c_str());
  // Espresso brewing finished, trigger Dobot pickup and set delay
  if (brewingEspresso && millis() - brewStartMillis >= 50000 && !publishCoffeeDone)
  {
    dobotCoffeeDone();
    Serial.println("Espresso done at millis: " + String(millis()));
    publishCoffeeDone = true;
    doneMillis = millis(); // start delay countdown
    mqttClient.publish(pubtopic, 2, false, String(doneMillis).c_str());
  }

  // Americano brewing finished
  if (brewingAmericano && millis() - brewStartMillis >= 60000 && !publishCoffeeDone)
  {
    dobotCoffeeDone();
    Serial.println("Americano done at millis: " + String(millis()));
    publishCoffeeDone = true;
    doneMillis = millis(); // reuse same timer
    mqttClient.publish(pubtopic, 2, false, String(doneMillis).c_str());
  }

  if (publishCoffeeDone && millis() - doneMillis >= 10000)
  {
    mqttClient.publish(pubtopic, 2, false, String(doneMillis).c_str());
    String jsonMessage = "{\"orderId\":\"" + currentOrderId + "\",\"robotStatus\":\"done\"}";
    mqttClient.publish(pubtopic, 2, false, jsonMessage.c_str());
    Serial.println("Published to " + String(pubtopic) + ": " + jsonMessage);

    brewingAmericano = false;
    brewingEspresso = false;
    publishCoffeeDone = false;
    brewStartMillis = 0;
    currentOrderId = "";
  }
}


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
  uint16_t packetIdSub = mqttClient.subscribe(subtopic, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
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

  // Parse JSON payload
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, len);

  if (error)
  {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  int coffeeType = doc["coffeeType"];
  currentOrderId = doc["orderId"].as<String>();
  Serial.printf("CoffeeType: %d, OrderId: %s\n", coffeeType, currentOrderId.c_str());

  if (strcmp(topic, subtopic) == 0)
  {
    if (coffeeType == 0)
    {
      dobotStart(coffeeType);
      // espresso();
    }
    else if (coffeeType == 1)
    {
      dobotStart(coffeeType);
      // americano();
    }
    else
    {
      Serial.println("Unknown coffeeType, ignoring.");
      mqttClient.publish(debugTopic, 2, false, "Invalid command.");
    }
  }
}

void updateCoffeeTask(void *parameter)
{
  while (true)
  {
    // coffeeStart = digitalRead(DobotDO1);
    // mqttClient.publish(debugTopic, 2, false, String(coffeeStart).c_str());
    updateCoffee();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void waitForDobotStartTask(void *parameter)
{
  while (true)
  {
    if (waitingForDobot)
    {
      int state = digitalRead(DobotDO1);
      mqttClient.publish(debugTopic, 2, false, ("DobotDO1: " + String(state)).c_str());

      if (state == LOW)
      {
        mqttClient.publish(debugTopic, 2, false, "Dobot ready. Starting coffee.");
        if (currentCoffeeType == 0)
        {
          espresso();
        }
        else if (currentCoffeeType == 1)
        {
          americano();
        }
        waitingForDobot = false;
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  for (int i = 0; i < numPins; i++)
  {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  pinMode(DobotDO1, INPUT);

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
  OTA.begin();
  xTaskCreate(updateCoffeeTask, "updateCoffeeTask", 2048, NULL, 1, NULL);
  xTaskCreate(waitForDobotStartTask, "waitForDobotStartTask", 2048, NULL, 1, NULL);
}

void loop()
{
  OTA.handle();
}