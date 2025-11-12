#pragma once
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "include/Globals.h"
#include "config.h"

// Topic MQTT
#define TOPIC_TEMP "home/bme688/temperature"
#define TOPIC_HUM "home/bme688/humidity"
#define TOPIC_GAS "home/bme688/gas_index"
#define TOPIC_AIR_QUALITY "home/bme688/air_quality"
#define TOPIC_FAN_MODE "home/fan/mode"
#define TOPIC_FAN_SPEED "home/fan/speed"
#define TOPIC_FAN_RPM "home/fan/rpm"
#define TOPIC_PIR "home/pir/motion"

// Client MQTT
extern WiFiClient espClient;
extern PubSubClient mqttClient;

void setup_wifi();
void reconnect_mqtt();
void task_mqtt_publish(void *pvParameters);