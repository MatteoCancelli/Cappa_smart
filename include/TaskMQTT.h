#pragma once
#include <WiFi.h>
#include <PubSubClient.h>
#include "include/Globals.h"
#include "include/SystemState.h"
#include "config.h"

#define TOPIC_TEMP        "home/bme680/temperature"
#define TOPIC_HUM         "home/bme680/humidity"
#define TOPIC_GAS         "home/bme680/air_quality_pct"
#define TOPIC_IAQ_RAW     "home/bme680/iaq_score"
#define TOPIC_AIR_QUALITY "home/bme680/air_quality_msg"
#define TOPIC_IAQ_ACCURACY "home/bme680/iaq_accuracy"
#define TOPIC_FAN_MODE    "home/fan/mode"
#define TOPIC_FAN_SPEED   "home/fan/speed"
#define TOPIC_PIR         "home/pir/motion"

extern WiFiClient espClient;
extern PubSubClient mqttClient;

void setup_wifi();
void reconnect_mqtt();
void task_mqtt(void* state);
