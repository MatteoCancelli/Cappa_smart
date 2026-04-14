#pragma once
#include <WiFi.h>
#include <PubSubClient.h>
#include "include/Globals.h"
#include "include/SystemState.h"
#include "config.h"

#define TOPIC_TEMP        "home/bme688/temperature"
#define TOPIC_HUM         "home/bme688/humidity"
#define TOPIC_GAS         "home/bme688/air_quality_pct"
#define TOPIC_IAQ_RAW     "home/bme688/iaq_score"
#define TOPIC_AIR_QUALITY "home/bme688/air_quality_msg"
#define TOPIC_FAN_MODE    "home/fan/mode"
#define TOPIC_FAN_SPEED   "home/fan/speed"
#define TOPIC_PIR         "home/pir/motion"

extern WiFiClient espClient;
extern PubSubClient mqttClient;

void setup_wifi();
void reconnect_mqtt();
void task_mqtt(void* state);
