#include "include/Globals.h"

void setup()
{
  pinMode(GPIO_FAN_PWM, OUTPUT);
  digitalWrite(GPIO_FAN_PWM, LOW);

  init_gpio();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  system_state.fan_mutex = xSemaphoreCreateMutex();

  init_display();
  init_environment_sensor(&system_state);

  setup_wifi();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  Serial.println("Avvio task...");

  xTaskCreate(task_environment_sensor, "EnvSensor",  4096, &system_state, 2, NULL);
  xTaskCreate(task_display_update,     "Display",     4096, &system_state, 1, NULL);
  xTaskCreate(task_motion_sensor,      "MotionSensor",4096, &system_state, 3, NULL);
  xTaskCreate(task_fan_control,        "FanControl",  4096, &system_state, 2, NULL);
  xTaskCreate(task_mode_button,        "ModeButton",  4096, &system_state, 3, NULL);
  xTaskCreate(task_mqtt,               "MQTT",        4096, &system_state, 2, NULL);

  digitalWrite(GPIO_FAN_ENABLE, HIGH);
}

void loop()
{
  // vuoto: tutto gestito da FreeRTOS
}
