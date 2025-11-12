#include "include/Globals.h"

void setup(){
  setup_GPIO();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  fan_mutex = xSemaphoreCreateMutex();
  
  check_display();
  check_bme();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150);

  setup_wifi();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  Serial.println("Inizio Task");

  // Task sensori e display
  xTaskCreate(task_bme, "BME688", 4096, NULL, 2, NULL);
  xTaskCreate(task_display, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(task_pir, "PIR", 4096, NULL, 3, NULL);
  
  // Task ventola: separazione responsabilità
  xTaskCreate(task_logica_automatica_ventola, "LogicaAuto", 4096, NULL, 1, NULL);
  xTaskCreate(task_controllo_manuale_velocita, "PotenzioMetro", 4096, NULL, 1, NULL);
  xTaskCreate(task_toggle_mode, "ToggleMode", 4096, NULL, 1, NULL);
  xTaskCreate(task_attuatore_ventola, "AttuatoreVentola", 4096, NULL, 2, NULL);

  xTaskCreate(task_mqtt_publish, "MQTT_Publish", 4096, NULL, 2, NULL);
}

void loop(){
  // Vuoto: tutto gestito da FreeRTOS
}