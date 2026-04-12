#include "include/Globals.h"
#include "include/Logic.h"
#include <Adafruit_BME680.h>


void check_bme(){
  if (!bme.begin(0x76)) {
    Serial.println("Errore: BME688 non trovato!");
    bme_error_msg();
    while (true);
  }
  Serial.println("BME688 trovato!");
  bme_ok_msg();
  delay(1000);
}

void task_bme(void *pvParameters){
  for(;;) {
    if (!bme.performReading()) {
      Serial.println("Lettura fallita!");
      bme_fail_msg();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else {
      temp = bme.temperature;
      hum  = bme.humidity;
      gas_index = gas_to_AirQualityIndex(bme.gas_resistance);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}
