#include "include/Globals.h"
#include "include/BME680.h"
#include "include/Logic.h"
#include <bsec.h>

Bsec bsec;

void check_bme()
{
  bsec.begin(BME68X_I2C_ADDR_LOW, Wire);

  if (bsec.bsecStatus != BSEC_OK)
  {
    Serial.println("Errore: BME680 non trovato!");
    bme_error_msg();
    while (true);
  }

  bsec_virtual_sensor_t sensor_list[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
  };

  bsec.updateSubscription(sensor_list, 3, BSEC_SAMPLE_RATE_LP);

  Serial.println("BME680 + BSEC OK!");
  bme_ok_msg();
  delay(1000);
}

void task_bme(void *pvParameters)
{
  for (;;)
  {
    if (bsec.run())
    {
      temp      = bsec.temperature;
      hum       = bsec.humidity;
      gas_index = bsec.iaq;
    }
    else if (bsec.bsecStatus != BSEC_OK || bsec.bme68xStatus != BME68X_OK)
    {
      Serial.printf("Errore BSEC: %d BME: %d\n", bsec.bsecStatus, bsec.bme68xStatus);
      bme_fail_msg();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}