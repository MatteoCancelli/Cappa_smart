#include "include/Logic.h"

int calcola_velocita_automatica(float gas, float humidity, float temperature)
{
  bool accensione = (gas < 40.0f || humidity > 75.0f || temperature > 50.0f);
  return accensione ? 180 : 0;
}

float gas_to_AirQualityIndex(double gas_ohm)
{
  const double GAS_MIN = 10000.0;
  const double GAS_MAX = 120000.0;

  if (gas_ohm < GAS_MIN) gas_ohm = GAS_MIN;
  if (gas_ohm > GAS_MAX) gas_ohm = GAS_MAX;

  return (gas_ohm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100.0f;
}

String air_index_to_msg(float quality_index)
{
  if      (quality_index < 20) return "Apri tutto";
  else if (quality_index < 40) return "Aria stantia";
  else if (quality_index < 60) return "Aria viziata";
  else if (quality_index < 80) return "Aria normale";
  else                         return "Aria fresca";
}