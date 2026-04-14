#include "include/Logic.h"

int calcola_velocita_automatica(float iaq, float humidity, float temperature)
{
  if (humidity > 75.0f || temperature > 50.0f) return 180;
  if (iaq <= 100.0f) return 0;
  if (iaq <= 200.0f) return 180;
  return 255;
}

static float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float iaq_to_percentage(float iaq)
{
  if (iaq < 0)   iaq = 0;
  if (iaq > 500) iaq = 500;

  if      (iaq <= 50)  return mapf(iaq,   0,  50, 100, 80);
  else if (iaq <= 100) return mapf(iaq,  50, 100,  80, 60);
  else if (iaq <= 150) return mapf(iaq, 100, 150,  60, 40);
  else if (iaq <= 200) return mapf(iaq, 150, 200,  40, 25);
  else if (iaq <= 350) return mapf(iaq, 200, 350,  25, 10);
  else                 return mapf(iaq, 350, 500,  10,  0);
}

String air_index_to_msg(float iaq)
{
  if      (iaq <= 50)  return "Aria pulita";
  else if (iaq <= 100) return "Aria ok";
  else if (iaq <= 150) return "Odori rilevati";
  else if (iaq <= 200) return "Aria stantia";
  else if (iaq <= 350) return "Apri finestre";
  else                 return "Ventila subito";
}
