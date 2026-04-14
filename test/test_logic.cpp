#include <gtest/gtest.h>
#include "include/Logic.h"

TEST(IaqToPercentage, AriaOttima) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(0), 100.0f);
}

TEST(IaqToPercentage, AriaPessima) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(500), 0.0f);
}

TEST(IaqToPercentage, SogliaEccellente) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(50), 80.0f);
}

TEST(IaqToPercentage, SogliaBuona) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(100), 60.0f);
}

TEST(IaqToPercentage, SogliaViziata) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(150), 40.0f);
}

TEST(IaqToPercentage, SogliaStantia) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(200), 25.0f);
}

TEST(IaqToPercentage, SogliaDannosa) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(350), 10.0f);
}

TEST(IaqToPercentage, ClampSopra) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(600), 0.0f);
}

TEST(IaqToPercentage, ClampSotto) {
  EXPECT_FLOAT_EQ(iaq_to_percentage(-10), 100.0f);
}

TEST(AirIndexToMsg, AriaPulita) {
  EXPECT_EQ(air_index_to_msg(25), "Aria pulita");
}

TEST(AirIndexToMsg, AriaOk) {
  EXPECT_EQ(air_index_to_msg(75), "Aria ok");
}

TEST(AirIndexToMsg, OdoriRilevati) {
  EXPECT_EQ(air_index_to_msg(125), "Odori rilevati");
}

TEST(AirIndexToMsg, AriaStantia) {
  EXPECT_EQ(air_index_to_msg(175), "Aria stantia");
}

TEST(AirIndexToMsg, ApriFinestre) {
  EXPECT_EQ(air_index_to_msg(300), "Apri finestre");
}

TEST(AirIndexToMsg, VentilaSubito) {
  EXPECT_EQ(air_index_to_msg(400), "Ventila subito");
}

TEST(CalcolaVelocita, IaqNormale) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 50.0f, 25.0f), 0);
}

TEST(CalcolaVelocita, IaqMedio) {
  EXPECT_EQ(calcola_velocita_automatica(150.0f, 50.0f, 25.0f), 180);
}

TEST(CalcolaVelocita, IaqAlto) {
  EXPECT_EQ(calcola_velocita_automatica(250.0f, 50.0f, 25.0f), 255);
}

TEST(CalcolaVelocita, UmiditaAlta) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 80.0f, 25.0f), 180);
}

TEST(CalcolaVelocita, TemperaturaAlta) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 50.0f, 60.0f), 180);
}

TEST(CalcolaVelocita, TuttoNormale) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 50.0f, 25.0f), 0);
}