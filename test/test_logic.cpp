/*
 * Cappa_smart - ESP32-based smart kitchen hood controller
 * Copyright (C) 2026 Matteo Cancelli
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

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

TEST(AirIndexToMsg, Calibrazione) {
  EXPECT_EQ(air_index_to_msg(25, 0), "Calibrazione...");
  EXPECT_EQ(air_index_to_msg(25, 1), "Calibrazione...");
}

TEST(AirIndexToMsg, AriaPulita) {
  EXPECT_EQ(air_index_to_msg(25, 2), "Aria pulita");
}

TEST(AirIndexToMsg, AriaOk) {
  EXPECT_EQ(air_index_to_msg(75, 2), "Aria ok");
}

TEST(AirIndexToMsg, OdoriRilevati) {
  EXPECT_EQ(air_index_to_msg(125, 2), "Odori rilevati");
}

TEST(AirIndexToMsg, AriaStantia) {
  EXPECT_EQ(air_index_to_msg(175, 2), "Aria stantia");
}

TEST(AirIndexToMsg, ApriFinestre) {
  EXPECT_EQ(air_index_to_msg(300, 2), "Apri finestre");
}

TEST(AirIndexToMsg, VentilaSubito) {
  EXPECT_EQ(air_index_to_msg(400, 2), "Ventila subito");
}

TEST(CalcolaVelocita, IaqNormale) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 50.0f, 25.0f, 3), 0);
}

TEST(CalcolaVelocita, IaqMedio) {
  EXPECT_EQ(calcola_velocita_automatica(150.0f, 50.0f, 25.0f, 3), 180);
}

TEST(CalcolaVelocita, IaqAlto) {
  EXPECT_EQ(calcola_velocita_automatica(250.0f, 50.0f, 25.0f, 3), 255);
}

TEST(CalcolaVelocita, UmiditaAlta) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 80.0f, 25.0f, 3), 180);
}

TEST(CalcolaVelocita, TemperaturaAlta) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 50.0f, 60.0f, 3), 180);
}

TEST(CalcolaVelocita, TuttoNormale) {
  EXPECT_EQ(calcola_velocita_automatica(50.0f, 50.0f, 25.0f, 3), 0);
}

TEST(CalcolaVelocita, AccuratezzaBassa) {
  EXPECT_EQ(calcola_velocita_automatica(250.0f, 50.0f, 25.0f, 1), 0);
}

TEST(CalcolaVelocita, AccuratezzaZero) {
  EXPECT_EQ(calcola_velocita_automatica(250.0f, 50.0f, 25.0f, 0), 0);
}
