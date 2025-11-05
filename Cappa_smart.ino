#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String msg_mod = "";

// BME688 a indirizzo 0x76 (o 0x77 se hai collegato ADDR a VCC)
Adafruit_BME680 bme;

// Funzione per convertire resistenza gas in Air Quality Index (0-100)
float gas_to_AirQualityIndex(double gasOhm) {
  const double GAS_MIN = 10000.0;   // 10 kΩ = aria pessima
  const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

  if (gasOhm < GAS_MIN) gasOhm = GAS_MIN;
  if (gasOhm > GAS_MAX) gasOhm = GAS_MAX;

  int aqi = (int)((gasOhm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100);
  return aqi;
}

// Funzione che converte AQI (%) in messaggio leggibile
String air_index_to_msg(float g) {
  if (g < 20)      return "Apri tutto";     // pessima
  else if (g < 40) return "Aria stantia";    // scarsa
  else if (g < 60) return "Aria viziata";   // media
  else if (g < 80) return "Aria normale";   // buona
  else             return "Aria fresca";    // ottima
}

void visualizza_msg_scorrevole(String msg){
  if (msg.length() > 11){
    Serial.println(msg_mod);
    Serial.println("------------------");
    display.print(msg_mod);
    msg_mod.remove(0,1);
  }
  else{
    display.print(msg);
  }
}

void check_display(){
  Serial.println("=== Avvio Display ===");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Errore display");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Display OK");
  display.display();

  delay(1000);
}

void check_bme(){
  if (!bme.begin(0x76)) {
    Serial.println("Errore: BME688 non trovato!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ERRORE BME688");
    display.setCursor(0, 16);
    display.println("Controlla I2C!");
    display.display();
    while (true);
  }
  Serial.println("BME688 trovato!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BME688 OK");
  display.display();
  delay(1000);
}

void setup(){
  Serial.begin(115200);
  Wire.begin(); //I2C
  Wire.setClock(100000);

  check_display();

  check_bme();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(300, 150);
}

// void loop(){

// }

void loop() {
  if (!bme.performReading()) {
    Serial.println("⚠️ Lettura fallita!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Lettura fallita!");
    display.display();
    delay(1000);
    return;
  }

  float t = bme.temperature;
  float h = bme.humidity;
  float g = gas_to_AirQualityIndex(bme.gas_resistance);
  String msg = air_index_to_msg(g);   
  if (msg_mod.length() < 10)
    msg_mod = msg; 

  Serial.print("Temperatura:  "); Serial.print(t); Serial.println(" °C");
  Serial.print("Umidità:      "); Serial.print(h); Serial.println(" %");
  Serial.print("Qualità aria: "); Serial.print(g); Serial.print(" %"); 
  Serial.println(msg);
  Serial.println("------------------");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Temperatura:  "); display.print(t, 1); display.println(" C");
  display.setCursor(0, 12);
  display.print("Umidita:      "); display.print(h, 1); display.println(" %");
  display.setCursor(0, 24);
  display.print("Qualita aria: "); display.print(g, 1); display.println(" %");

  display.setCursor(0, 48);
  display.setTextSize(2);
  visualizza_msg_scorrevole(msg);


  display.display();

  delay(2000);
}
