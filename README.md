# Cappa_smart

Progetto per ESP32 che legge un sensore BME688, gestisce una ventola (modalità manuale/automatica), un sensore PIR e pubblica i dati via MQTT.

**Caratteristiche**
- Lettura temperatura/umidità/gas con BME688
- Display OLED (SSD1306) per stato
- Controllo ventola PWM con logica automatica e controllo manuale (potenziometro)
- Rilevamento movimento (PIR) per illuminazione
- Pubblicazione dati su MQTT
- Architettura a task FreeRTOS

**File importanti**
- `Cappa_smart.ino` — sketch principale, crea task e inizializza periferiche
- `include/Globals.h` — macro GPIO/PWM e dichiarazioni `extern` (variabili condivise)
- `Globals.cpp` — definizioni concrete delle variabili globali (display, bme, mutex, mqtt client, ecc.)
- `include/*.h` — header modulare: `Display.h`, `BME680.h`, `Aspiration.h`, `TaskMQTT.h`, `PIR.h`
- `*.cpp` — implementazioni dei task: `Display.cpp`, `BME680.cpp`, `Aspiration.cpp`, `TaskMQTT.cpp`, `PIR.cpp`
- `config.h` — credenziali WiFi e parametri MQTT (NO COMMIT)

**Hardware richiesto**
| Componente | Modello / Specifica | Wiring |
|---|---|---|
| Microcontrollore | ESP32-WROOM-32D | - |
| Sensore ambientale | BME680 (o BME688 compatibile) | SDA -> GPIO21, SCL -> GPIO22 |
| Sensore movimento | PIR Sensor | GPIO23 |
| Display | SSD1306 128x64 (I2C) | SDA -> GPIO21, SCL -> GPIO22 |
| Ventola | Noctua NF-P14s redux 1500 PWM — 12 V, 2.40 W (0.20 A) | GPIO32 PWM, GPIO33 Tachimetro |
| Relay | SRD-05VDC-SL-C (collegamento con lampadina 220v)| GPIO5 |
| Step-Up | MT3608 | 5 V -> 12 V |
| Breadboard + jumper | - | - |
| Modulo alimentazione breadboard | Output 3.3 V e 5 V | - | 
| Potenziometro | B10K | GPIO36 (ADC1 perché ADC2 condiviso con WiFi)|
| LED + Resistenza | LED + 1 kΩ | GPIO5 |
| Condensatore | 2200 µF, 25 V (per stabilizzare alimentazione ventola e ridurre reset ESP) | 5 V |

**Note sul codice e best practice**
- Variabili globali: il progetto dichiara `extern` negli header e le definisce una sola volta in `Globals.cpp`. Questo evita errori di linker e riduce duplicazioni.
- FreeRTOS: i task usano `vTaskDelay`, `xSemaphoreTake` e simili. Includere `freertos/task.h` e `freertos/semphr.h` solo nei .cpp dove si usano le API.
- MQTT: `mqttClient` è definito in `Globals.cpp` e dichiarato `extern` in `include/TaskMQTT.h`.
- ADC: usare pin ADC1 (32..39) per letture analogiche se il WiFi è attivo.
- Buffer MQTT: il codice usa buffer locali; se hai payload grandi valuta l'aumento di `MQTT_MAX_PACKET_SIZE` nella libreria PubSubClient.

**Licenza**
- (Aggiungi qui la licenza che preferisci, es. MIT)

