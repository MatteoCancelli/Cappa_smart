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
| Ventola | Noctua NF-P14s redux 1500 PWM - 12 V, 2.40 W (0.20 A) | GPIO32 PWM, GPIO33 Tachimetro |
| Relay | SRD-05VDC-SL-C (collegamento con lampadina 220v) | GPIO5 |
| Step-Up | MT3608 | 5 V -> 12 V |
| Breadboard + jumper | - | - |
| Modulo alimentazione breadboard | Output 3.3 V e 5 V | - |
| Potenziometro | B10K | GPIO36 (ADC1 perché ADC2 condiviso con WiFi) |
| Bottone | - | GPIO18 |
| LED + Resistenza | LED + 1 kΩ | GPIO19 |
| MOSFET | 2N7000 N-channel — interruttore GND ventola | Drain -> GND ventola, Source -> GND, Gate -> GPIO33 |
| Resistenza pull-down | 10 kΩ | Gate 2N7000 -> GND (tiene ventola spenta durante il boot) |
| Condensatore elettrolitico | 4700 µF, 16 V - bulk lato 5 V (ingresso step-up) | tra 5 V e GND, vicino al MT3608 |
| Condensatore elettrolitico | 1000 µF, 25 V - bulk lato 12 V (uscita step-up) | tra 12 V e GND, vicino alla ventola |
| Condensatore elettrolitico | 100 µF, 10 V - stabilizzazione rail 3.3 V | tra 3.3 V e GND, vicino all'uscita del modulo alimentazione |
| Condensatori ceramici bypass | 3 100 nF, 50 V (MLCC "104") | uno in parallelo a ciascun elettrolitico |

**Note sul codice e best practice**
- Variabili globali: il progetto dichiara `extern` negli header e le definisce una sola volta in `Globals.cpp`. Questo evita errori di linker e riduce duplicazioni.
- FreeRTOS: i task usano `vTaskDelay`, `xSemaphoreTake` e simili. Includere `freertos/task.h` e `freertos/semphr.h` solo nei .cpp dove si usano le API.
- MQTT: `mqttClient` è definito in `Globals.cpp` e dichiarato `extern` in `include/TaskMQTT.h`.
- ADC: usare pin ADC1 (32..39) per letture analogiche se il WiFi è attivo.
- Buffer MQTT: il codice usa buffer locali; se hai payload grandi valuta l'aumento di `MQTT_MAX_PACKET_SIZE` nella libreria PubSubClient.
- Ventola: soft-start PWM implementato in `attuatore_ventola()` - rampa graduale di 5 step ogni 30ms per evitare picchi di corrente e brown-out dell'ESP32.
- Boot: il 2N7000 con pull-down 10kΩ sul Gate tiene la ventola fisicamente spenta durante il boot. `GPIO_FAN_ENABLE` viene portato HIGH solo a fine setup, dopo WiFi e inizializzazione completa.

**Licenza**
- (Aggiungi qui la licenza che preferisci, es. MIT)