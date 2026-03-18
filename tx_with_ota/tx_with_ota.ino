#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

// ================== NodeMCU ↔ RFM96 wiring ==================
// Hardware SPI on ESP8266:
// SCK  = D5 (GPIO14)
// MISO = D6 (GPIO12)
// MOSI = D7 (GPIO13)

static const int PIN_LORA_SS   = D8;  // GPIO15
static const int PIN_LORA_RST  = D1;  // GPIO5
static const int PIN_LORA_DIO0 = D2;  // GPIO4

// Existing button kept only to preserve structure (not used)
static const int PIN_BUTTON    = D3;  // GPIO0

// ================== OTA control ==================
// Safe manual OTA trigger button:
// Connect push button between GPIO3 (RX) and GND
static const int PIN_OTA_BUTTON = 3;   // GPIO3 / RX

// Built-in LED on NodeMCU is usually active LOW
static const int PIN_STATUS_LED = LED_BUILTIN;

// ================== WiFi for OTA ==================
const char* WIFI_SSID = "red";
const char* WIFI_PASS = "amir@1212";
const char* OTA_HOSTNAME = "DCI_REZLER";
const char* OTA_PASSWORD = "1234";

// ================== EEPROM ==================
static const int EEPROM_SIZE      = 8;
static const int EEPROM_SEQ_ADDR  = 0;

// ================== TX behavior ==================
static const uint8_t  TX_REPEATS        = 4;
static const uint16_t TX_REPEAT_GAP_MS  = 120;

// Persist sequence across reset/deep sleep wake
static uint32_t seq = 0;

// Kept only to preserve structure (not used)
static uint32_t lastPressMs = 0;

static bool otaMode = false;
static unsigned long lastLedToggleMs = 0;
static bool ledState = false;
static uint8_t lastOtaPercent = 255;

// ---------- LED helpers ----------
static void ledOn() {
  digitalWrite(PIN_STATUS_LED, LOW);   // active LOW
}

static void ledOff() {
  digitalWrite(PIN_STATUS_LED, HIGH);  // active LOW
}

static void blinkLedInOtaMode() {
  unsigned long now = millis();
  if (now - lastLedToggleMs >= 250) {
    lastLedToggleMs = now;
    ledState = !ledState;
    digitalWrite(PIN_STATUS_LED, ledState ? LOW : HIGH);
  }
}

// ---------- EEPROM helpers ----------
static uint32_t loadSeq() {
  uint32_t value = 0;
  EEPROM.get(EEPROM_SEQ_ADDR, value);

  if (value == 0xFFFFFFFFUL) {
    value = 0;
  }
  return value;
}

static void saveSeq(uint32_t value) {
  EEPROM.put(EEPROM_SEQ_ADDR, value);
  EEPROM.commit();
}

// ---------- Radio send ----------
static bool sendOnePacket(const char* msg) {
  LoRa.beginPacket();
  LoRa.print(msg);
  int ret = LoRa.endPacket();
  return (ret == 1);
}

static void sendBlinkCommand() {
  seq++;
  saveSeq(seq);

  char msg[32];
  snprintf(msg, sizeof(msg), "BLINK,%lu", (unsigned long)seq);

  Serial.print("Prepared: ");
  Serial.println(msg);

  for (uint8_t i = 0; i < TX_REPEATS; i++) {
    bool ok = sendOnePacket(msg);

    Serial.print("TX ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(TX_REPEATS);
    Serial.print(" -> ");
    Serial.println(ok ? "OK" : "FAIL");

    delay(TX_REPEAT_GAP_MS);
  }

  Serial.print("Sent final: ");
  Serial.println(msg);
}

// ---------- OTA mode check ----------
static bool shouldEnterOtaMode() {
  pinMode(PIN_OTA_BUTTON, INPUT_PULLUP);
  delay(20);  // small settle time

  // Button pressed -> pin reads LOW
  if (digitalRead(PIN_OTA_BUTTON) == LOW) {
    delay(30);
    if (digitalRead(PIN_OTA_BUTTON) == LOW) {
      return true;
    }
  }
  return false;
}

// ---------- OTA setup ----------
static void startOtaMode() {
  otaMode = true;

  Serial.println();
  Serial.println("OTA button detected.");
  Serial.println("Entering OTA mode...");

  ledOff();
  delay(100);
  ledOn();
  delay(100);
  ledOff();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    blinkLedInOtaMode();
  }

  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
    ledOn();
    lastOtaPercent = 255;
  });

  ArduinoOTA.onEnd([]() {
    Serial.println();
    Serial.println("OTA End");
    ledOff();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint8_t percent = (progress * 100) / total;

    // Print only when value changes by 10% to reduce spam
    if (lastOtaPercent == 255 || percent >= lastOtaPercent + 10 || percent == 100) {
      Serial.print("OTA Progress: ");
      Serial.print(percent);
      Serial.println("%");
      lastOtaPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print("OTA Error[");
    Serial.print((int)error);
    Serial.print("]: ");

    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } else {
      Serial.println("Unknown Error");
    }
  });

  ArduinoOTA.begin();

  Serial.println("OTA Ready");
  Serial.print("Hostname: ");
  Serial.println(OTA_HOSTNAME);
  Serial.print("Password: ");
  Serial.println(OTA_PASSWORD);
  Serial.println("Select the network port in Arduino IDE and upload wirelessly.");
  Serial.println("Board will remain in OTA mode until reset/power cycle.");
}

// ---------- Normal LoRa operation ----------
static void runNormalOperation() {
  // Power saving: disable WiFi fully
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  EEPROM.begin(EEPROM_SIZE);
  seq = loadSeq();

  Serial.println();
  Serial.println("Starting LoRa TX (RFM96W 433MHz)...");
  Serial.print("Last stored seq = ");
  Serial.println(seq);

  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring / power / frequency.");
    delay(100);
    ESP.deepSleep(0);
  }

  // Must match receiver
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(20);

  Serial.println("Radio configured.");
  Serial.println("Sending command now...");

  sendBlinkCommand();

  Serial.println("Going to deep sleep now. Wake again by RESET button.");
  delay(200);

  ESP.deepSleep(0);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_STATUS_LED, OUTPUT);
  ledOff();

  // Existing structure preserved
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  if (shouldEnterOtaMode()) {
    startOtaMode();
    return;
  }

  runNormalOperation();
}

void loop() {
  if (otaMode) {
    ArduinoOTA.handle();
    blinkLedInOtaMode();
  }
}