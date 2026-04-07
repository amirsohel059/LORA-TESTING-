#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>

// ============================================================
// Build / behavior switches
// ============================================================
#define ENABLE_LOGGING  1   //To enable all logs
// #define ENABLE_LOGGING  0   //To disable all logs
#define LOG_BAUD        9600
// #define LOG_BAUD        115200

// ============================================================
// Logging macros
// ============================================================
#if ENABLE_LOGGING
  #define LOG_BEGIN()         do { Serial.begin(LOG_BAUD); } while (0)
  #define LOG_FLUSH()         do { Serial.flush(); } while (0)
  #define LOG_NL()            do { Serial.println(); } while (0)
  #define LOG_PRINT(x)        do { Serial.print(x); } while (0)
  #define LOG_PRINTLN(x)      do { Serial.println(x); } while (0)
  #define LOG_PRINT_HEX(x)    do { Serial.print((x), HEX); } while (0)
  #define LOG_PRINTLN_HEX(x)  do { Serial.println((x), HEX); } while (0)
#else
  #define LOG_BEGIN()         do {} while (0)
  #define LOG_FLUSH()         do {} while (0)
  #define LOG_NL()            do {} while (0)
  #define LOG_PRINT(x)        do {} while (0)
  #define LOG_PRINTLN(x)      do {} while (0)
  #define LOG_PRINT_HEX(x)    do {} while (0)
  #define LOG_PRINTLN_HEX(x)  do {} while (0)
#endif

// ================== NodeMCU ↔ RFM96 wiring ==================
// Hardware SPI on ESP8266:
// SCK  = D5 (GPIO14)
// MISO = D6 (GPIO12)
// MOSI = D7 (GPIO13)

static const int PIN_LORA_SS   = D8;  // GPIO15
static const int PIN_LORA_RST  = D1;  // GPIO5
static const int PIN_LORA_DIO0 = D2;  // GPIO4

// Existing user button / trigger pin kept as-is
// static const int PIN_BUTTON    = D3;  // GPIO0

// ================== OTA control ==================
// Hold GPIO3 (RX) LOW during power-up/reset to enter OTA mode
static const int PIN_OTA_BUTTON = 3;             // GPIO3 / RX
static const int PIN_STATUS_LED = LED_BUILTIN;   // active LOW on most NodeMCU boards

// ================== WiFi / OTA ==================
const char* WIFI_SSID    = "red";
const char* WIFI_PASS    = "amir@1212";
const char* OTA_HOSTNAME = "REZLER_TX";
const char* OTA_PASSWORD = "1234";

// ================== EEPROM ==================
static const int EEPROM_SIZE      = 8;
static const int EEPROM_SEQ_ADDR  = 0;

// ================== TX behavior ==================
// Reduced for lower latency while keeping one backup send
static const uint8_t  TX_REPEATS       = 2;
static const uint16_t TX_REPEAT_GAP_MS = 15;

// Persist sequence across reset / wake
static uint32_t seq = 0;

// ================== DCI protocol ==================
static const uint8_t DCI_MAGIC1  = 0x44;   // 'D'
static const uint8_t DCI_MAGIC2  = 0x43;   // 'C'
static const uint8_t DCI_VERSION = 0x01;
static const uint8_t DCI_PAIR_ID = 0x11;

static const uint8_t DCI_CMD_TRIGGER   = 0x01;
static const uint8_t DCI_CMD_HEARTBEAT = 0x02;
static const uint8_t DCI_CMD_ACK       = 0x03;

struct __attribute__((packed)) DciPacketV1 {
  uint8_t  magic1;
  uint8_t  magic2;
  uint8_t  version;
  uint8_t  pairId;
  uint32_t seq;
  uint8_t  cmd;
  uint8_t  flags;
  uint16_t reserved;
};

static_assert(sizeof(DciPacketV1) == 12, "Unexpected DciPacketV1 size");

// ===== build mark =====
static const char kSig[] PROGMEM = "asx-98";

// ================== OTA state ==================
static bool otaModeActive = false;
static uint32_t otaLedLastToggleMs = 0;
static bool otaLedState = false;

// ---------- LED helpers ----------
static void setStatusLed(bool on) {
  digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);
}

static void blinkStatusLedTask(uint32_t intervalMs) {
  uint32_t now = millis();
  if ((uint32_t)(now - otaLedLastToggleMs) >= intervalMs) {
    otaLedLastToggleMs = now;
    otaLedState = !otaLedState;
    setStatusLed(otaLedState);
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

// ---------- Packet logging ----------
static void printPacket(const DciPacketV1& pkt) {
#if ENABLE_LOGGING
  LOG_PRINT("Packet: pairId=0x");
  LOG_PRINT_HEX(pkt.pairId);
  LOG_PRINT(" seq=");
  LOG_PRINT(pkt.seq);
  LOG_PRINT(" cmd=");
  LOG_PRINT(pkt.cmd);
  LOG_PRINT(" flags=0x");
  LOG_PRINT_HEX(pkt.flags);
  LOG_PRINT(" reserved=0x");
  LOG_PRINTLN_HEX(pkt.reserved);
#endif
}

// ---------- Packet send ----------
static bool sendOnePacket(const DciPacketV1& pkt) {
  LoRa.beginPacket();
  size_t written = LoRa.write((const uint8_t*)&pkt, sizeof(pkt));

  // Keep blocking send. This is intentional:
  // the device sleeps immediately after sending, so async TX is unsafe here.
  int ret = LoRa.endPacket();

  return (written == sizeof(pkt) && ret == 1);
}

static void sendTriggerCommand() {
  seq++;
  saveSeq(seq);

  DciPacketV1 pkt;
  pkt.magic1   = DCI_MAGIC1;
  pkt.magic2   = DCI_MAGIC2;
  pkt.version  = DCI_VERSION;
  pkt.pairId   = DCI_PAIR_ID;
  pkt.seq      = seq;
  pkt.cmd      = DCI_CMD_TRIGGER;
  pkt.flags    = 0;
  pkt.reserved = 0;

  printPacket(pkt);

  for (uint8_t i = 0; i < TX_REPEATS; i++) {
    bool ok = sendOnePacket(pkt);

#if ENABLE_LOGGING
    LOG_PRINT("TX ");
    LOG_PRINT(i + 1);
    LOG_PRINT("/");
    LOG_PRINT(TX_REPEATS);
    LOG_PRINT(" -> ");
    LOG_PRINTLN(ok ? "OK" : "FAIL");
#endif

    if ((i + 1) < TX_REPEATS) {
      delay(TX_REPEAT_GAP_MS);
    }
  }

  LOG_PRINTLN("Trigger packet send complete.");
}

// ---------- OTA helpers ----------
static void initPins() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  pinMode(PIN_OTA_BUTTON, INPUT_PULLUP);
  // pinMode(PIN_BUTTON, INPUT_PULLUP);
}

static bool shouldEnterOtaMode() {
  // Fast exit in normal case: no extra wait unless button is actually pressed
  if (digitalRead(PIN_OTA_BUTTON) != LOW) {
    return false;
  }

  // Deliberate hold to avoid accidental OTA entry
  const uint32_t holdMs = 900;
  uint32_t startMs = millis();

  while ((uint32_t)(millis() - startMs) < holdMs) {
    if (digitalRead(PIN_OTA_BUTTON) != LOW) {
      setStatusLed(false);
      return false;
    }
    blinkStatusLedTask(120);
    delay(10);
  }

  setStatusLed(false);
  return true;
}

static void startOtaMode() {
  otaModeActive = true;

  LOG_NL();
  LOG_PRINTLN("OTA trigger detected.");
  LOG_PRINTLN("Entering OTA mode...");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(OTA_HOSTNAME);     // station hostname for AP/router client list
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  LOG_PRINT("Connecting to WiFi");

  uint32_t lastDotMs = 0;
  uint32_t lastStatusMs = millis();

  while (WiFi.status() != WL_CONNECTED) {
    blinkStatusLedTask(150);

    uint32_t now = millis();
    if ((uint32_t)(now - lastDotMs) >= 250) {
      lastDotMs = now;
      LOG_PRINT(".");
    }

    if ((uint32_t)(now - lastStatusMs) >= 10000) {
      lastStatusMs = now;
      LOG_NL();
      LOG_PRINTLN("Still trying to connect to WiFi...");
    }

    delay(10);
    yield();
  }

  LOG_NL();
  LOG_PRINTLN("WiFi connected.");
  LOG_PRINT("IP: ");
  LOG_PRINTLN(WiFi.localIP());
  LOG_PRINT("WiFi hostname: ");
  LOG_PRINTLN(OTA_HOSTNAME);

  ArduinoOTA.setHostname(OTA_HOSTNAME);

  if (OTA_PASSWORD != nullptr && OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    setStatusLed(true);
    LOG_PRINTLN("OTA update started.");
  });

  ArduinoOTA.onEnd([]() {
    setStatusLed(false);
    LOG_NL();
    LOG_PRINTLN("OTA update finished.");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    blinkStatusLedTask(80);

#if ENABLE_LOGGING
    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - lastPrintMs) >= 1000) {
      lastPrintMs = now;
      unsigned int percent = (total > 0) ? ((progress * 100U) / total) : 0;
      LOG_PRINT("OTA progress: ");
      LOG_PRINT(percent);
      LOG_PRINTLN("%");
    }
#endif
  });

  ArduinoOTA.onError([](ota_error_t error) {
    setStatusLed(false);

#if ENABLE_LOGGING
    LOG_PRINT("OTA error: ");
    LOG_PRINTLN((int)error);

    if (error == OTA_AUTH_ERROR) {
      LOG_PRINTLN("Auth failed");
    } else if (error == OTA_BEGIN_ERROR) {
      LOG_PRINTLN("Begin failed");
    } else if (error == OTA_CONNECT_ERROR) {
      LOG_PRINTLN("Connect failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      LOG_PRINTLN("Receive failed");
    } else if (error == OTA_END_ERROR) {
      LOG_PRINTLN("End failed");
    }
#endif
  });

  ArduinoOTA.begin();

  LOG_PRINTLN("OTA ready.");
  LOG_PRINT("Hostname: ");
  LOG_PRINTLN(OTA_HOSTNAME);
  LOG_PRINTLN("LED will blink while OTA mode is active.");
}

static void handleOtaMode() {
  ArduinoOTA.handle();
  blinkStatusLedTask(300);
  yield();
}

// ---------- Normal LoRa operation ----------
static void runNormalOperation() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);   // keep this one; ESP8266 needs a tiny settle time here

  EEPROM.begin(EEPROM_SIZE);
  seq = loadSeq();

  LOG_NL();
  LOG_PRINTLN("Starting LoRa TX (RFM96W 433MHz)...");
  LOG_PRINT("Last stored seq = ");
  LOG_PRINTLN(seq);
  LOG_PRINT("DCI pair ID = 0x");
  LOG_PRINTLN_HEX(DCI_PAIR_ID);

  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    LOG_PRINTLN("LoRa begin failed! Check wiring / power / frequency.");
    LOG_FLUSH();
    ESP.deepSleep(0);
    return;
  }

  // Must match current low-latency receiver profile
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(17);

  LOG_PRINTLN("Radio configured.");
  LOG_PRINTLN("Sending command now...");

  sendTriggerCommand();

  LOG_PRINTLN("Going to deep sleep now.");
  LOG_FLUSH();
  ESP.deepSleep(0);
}

void setup() {
  LOG_BEGIN();
  initPins();

  if (shouldEnterOtaMode()) {
    startOtaMode();
    return;
  }

  runNormalOperation();
}

void loop() {
  if (otaModeActive) {
    handleOtaMode();
  }
}