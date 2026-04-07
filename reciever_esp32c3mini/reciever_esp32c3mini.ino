#include <SPI.h>
#include <LoRa.h>

// ============================================================
// Build / behavior switches
// ============================================================
#define ENABLE_LOGGING  1     //enable all logs, change:
// #define ENABLE_LOGGING  0  	  //disable all logs, change:
#define LOG_BAUD        115200

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

// ===== ESP32-C3 Super Mini pin mapping =====
static const int PIN_LORA_SCK  = 4;   // GPIO4
static const int PIN_LORA_MISO = 5;   // GPIO5
static const int PIN_LORA_MOSI = 6;   // GPIO6
static const int PIN_LORA_SS   = 7;   // GPIO7

static const int PIN_LORA_RST  = 3;   // GPIO3
static const int PIN_LORA_DIO0 = 2;   // GPIO2

static const int PIN_RELAY     = 21;  // GPIO21

// ===== Relay logic =====
static const uint8_t RELAY_ON_LEVEL  = HIGH;
static const uint8_t RELAY_OFF_LEVEL = LOW;

// Relay ON time after valid command
static const uint32_t RELAY_ON_MS = 1000;
static uint32_t relayOffAtMs = 0;

// ================== DCI protocol ==================
static const uint8_t DCI_MAGIC1  = 0x44;   // 'D'
static const uint8_t DCI_MAGIC2  = 0x43;   // 'C'
static const uint8_t DCI_VERSION = 0x01;

// Must match TX
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

// subtle internal build mark
static const char kBuildMark[] = "asx-98";

// Duplicate / old packet protection
static uint32_t lastSeq = 0;
static bool haveLastSeq = false;

// ---------- Relay helpers ----------
static void relayOff() {
  digitalWrite(PIN_RELAY, RELAY_OFF_LEVEL);
}

static void relayOn() {
  digitalWrite(PIN_RELAY, RELAY_ON_LEVEL);
}

// ---------- Packet validation ----------
static bool isPacketValidBasic(const DciPacketV1& pkt) {
  if (pkt.magic1 != DCI_MAGIC1 || pkt.magic2 != DCI_MAGIC2) {
    LOG_PRINTLN("Rejected: bad magic");
    return false;
  }

  if (pkt.version != DCI_VERSION) {
    LOG_PRINT("Rejected: unsupported version ");
    LOG_PRINTLN(pkt.version);
    return false;
  }

  if (pkt.pairId != DCI_PAIR_ID) {
    LOG_PRINT("Rejected: pairId mismatch. RX expects 0x");
    LOG_PRINT_HEX(DCI_PAIR_ID);
    LOG_PRINT(", got 0x");
    LOG_PRINTLN_HEX(pkt.pairId);
    return false;
  }

  if (pkt.cmd != DCI_CMD_TRIGGER) {
    LOG_PRINT("Rejected: unsupported cmd ");
    LOG_PRINTLN(pkt.cmd);
    return false;
  }

  return true;
}

static bool isPacketNew(uint32_t seq) {
  if (!haveLastSeq) {
    return true;
  }

  if (seq <= lastSeq) {
    LOG_PRINT("Rejected: duplicate/old seq ");
    LOG_PRINT(seq);
    LOG_PRINT(" (last=");
    LOG_PRINT(lastSeq);
    LOG_PRINTLN(")");
    return false;
  }

  return true;
}

static void printPacket(const DciPacketV1& pkt, int rssi, float snr) {
  LOG_PRINT("Valid packet: pairId=0x");
  LOG_PRINT_HEX(pkt.pairId);
  LOG_PRINT(" seq=");
  LOG_PRINT(pkt.seq);
  LOG_PRINT(" cmd=");
  LOG_PRINT(pkt.cmd);
  LOG_PRINT(" flags=0x");
  LOG_PRINT_HEX(pkt.flags);
  LOG_PRINT(" reserved=0x");
  LOG_PRINT_HEX(pkt.reserved);
  LOG_PRINT(" | RSSI=");
  LOG_PRINT(rssi);
  LOG_PRINT(" | SNR=");
  LOG_PRINTLN(snr);
}

static void handlePacket(const DciPacketV1& pkt, int rssi, float snr) {
  if (!isPacketValidBasic(pkt)) {
    return;
  }

  if (!isPacketNew(pkt.seq)) {
    return;
  }

  lastSeq = pkt.seq;
  haveLastSeq = true;

  printPacket(pkt, rssi, snr);

  relayOn();
  relayOffAtMs = millis() + RELAY_ON_MS;

  LOG_PRINTLN("RELAY ON");
}

void setup() {
  LOG_BEGIN();

  pinMode(PIN_RELAY, OUTPUT);
  relayOff();

  LOG_NL();
  LOG_PRINTLN("LoRa RX Relay (RFM96W 433MHz) starting...");
  LOG_PRINT("DCI pair ID = 0x");
  LOG_PRINTLN_HEX(DCI_PAIR_ID);

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    LOG_PRINTLN("LoRa begin failed! Check wiring / power / frequency.");
    LOG_FLUSH();

    // only fatal fallback delay to avoid watchdog issues
    while (true) {
      delay(1);
    }
  }

  // Must match transmitter exactly
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);

  LOG_PRINTLN("LoRa RX ready. Waiting for packets...");
  LOG_PRINTLN("Relay default state: OFF");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    LOG_PRINT("Packet received, size=");
    LOG_PRINTLN(packetSize);

    if (packetSize != (int)sizeof(DciPacketV1)) {
      LOG_PRINT("Rejected: unexpected packet size ");
      LOG_PRINT(packetSize);
      LOG_PRINT(", expected ");
      LOG_PRINTLN(sizeof(DciPacketV1));

      while (LoRa.available()) {
        LoRa.read();
      }
      return;
    }

    DciPacketV1 pkt;
    uint8_t* p = (uint8_t*)&pkt;
    size_t idx = 0;

    while (LoRa.available() && idx < sizeof(DciPacketV1)) {
      p[idx++] = (uint8_t)LoRa.read();
    }

    if (idx != sizeof(DciPacketV1)) {
      LOG_PRINTLN("Rejected: incomplete packet read");
      return;
    }

    handlePacket(pkt, LoRa.packetRssi(), LoRa.packetSnr());
  }

  if (relayOffAtMs != 0 && (int32_t)(millis() - relayOffAtMs) >= 0) {
    relayOff();
    relayOffAtMs = 0;
    LOG_PRINTLN("RELAY OFF");
  }
}