#include <SPI.h>
#include <LoRa.h>

// ===== ESP32-C3 Super Mini pin mapping =====
static const int PIN_LORA_SCK  = 4;   // GPIO4
static const int PIN_LORA_MISO = 5;   // GPIO5
static const int PIN_LORA_MOSI = 6;   // GPIO6
static const int PIN_LORA_SS   = 7;   // GPIO7

static const int PIN_LORA_RST  = 3;   // GPIO3
static const int PIN_LORA_DIO0 = 2;   // GPIO2

// Relay connected on same pin where LED was connected
static const int PIN_RELAY     = 21;  // GPIO21

// ===== Relay logic =====
// Most relay modules are ACTIVE LOW:
// LOW  = relay ON
// HIGH = relay OFF
// static const uint8_t RELAY_ON_LEVEL  = LOW;
// static const uint8_t RELAY_OFF_LEVEL = HIGH;

static const uint8_t RELAY_ON_LEVEL  = HIGH;
static const uint8_t RELAY_OFF_LEVEL = LOW;

// Relay ON time after valid command
static const uint32_t RELAY_ON_MS = 2000;
static uint32_t relayOffAtMs = 0;

// Duplicate protection
static uint16_t lastSeq = 0;
static bool haveLastSeq = false;

// Safety: ignore malformed / too long packets
static const size_t MAX_MSG_LEN = 32;

// ---------- Relay helpers ----------
static void relayOff() {
  digitalWrite(PIN_RELAY, RELAY_OFF_LEVEL);
}

static void relayOn() {
  digitalWrite(PIN_RELAY, RELAY_ON_LEVEL);
}

// Parse BLINK,<seq>
static bool parseBlinkPacket(const String& s, uint16_t& seqOut) {
  if (!s.startsWith("BLINK,")) {
    return false;
  }

  String seqStr = s.substring(6);
  seqStr.trim();

  if (seqStr.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < seqStr.length(); i++) {
    if (!isDigit(seqStr[i])) {
      return false;
    }
  }

  unsigned long v = seqStr.toInt();
  if (v > 65535UL) {
    return false;
  }

  seqOut = (uint16_t)v;
  return true;
}

static void handlePacket(const String& s, int rssi, float snr) {
  uint16_t seq = 0;

  if (!parseBlinkPacket(s, seq)) {
    Serial.print("Ignored invalid packet: ");
    Serial.println(s);
    return;
  }

  // Ignore immediate duplicates caused by repeated TX sends
  if (haveLastSeq && seq == lastSeq) {
    Serial.print("Duplicate seq ignored: ");
    Serial.println(seq);
    return;
  }

  lastSeq = seq;
  haveLastSeq = true;

  Serial.print("Valid command: ");
  Serial.print(s);
  Serial.print(" | RSSI=");
  Serial.print(rssi);
  Serial.print(" | SNR=");
  Serial.println(snr);

  relayOn();
  relayOffAtMs = millis() + RELAY_ON_MS;

  Serial.println("RELAY ON");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Make relay safe immediately at boot
  pinMode(PIN_RELAY, OUTPUT);
  relayOff();

  Serial.println();
  Serial.println("LoRa RX Relay (RFM96W 433MHz) starting...");

  // Start SPI with explicit pins
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);

  // Tell LoRa library which pins are used
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  // Start LoRa at 433 MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring / power / frequency.");
    while (true) {
      delay(1000);
    }
  }

  //   // Match transmitter settings (you used SF10, BW125k, CR4/5, CRC)
  // LoRa.enableCrc();
  // LoRa.setSpreadingFactor(10);       // must match TX if TX changed it
  // LoRa.setSignalBandwidth(125E3);
  // LoRa.setCodingRate4(5);


  // ===== Longer range settings =====
  // IMPORTANT:
  // Apply the SAME settings on transmitter too.
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);        // higher sensitivity, slower link
  LoRa.setSignalBandwidth(62.5E3);    // narrower BW = better sensitivity, slower
  LoRa.setCodingRate4(5);             // keep simple / compatible
  LoRa.setPreambleLength(12);         // a bit more robust
  LoRa.setSyncWord(0x12);             // keep both sides same

  // Optional: max TX power is set on transmitter side, not receiver side

  Serial.println("LoRa RX ready. Waiting for packets...");
  Serial.println("Relay default state: OFF");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg;
    msg.reserve(MAX_MSG_LEN);

    while (LoRa.available()) {
      char c = (char)LoRa.read();

      // Prevent very long / garbage packets
      if (msg.length() < MAX_MSG_LEN) {
        msg += c;
      }
    }

    msg.trim();

    Serial.print("RX raw: ");
    Serial.println(msg);

    handlePacket(msg, LoRa.packetRssi(), LoRa.packetSnr());
  }

  // Relay timeout handling
  if (relayOffAtMs != 0 && (int32_t)(millis() - relayOffAtMs) >= 0) {
    relayOff();
    relayOffAtMs = 0;
    Serial.println("RELAY OFF");
  }
}