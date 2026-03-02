#include <SPI.h>
#include <LoRa.h>

// ===== ESP32-C3 Super Mini pin mapping (from your image) =====
static const int PIN_LORA_SCK  = 4;   // GPIO4  (SCK)
static const int PIN_LORA_MISO = 5;   // GPIO5  (MISO)
static const int PIN_LORA_MOSI = 6;   // GPIO6  (MOSI)
static const int PIN_LORA_SS   = 7;   // GPIO7  (SS / CS)

static const int PIN_LORA_RST  = 3;   // GPIO3  (reset)
static const int PIN_LORA_DIO0 = 2;   // GPIO2  (DIO0)

static const int PIN_LED       = 21;  // GPIO21 (TX on your image) -> LED+res -> GND

// LED ON time after receiving command
static const uint32_t LED_ON_MS = 5000;
static uint32_t ledOffAtMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.println("LoRa RX (RFM96W 433MHz) starting...");

  // Start SPI with explicit pins (important on ESP32-C3)
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);

  // Tell LoRa library which control pins we use
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  // Start LoRa at 433 MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring/power/frequency.");
    while (true) delay(1000);
  }

  // Match transmitter settings (you used SF10, BW125k, CR4/5, CRC)
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(10);       // must match TX if TX changed it
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println("LoRa RX ready. Waiting for packets...");
}

static void handlePacket(const String& s) {
  // Expect: "BLINK,<seq>"
  if (s.startsWith("BLINK,")) {
    Serial.print("Command OK: ");
    Serial.println(s);

    // Turn LED ON for 5 seconds
    digitalWrite(PIN_LED, HIGH);
    ledOffAtMs = millis() + LED_ON_MS;
  } else {
    Serial.print("Unknown packet: ");
    Serial.println(s);
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg;
    msg.reserve(64);

    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }

    Serial.print("RX: ");
    Serial.print(msg);
    Serial.print(" | RSSI=");
    Serial.print(LoRa.packetRssi());
    Serial.print(" | SNR=");
    Serial.println(LoRa.packetSnr());

    handlePacket(msg);
  }

  // LED timeout handling
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    digitalWrite(PIN_LED, LOW);
    ledOffAtMs = 0;
    Serial.println("LED OFF");
  }
}