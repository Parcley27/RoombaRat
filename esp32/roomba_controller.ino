/**
 * roomba_controller.ino
 *
 * ESP32 Freenove Wroover – Roomba 600 OI bridge over WiFi.
 *
 * Wiring:
 *   GPIO 4  → Roomba Mini-DIN pin 5 (BRC)
 *   GPIO 25 → Roomba Mini-DIN pin 4 (Roomba TXD  → ESP32 RX)
 *   GPIO 26 → Roomba Mini-DIN pin 3 (Roomba RXD  ← ESP32 TX)
 *   GND     → Roomba Mini-DIN pin 6 or 7 (GND)
 *
 * The laptop connects via TCP to this device on port 8080.
 * Sensor data is streamed at ~20 Hz; drive commands are accepted any time.
 */

#include <Arduino.h>
#include <WiFi.h>

// ─── WiFi ──────────────────────────────────────────────────────────────────
// ** CHANGE THESE **
const char* WIFI_SSID = "The wifi";
const char* WIFI_PASS = "6394701868";
const uint16_t TCP_PORT = 8080;

// ─── Pin assignments ───────────────────────────────────────────────────────
#define BRC_PIN  4
#define RX_PIN  25   // GPIO25 ← Roomba TXD (pin 4)
#define TX_PIN  26   // GPIO26 → Roomba RXD (pin 3)

// ─── Roomba Open Interface opcodes ────────────────────────────────────────
#define OI_START        128
#define OI_RESET        7
#define OI_SAFE         131
#define OI_FULL         132
#define OI_DRIVE_DIRECT 145
#define OI_QUERY_LIST   149

// ─── Sensor packet IDs ────────────────────────────────────────────────────
// Packet | Bytes | Content
#define PKT_BUMPS_DROPS  7   // 1 – bits: bumpR=0, bumpL=1, dropR=2, dropL=3
#define PKT_WALL         8   // 1 – IR wall sensor (right side)
#define PKT_CLIFF_L      9   // 1
#define PKT_CLIFF_FL    10   // 1
#define PKT_CLIFF_FR    11   // 1
#define PKT_CLIFF_R     12   // 1
#define PKT_OVERCURRENT 14   // 1 – motor overcurrent bits
#define PKT_LEFT_ENC    43   // 2 unsigned – counts since last read (wraps at 65535)
#define PKT_RIGHT_ENC   44   // 2 unsigned

static const uint8_t QUERY_IDS[]   = { PKT_BUMPS_DROPS, PKT_WALL,
                                        PKT_CLIFF_L, PKT_CLIFF_FL,
                                        PKT_CLIFF_FR, PKT_CLIFF_R,
                                        PKT_OVERCURRENT,
                                        PKT_LEFT_ENC, PKT_RIGHT_ENC };
static const uint8_t N_QUERY       = sizeof(QUERY_IDS);
static const uint8_t RESPONSE_BYTES = 1+1+1+1+1+1+1+2+2;  // 11

// ─── Globals ───────────────────────────────────────────────────────────────
HardwareSerial roombaSer(1);
WiFiServer     tcpServer(TCP_PORT);
WiFiClient     client;

uint32_t lastSensorMs  = 0;
uint32_t lastSafeMs    = 0;
const uint32_t SENSOR_PERIOD_MS = 50;   // 20 Hz
const uint32_t SAFE_RENEW_MS   = 5000; // re-assert SAFE every 5 s (cliff recovery)

// ─── Roomba helpers ───────────────────────────────────────────────────────

void wakeRoomba() {
  pinMode(BRC_PIN, OUTPUT);
  digitalWrite(BRC_PIN, HIGH);
  delay(200);
  digitalWrite(BRC_PIN, LOW);   // low pulse wakes it from sleep
  delay(600);
  digitalWrite(BRC_PIN, HIGH);
  delay(200);
}

void roombaSend(const uint8_t* buf, size_t len) {
  roombaSer.write(buf, len);
}

void initRoomba() {
  wakeRoomba();
  delay(1000);
  uint8_t seq[] = { OI_START, OI_SAFE };
  roombaSend(seq, sizeof(seq));
  delay(200);
  Serial.println("[Roomba] OI started, Safe mode active");
}

void driveRoomba(int16_t leftMmS, int16_t rightMmS) {
  leftMmS  = constrain(leftMmS,  -500, 500);
  rightMmS = constrain(rightMmS, -500, 500);
  uint8_t cmd[5] = {
    OI_DRIVE_DIRECT,
    (uint8_t)((rightMmS >> 8) & 0xFF), (uint8_t)(rightMmS & 0xFF),
    (uint8_t)((leftMmS  >> 8) & 0xFF), (uint8_t)(leftMmS  & 0xFF)
  };
  roombaSend(cmd, 5);
}

struct Sensors {
  uint8_t  bumpsDrops;
  uint8_t  wall;
  uint8_t  cliffL, cliffFL, cliffFR, cliffR;
  uint8_t  overcurrent;
  uint16_t leftEnc, rightEnc;
  bool     valid;
};

Sensors querySensors() {
  Sensors s = {};

  // Flush stale data
  while (roombaSer.available()) roombaSer.read();

  // Request query list
  roombaSer.write(OI_QUERY_LIST);
  roombaSer.write(N_QUERY);
  for (uint8_t i = 0; i < N_QUERY; i++) roombaSer.write(QUERY_IDS[i]);

  // Wait with timeout
  uint32_t deadline = millis() + 80;
  while (roombaSer.available() < RESPONSE_BYTES && millis() < deadline)
    delayMicroseconds(200);

  if (roombaSer.available() < RESPONSE_BYTES) {
    s.valid = false;
    return s;
  }

  s.bumpsDrops  = roombaSer.read();
  s.wall        = roombaSer.read();
  s.cliffL      = roombaSer.read();
  s.cliffFL     = roombaSer.read();
  s.cliffFR     = roombaSer.read();
  s.cliffR      = roombaSer.read();
  s.overcurrent = roombaSer.read();
  s.leftEnc     = ((uint16_t)roombaSer.read() << 8) | roombaSer.read();
  s.rightEnc    = ((uint16_t)roombaSer.read() << 8) | roombaSer.read();
  s.valid       = true;
  return s;
}

// ─── Command parser ───────────────────────────────────────────────────────

void handleLine(const String& line) {
  if (line.startsWith("DRIVE ")) {
    // "DRIVE <left_mms> <right_mms>"
    int sp = line.indexOf(' ', 6);
    if (sp < 0) return;
    int16_t L = (int16_t)line.substring(6, sp).toInt();
    int16_t R = (int16_t)line.substring(sp + 1).toInt();
    driveRoomba(L, R);
  } else if (line == "STOP") {
    driveRoomba(0, 0);
  } else if (line == "SAFE") {
    uint8_t cmd = OI_SAFE;
    roombaSend(&cmd, 1);
  } else if (line == "FULL") {
    uint8_t cmd = OI_FULL;
    roombaSend(&cmd, 1);
  }
}

// ─── Arduino entry points ─────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  roombaSer.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.printf("Connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
  }
  Serial.printf("\nIP address: %s\n", WiFi.localIP().toString().c_str());

  initRoomba();
  tcpServer.begin();
  Serial.printf("TCP server listening on :%u\n", TCP_PORT);
}

void loop() {
  // Accept new client
  if (!client || !client.connected()) {
    WiFiClient incoming = tcpServer.accept();
    if (incoming) {
      if (client) { client.stop(); driveRoomba(0, 0); }
      client = incoming;
      client.setNoDelay(true);
      Serial.println("[TCP] Client connected");
    }
    return;
  }

  // Read and parse incoming commands (newline-delimited)
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length()) handleLine(line);
  }

  uint32_t now = millis();

  // Periodically re-assert Safe mode (Roomba drops to Passive after cliff)
  if (now - lastSafeMs >= SAFE_RENEW_MS) {
    lastSafeMs = now;
    uint8_t cmd = OI_SAFE;
    roombaSend(&cmd, 1);
  }

  // Stream sensor data at 20 Hz
  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    Sensors s = querySensors();
    if (s.valid) {
      char buf[96];
      // Format: "S <bumpsDrops> <wall> <cliffL> <cliffFL> <cliffFR> <cliffR> <overcurrent> <leftEnc> <rightEnc>\n"
      snprintf(buf, sizeof(buf), "S %u %u %u %u %u %u %u %u %u\n",
               s.bumpsDrops, s.wall,
               s.cliffL, s.cliffFL, s.cliffFR, s.cliffR,
               s.overcurrent,
               s.leftEnc, s.rightEnc);
      client.print(buf);
    }
  }
}
