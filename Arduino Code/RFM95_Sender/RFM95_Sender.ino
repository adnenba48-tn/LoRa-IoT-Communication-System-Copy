
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

#define LORA_FREQUENCY 868E6

// DHT11 configuration
#define DHTPIN A0
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// LoRa configuration
const long frequency = LORA_FREQUENCY;
int spreadingFactor = 7;
int txPower = 17;
long bandwidth = 125E3;

// Sending configuration
bool autoSend = true;
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;

// Statistics
unsigned long sent = 0;
unsigned long received = 0;

int lastRssi = 0;
float lastSnr = 0;

// Serial commands
char line[48];
byte lineLength = 0;


// ======================================
// CRC16 - CCITT-FALSE
// Polynomial: 0x1021
// Initial value: 0xFFFF
// ======================================

uint16_t calculateCRC16(const char *data) {

  uint16_t crc = 0xFFFF;

  while (*data) {

    crc ^= ((uint16_t)(uint8_t)*data << 8);

    for (byte i = 0; i < 8; i++) {

      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }

    data++;
  }

  return crc;
}


// ======================================
// SETUP
// ======================================

void setup() {

  Serial.begin(9600);

  dht.begin();

  Serial.println();
  Serial.println("================================");
  Serial.println("   NODE 1 - LORA SENDER");
  Serial.println("================================");

  LoRa.setPins(10, 9, 2);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa module not responding!");
    while (true) {}
  }

  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(bandwidth);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(txPower);

  Serial.println("LoRa READY!");
  Serial.println("Frequency: 868 MHz");
  Serial.println("SF7 - BW125 kHz - CR4/5");
  Serial.println("DHT11 initialized!");
  Serial.println("CRC16: ENABLED");
  Serial.println("Type HELP for commands");

  lastSend = millis() - SEND_INTERVAL;
}


// ======================================
// SEND SENSOR DATA WITH CRC16
// ======================================

void sendSensorData() {

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT11 ERROR: Sensor reading failed!");
    return;
  }

  Serial.println();
  Serial.println("========== SENSOR DATA ==========");

  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");


  // Create original payload
  String payload = "DATA,";
  payload += String(temperature, 1);
  payload += ",";
  payload += String(humidity, 1);


  // Calculate CRC16 over payload only
  uint16_t crc = calculateCRC16(payload.c_str());


  // Convert CRC to exactly 4 hexadecimal characters
  char crcText[5];
  snprintf(crcText, sizeof(crcText), "%04X", crc);


  // Display payload and CRC
  Serial.println();
  Serial.print("Payload: ");
  Serial.println(payload);

  Serial.print("CRC16: ");
  Serial.println(crcText);


  // Send payload + comma + CRC
  LoRa.beginPacket();

  LoRa.print(payload);
  LoRa.print(",");
  LoRa.print(crcText);

  LoRa.endPacket();

  sent++;

  Serial.println();
  Serial.println("CRC16 packet sent to Node 2!");
  Serial.println("Waiting for ACK...");


  // Wait for ACK
  unsigned long startWait = millis();
  bool ackReceived = false;

  while (millis() - startWait < 2000) {

    int packetSize = LoRa.parsePacket();

    if (packetSize) {

      String response = "";

      while (LoRa.available()) {
        response += (char)LoRa.read();
      }

      lastRssi = LoRa.packetRssi();
      lastSnr = LoRa.packetSnr();

      received++;

      if (response == "ACK") {

        ackReceived = true;

        Serial.println();
        Serial.println("ACK RECEIVED!");
        Serial.println("Transmission successful!");

        Serial.print("ACK RSSI: ");
        Serial.print(lastRssi);
        Serial.println(" dBm");

        Serial.print("ACK SNR: ");
        Serial.print(lastSnr);
        Serial.println(" dB");

        break;
      }
    }
  }

  if (!ackReceived) {

    Serial.println();
    Serial.println("ACK TIMEOUT!");
    Serial.println("No ACK received from Node 2.");
  }

  Serial.println("================================");
}


// ======================================
// DISPLAY STATUS
// ======================================

void status() {

  Serial.println();
  Serial.println("========== NODE 1 STATUS ==========");

  Serial.print("Frequency: ");
  Serial.print(frequency / 1000000);
  Serial.println(" MHz");

  Serial.print("Spreading Factor: SF");
  Serial.println(spreadingFactor);

  Serial.print("Bandwidth: ");
  Serial.print(bandwidth / 1000);
  Serial.println(" kHz");

  Serial.print("Packets sent: ");
  Serial.println(sent);

  Serial.print("Packets received: ");
  Serial.println(received);

  Serial.print("Auto-send: ");
  Serial.println(autoSend ? "ON" : "OFF");

  Serial.println("CRC16: ENABLED");

  Serial.println("==================================");
}


// ======================================
// HELP COMMANDS
// ======================================

void help() {

  Serial.println();
  Serial.println("Commands:");
  Serial.println("HELP       - Show commands");
  Serial.println("STATUS     - Show node status");
  Serial.println("SEND       - Send sensor data now");
  Serial.println("AUTO ON    - Enable automatic sending");
  Serial.println("AUTO OFF   - Disable automatic sending");
}


// ======================================
// PROCESS SERIAL COMMANDS
// ======================================

void command(char* text) {

  while (*text == ' ') ++text;

  char* arg = strchr(text, ' ');

  if (arg) {
    *arg++ = 0;
    while (*arg == ' ') ++arg;
  } else {
    arg = text + strlen(text);
  }

  for (char* p = text; *p; ++p) {
    *p = toupper(*p);
  }

  if (!*text) return;

  if (strcmp(text, "HELP") == 0) {
    help();
  }

  else if (strcmp(text, "STATUS") == 0) {
    status();
  }

  else if (strcmp(text, "SEND") == 0) {
    sendSensorData();
  }

  else if (strcmp(text, "AUTO") == 0) {

    for (char* p = arg; *p; ++p) {
      *p = toupper(*p);
    }

    if (strcmp(arg, "ON") == 0) {
      autoSend = true;
      Serial.println("Auto-send ENABLED");
    }

    else if (strcmp(arg, "OFF") == 0) {
      autoSend = false;
      Serial.println("Auto-send DISABLED");
    }

    else {
      Serial.println("Use AUTO ON or AUTO OFF");
    }
  }

  else {
    Serial.println("Unknown command - type HELP");
  }
}


// ======================================
// READ SERIAL MONITOR INPUT
// ======================================

void readSerial() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\r' || c == '\n') {

      if (lineLength) {
        line[lineLength] = '\0';
        lineLength = 0;
        command(line);
      }
    }

    else if (lineLength < sizeof(line) - 1) {
      line[lineLength++] = c;
    }
  }
}


// ======================================
// MAIN LOOP
// ======================================

void loop() {

  readSerial();

  if (autoSend && millis() - lastSend >= SEND_INTERVAL) {

    lastSend = millis();

    sendSensorData();
  }
}
