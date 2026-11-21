
#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define LORA_FREQUENCY 868E6

// Signal information
int lastRssi = 0;
float lastSnr = 0;


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
// CHECK NUMERIC VALUE
// ======================================

bool isValidNumber(const char *text) {

  if (text == NULL || *text == '\0') {
    return false;
  }

  char *endPointer;

  // atof-compatible conversion with validation
  (void)strtod(text, &endPointer);

  return endPointer != text && *endPointer == '\0';
}


// ======================================
// SETUP
// ======================================

void setup() {

  Serial.begin(9600);

  Serial.println();
  Serial.println("================================");
  Serial.println("       NODE 2 - RECEIVER");
  Serial.println("================================");

  // LoRa pins: NSS, RESET, DIO0
  LoRa.setPins(10, 9, 2);

  // Initialize LoRa
  if (!LoRa.begin(LORA_FREQUENCY)) {

    Serial.println("ERROR: LoRa module not responding!");

    while (true) {
      delay(1000);
    }
  }

  // LoRa configuration
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(17);

  Serial.println("LoRa initialized successfully!");
  Serial.println("Frequency: 868 MHz");
  Serial.println("Spreading Factor: SF7");
  Serial.println("Bandwidth: 125 kHz");
  Serial.println("Coding Rate: 4/5");
  Serial.println("CRC16: ENABLED");
  Serial.println("--------------------------------");
  Serial.println("Waiting for messages...");
}


// ======================================
// LOOP
// ======================================

void loop() {

  int packetSize = LoRa.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  char message[64];
  byte index = 0;
  bool overflow = false;

  // Read received packet
  while (LoRa.available()) {

    char c = (char)LoRa.read();

    if (index < sizeof(message) - 1) {
      message[index++] = c;
    } else {
      overflow = true;
    }
  }

  message[index] = '\0';

  // Signal information
  lastRssi = LoRa.packetRssi();
  lastSnr = LoRa.packetSnr();

  Serial.println();
  Serial.println("========== MESSAGE RECEIVED ==========");

  Serial.print("Raw message: ");
  Serial.println(message);

  // Reject oversized packets
  if (overflow) {

    Serial.println("ERROR: Message too long!");
    Serial.println("Packet rejected. No ACK sent.");
    Serial.println("======================================");
    return;
  }


  // ======================================
  // EXTRACT CRC
  // ======================================

  char *crcSeparator = strrchr(message, ',');

  if (crcSeparator == NULL) {

    Serial.println("ERROR: CRC field missing!");
    Serial.println("Packet rejected. No ACK sent.");
    Serial.println("======================================");
    return;
  }

  // Separate payload and CRC
  *crcSeparator = '\0';

  char *receivedCRCText = crcSeparator + 1;

  // CRC must have exactly 4 hexadecimal characters
  if (strlen(receivedCRCText) != 4) {

    Serial.println("ERROR: Invalid CRC format!");
    Serial.println("Expected 4 hexadecimal characters.");
    Serial.println("Packet rejected. No ACK sent.");
    Serial.println("======================================");
    return;
  }

  // Validate hexadecimal characters
  for (byte i = 0; i < 4; i++) {

    if (!isxdigit((unsigned char)receivedCRCText[i])) {

      Serial.println("ERROR: Invalid CRC characters!");
      Serial.println("Packet rejected. No ACK sent.");
      Serial.println("======================================");
      return;
    }
  }

  // Convert received CRC
  uint16_t receivedCRC =
      (uint16_t)strtoul(receivedCRCText, NULL, 16);


  // ======================================
  // CALCULATE CRC AGAIN
  // ======================================

  uint16_t calculatedCRC = calculateCRC16(message);

  Serial.println();
  Serial.println("========== CRC16 CHECK ==========");

  Serial.print("Received CRC: ");
  Serial.println(receivedCRCText);

  Serial.print("Calculated CRC: ");
  Serial.println(calculatedCRC, HEX);

  // Compare CRC
  if (receivedCRC != calculatedCRC) {

    Serial.println();
    Serial.println("CRC ERROR!");
    Serial.println("DATA INTEGRITY CHECK FAILED!");
    Serial.println("Packet rejected. No ACK sent.");
    Serial.println("======================================");
    return;
  }

  Serial.println("CRC16 VALID!");
  Serial.println("DATA INTEGRITY CHECK PASSED!");


  // ======================================
  // VERIFY DATA FORMAT
  // ======================================

  if (strncmp(message, "DATA,", 5) != 0) {

    Serial.println("ERROR: Invalid message format!");
    Serial.println("Packet rejected. No ACK sent.");
    Serial.println("======================================");
    return;
  }

  char *firstComma = strchr(message, ',');
  char *secondComma = NULL;

  if (firstComma != NULL) {
    secondComma = strchr(firstComma + 1, ',');
  }

  if (firstComma == NULL || secondComma == NULL) {

    Serial.println("ERROR: Invalid DATA format!");
    Serial.println("Expected: DATA,Temperature,Humidity");
    Serial.println("======================================");
    return;
  }

  // Separate temperature and humidity
  *secondComma = '\0';

  char *temperatureText = firstComma + 1;
  char *humidityText = secondComma + 1;

  // Validate numeric values
  if (!isValidNumber(temperatureText) ||
      !isValidNumber(humidityText)) {

    Serial.println("ERROR: Invalid sensor values!");
    Serial.println("Packet rejected. No ACK sent.");
    Serial.println("======================================");
    return;
  }

  // Convert values - compatible with Arduino UNO
  float temperature = (float)atof(temperatureText);
  float humidity = (float)atof(humidityText);


  // ======================================
  // DISPLAY SENSOR DATA
  // ======================================

  Serial.println();
  Serial.println("========== SENSOR DATA ==========");

  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");


  // ======================================
  // DISPLAY SIGNAL INFORMATION
  // ======================================

  Serial.println();
  Serial.println("======= SIGNAL INFORMATION =======");

  Serial.print("RSSI: ");
  Serial.print(lastRssi);
  Serial.println(" dBm");

  Serial.print("SNR: ");
  Serial.print(lastSnr, 1);
  Serial.println(" dB");


  // ======================================
  // SEND ACK ONLY IF CRC IS VALID
  // ======================================

  Serial.println();
  Serial.println("Sending ACK to Node 1...");

  LoRa.beginPacket();
  LoRa.print("ACK");

  int result = LoRa.endPacket();

  if (result == 1) {
    Serial.println("ACK SENT SUCCESSFULLY!");
  } else {
    Serial.println("ERROR: ACK transmission failed!");
  }

  Serial.println("======================================");
}
