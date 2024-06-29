// SURFACE TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics

#include <RH_RF95.h>
#include <SPI.h>

#include "rov_common.hpp"

const uint8_t COMMAND_SPAM_TIMES = 5;
const uint32_t COMMAND_SPAM_DELAY = 500;
const uint32_t SURFACE_PKT_RX_TIMEOUT = 1000;
const uint8_t MAX_RESPONSE_LEN = RH_RF95_MAX_MESSAGE_LEN >> 1;  // Max length for ACKs/NACKs

bool printRFStatus = true;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

const uint8_t LEGAL_COMMANDS_LEN = 5;
const String LEGAL_COMMANDS[LEGAL_COMMANDS_LEN] = {"submerge", "suck", "pump", "stop", "return"};

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) {}

  Serial.println("Surface Transceiver");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather RFM95 RX Test!\n");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1) {}
  }
  Serial.println("RFM95 radio init OK!");

  // Defaults after init are: 434.0MHz, modulation GFSK_Rb250Fd250
  // +13dbM (for low power module), no encryption
  // But we override frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1) {}
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  serialPrintf("RFM95 radio @ %d MHz\n", static_cast<int>(RF95_FREQ));
}

void loop() {
  receivePacket();

  if (Serial.available() >= 1) {
    Serial.println("Getting serial command...");
    String command = Serial.readString();

    if (command.startsWith("submerge")) {
      printRFStatus = false;
    }

    for (int i = 0; i < LEGAL_COMMANDS_LEN; i++) {
      if (command.startsWith(LEGAL_COMMANDS[i])) {
        sendCommand(LEGAL_COMMANDS[i]);
        return;
      }
    }

    Serial.println("Invalid command; not sending");
  }
}

/**
 * Try to receive a packet for SURFACE_PKT_RX_TIMEOUT ms.
 * Return true if a packet was received and it was an ACK/NACK packet; else return false.
 * Print data from data packets (note we return false if data packets are successfully received).
 */
bool receivePacket() {
#ifdef DO_DEBUGGING
  Serial.println("Attempting to receive");
#endif  // DO_DEBUGGING

  if (!rf95.waitAvailableTimeout(SURFACE_PKT_RX_TIMEOUT)) {
    if (printRFStatus) {
      Serial.println("RF95 not available");
    }
    return false;
  }

  if (printRFStatus) {
    Serial.println("RF95 available");
  }
  byte byteBuffer[RH_RF95_MAX_MESSAGE_LEN];
  byte len = sizeof(byteBuffer);

  if (!rf95.recv(byteBuffer, &len)) {
    Serial.println("Receive failed");
    return false;
  }

  if (!len) {
    Serial.println("Message length 0, dropping");
    return false;
  }

  if (len < MAX_RESPONSE_LEN) {
    // This packet is probably an ACK/NACK, but could be a simple packet for judges
    byteBuffer[len] = '\0';
    char* charBuffer = reinterpret_cast<char*>(byteBuffer);
    // If this is a "single" judge/calibration packet
    if (strncmp(byteBuffer, "ROS:SINGLE:", 11) == 0) {
      Serial.println(charBuffer);
    }
    else {
      serialPrintf("Got pkt w/len %d, str '%s' & vals: ", len, charBuffer);
      for (int i = 0; i < len; i++) {
        serialPrintf("%d, ", byteBuffer[i]);
      }
      Serial.println();
    }

    bool isACK = (strncmp(byteBuffer, "ACK", 3) == 0);
    bool isNACK = (strncmp(byteBuffer, "NACK", 4) == 0);

    return isACK || isNACK;
  }
  else {
    // This packet is probably a data packet
    uint8_t numDatapoints = static_cast<uint8_t>(len - (PKT_HEADER_LEN >> 2));

#ifdef DO_DEBUGGING
    serialPrintf(
      "Received packet for team %d on profile %d half %d with length %d (%d datapoints): ",
      byteBuffer[PKT_IDX_TEAM_NUM], byteBuffer[PKT_IDX_PROFILE_NUM],
      byteBuffer[PKT_IDX_PROFILE_HALF], len, numDatapoints);

    for (int i = 0; i < len; i++) {
      serialPrintf("%d, ", byteBuffer[i]);
    }
    Serial.println();
#endif  // DO_DEBUGGING

    serialPrintf(
      "ROS:%d,%d,%d:", byteBuffer[PKT_IDX_TEAM_NUM], byteBuffer[PKT_IDX_PROFILE_NUM],
      byteBuffer[PKT_IDX_PROFILE_HALF]);
    for (int i = PKT_HEADER_LEN; i < len;) {
      // Wonky pointer casting to convert four bytes into a long (time)
      serialPrintf("%l,", *reinterpret_cast<uint32_t*>(byteBuffer + i));
      i += sizeof(uint32_t);

      // Same thing for a float (pressure)
      Serial.print(*reinterpret_cast<float*>(byteBuffer + i));
      i += sizeof(float);

      if (i < len - 1) {
        Serial.print(";");
      }
    }
    Serial.println();
  }

  return false;
}

void sendCommand(String command) {
  byte commandBytes[command.length() + 1];
  command.getBytes(commandBytes, command.length() + 1);

  for (int i = 0; i < COMMAND_SPAM_TIMES; i++) {
    rf95.send(commandBytes, command.length() + 1);
    rf95.waitPacketSent();

    serialPrintf(
      "'%s' (len=%d) command sent! Iteration: %d\n", commandBytes, command.length() + 1, i);

    bool receivedACK = receivePacket();
    if (receivedACK) {
      break;
    }

    delay(COMMAND_SPAM_DELAY);
  }
}
