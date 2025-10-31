#include <Arduino.h>
#include "CANManager.h"

CANManager::CANManager(uint8_t csPin) : mcp(csPin) {}

void CANManager::begin() {
  mcp.reset();
  mcp.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp.setNormalMode();

  pinMode(PC13, OUTPUT);  // LED debug
  digitalWrite(PC13, HIGH);
}

void CANManager::send(struct can_frame& frame) {
  mcp.sendMessage(&frame);
  digitalWrite(PC13, LOW);
  delay(100);
  digitalWrite(PC13, HIGH);
}
