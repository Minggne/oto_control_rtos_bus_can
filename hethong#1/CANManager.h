#pragma once
#include <mcp2515.h>

class CANManager {
public:
  CANManager(uint8_t csPin);
  void begin();
  void send(struct can_frame& frame);

private:
  MCP2515 mcp;
};
