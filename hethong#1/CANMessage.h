#pragma once
#include <Arduino.h>

struct CANMessage {
  uint16_t can_id;
  uint8_t  can_dlc;
  uint8_t  data[8];
};
