#ifndef AIRCONDITION_SCREEN_H
#define AIRCONDITION_SCREEN_H

#include "ScreenBase.h"

class AirConditionScreen : public ScreenBase {
private:
  int potPin;

public:
  AirConditionScreen(int pin) : potPin(pin) {}
  void update(const struct can_frame&) override {}
  void draw(U8G2& u8g2) override;
  const char* name() override { return "AirCon"; }
};

#endif
