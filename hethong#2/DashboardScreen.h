#ifndef DASHBOARD_SCREEN_H
#define DASHBOARD_SCREEN_H

#include "ScreenBase.h"

class DashboardScreen : public ScreenBase {
private:
  int speed = 0, weight = 0, potVal = 0, switchState = 0;
  bool hasData = false;

public:
  void update(const struct can_frame& frame) override;
  void draw(U8G2& u8g2) override;
  const char* name() override { return "Dashboard"; }
};

#endif
