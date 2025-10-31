#ifndef PLACEHOLDER_SCREEN_H
#define PLACEHOLDER_SCREEN_H

#include "ScreenBase.h"
#include <cstring>

class PlaceholderScreen : public ScreenBase {
private:
  const char* title;

public:
  PlaceholderScreen(const char* name) : title(name) {}

  void update(const struct can_frame&) override {}

  void draw(U8G2& u8g2) override {
    u8g2.setDrawColor(1);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(15, 25, "Coming soon...");
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(0, 45, "Screen: ");
    u8g2.drawStr(60, 45, title);
    u8g2.sendBuffer();
  }

  const char* name() override { return title; }
};

#endif
