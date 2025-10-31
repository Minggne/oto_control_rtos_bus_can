#include "DashboardScreen.h"

void DashboardScreen::update(const struct can_frame& frame) {
  if (frame.can_id == 0x100 && frame.can_dlc >= 3) {
    weight = frame.data[0];
    potVal = map(frame.data[1], 0, 255, 0, 180);
    switchState = frame.data[2];
    speed = map(weight, 0, 255, 0, 200);
    hasData = true;
  }
}

void DashboardScreen::draw(U8G2& u8g2) {
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso32_tf);
  u8g2.setCursor(10, 40);
  u8g2.print(hasData ? speed : -1);
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(75, 40);
  u8g2.print("km/h");

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 55);  u8g2.print("W:");
  u8g2.setCursor(37, 55); u8g2.print("| A:");
  u8g2.setCursor(87, 55); u8g2.print("| D:");

  if (hasData) {
    u8g2.setCursor(15, 55);  u8g2.print(weight);
    u8g2.setCursor(65, 55);  u8g2.print(potVal);
    u8g2.setCursor(115, 55); u8g2.print(switchState ? "F" : "B");
  } else {
    u8g2.drawStr(40, 62, "No data");
  }
  u8g2.sendBuffer();
}
