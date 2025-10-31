#include "ObstacleScreen.h"

void ObstacleScreen::update(const struct can_frame& frame) {
  if (frame.can_id == 0x200 && frame.can_dlc >= 3) {
    distL = frame.data[0];
    distR = frame.data[1];
    ir = frame.data[2];
  }
}

void ObstacleScreen::draw(U8G2& u8g2) {
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();

  if (ir == 1) {
    u8g2.setFont(u8g2_font_helvB24_tf);
    u8g2.setCursor(20, 30);
    if ((millis() / 500) % 2 == 0)
      u8g2.print("STOP");
  } else if (ir == 0) {
    u8g2.setFont(u8g2_font_helvB24_tf);
    u8g2.setCursor(20, 30);
    u8g2.print("SAFE");
  } else {
    u8g2.setFont(u8g2_font_helvB18_tf);
    u8g2.setCursor(15, 30);
    u8g2.print("ERROR!!");
  }

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(0, 44); u8g2.print("DistL");
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(0, 62); distL >= 0 ? u8g2.print(distL) : u8g2.print("E");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(30, 62); u8g2.print("cm");

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(95, 44); u8g2.print("DistR");
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(85, 62); distR >= 0 ? u8g2.print(distR) : u8g2.print("E");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(115, 62); u8g2.print("cm");

  u8g2.sendBuffer();
}
