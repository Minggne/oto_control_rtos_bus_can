#include "EngineInfoScreen.h"

void EngineInfoScreen::update(const struct can_frame& frame) {
  if (frame.can_id == 0x300 && frame.can_dlc >= 1) {
    temperature = frame.data[0];
    hasData = true;
  }
}

void EngineInfoScreen::draw(U8G2& u8g2) {
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.setCursor(35, 12);
  u8g2.print("TEMP ENGINE");

  if (hasData) {
    u8g2.setFont(u8g2_font_logisoso32_tf);
    u8g2.setCursor(40, 55);
    u8g2.print(temperature);

    u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2.setCursor(85, 55);
    u8g2.print("\xb0""C");

    u8g2.drawCircle(18, 55, 7, U8G2_DRAW_ALL);
    u8g2.drawRFrame(16, 3, 5, 49, 2);
    for (int i = 6; i <= 45; i += 3) {
      u8g2.drawPixel(21, i);
    }
    int barHeight = map(temperature, 0, 100, 0, 43);
    u8g2.drawLine(18, 46, 18, 46 - barHeight);
  } else {
    u8g2.setFont(u8g2_font_logisoso32_tf);
    u8g2.setCursor(40, 55);
    u8g2.print("E");
  }

  u8g2.sendBuffer();
}
