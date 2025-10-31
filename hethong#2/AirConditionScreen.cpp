#include "AirConditionScreen.h"
#include <math.h>
#include <Arduino.h>

float smoothstep(float x, float edge0 = 0.0f, float edge1 = 1.0f) {
  x = constrain((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return x * x * (3 - 2 * x);
}

void AirConditionScreen::draw(U8G2& u8g2) {
  int potValue = analogRead(potPin);
  float percent = constrain(map(potValue, 0, 4095, 0, 100), 0, 100);
  float angle = smoothstep(percent / 100.0) * 180.0;
  
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont22_tn);
  char buf[5];
  sprintf(buf, "%d", (int)percent);
  u8g2.drawStr(64 - u8g2.getStrWidth(buf) / 2, 40, buf);
  u8g2.setFont(u8g2_font_profont10_tr);
  u8g2.drawStr(54, 48, "POWER");

  u8g2.drawCircle(64, 60, 59, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(64, 60, 57, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);

  int endx = round(-cos(radians(angle)) * 54);
  int endy = round(-sin(radians(angle)) * 54);
  int startx_off = round(-cos(radians(angle + 90)) * 3);
  int starty_off = round(-sin(radians(angle + 90)) * 3);

  for (int i = 0; i <= 180; i += 45) {
    int tick_x1 = 64 + round(-cos(radians(i)) * 50);
    int tick_y1 = 60 + round(-sin(radians(i)) * 50);
    int tick_x2 = 64 + round(-cos(radians(i)) * 54);
    int tick_y2 = 60 + round(-sin(radians(i)) * 54);
    u8g2.drawLine(tick_x1, tick_y1, tick_x2, tick_y2);

    int label_x = 64 + round(-cos(radians(i)) * 41);
    int label_y = 60 + round(-sin(radians(i)) * 41) + 4;
    char label_buf[4];
    sprintf(label_buf, "%d", (i * 100) / 180);
    u8g2.drawStr(label_x - u8g2.getStrWidth(label_buf) / 2, label_y, label_buf);
  }

  u8g2.setDrawColor(0);
  u8g2.drawTriangle(64 + startx_off * 1.8, 60 + starty_off * 1.8,
                    64 - startx_off * 1.8, 60 - starty_off * 1.8,
                    64 + endx * 1.05, 60 + endy * 1.05);
  u8g2.setDrawColor(1);
  u8g2.drawLine(64 + startx_off, 60 + starty_off, 64 + endx, 60 + endy);
  u8g2.drawLine(64 - startx_off, 60 - starty_off, 64 + endx, 60 + endy);

  u8g2.drawDisc(64, 60, 8);
  u8g2.setDrawColor(0);
  u8g2.drawDisc(64, 60, 7);

  u8g2.sendBuffer();
}
