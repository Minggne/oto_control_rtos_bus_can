#ifndef SCREENBASE_H
#define SCREENBASE_H

#include <U8g2lib.h>
#include <mcp2515.h>

class ScreenBase {
public:
  virtual void update(const struct can_frame& frame) = 0;
  virtual void draw(U8G2& u8g2) = 0;
  virtual const char* name() = 0;
  virtual ~ScreenBase() {}
};

#endif
