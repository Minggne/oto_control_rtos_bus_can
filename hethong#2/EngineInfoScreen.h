#ifndef ENGINE_INFO_SCREEN_H
#define ENGINE_INFO_SCREEN_H

#include "ScreenBase.h"

class EngineInfoScreen : public ScreenBase {
private:
  int8_t temperature = -1;
  bool hasData = false;

public:
  void update(const struct can_frame& frame) override;
  void draw(U8G2& u8g2) override;
  const char* name() override { return "Engine Info"; }
};

#endif
