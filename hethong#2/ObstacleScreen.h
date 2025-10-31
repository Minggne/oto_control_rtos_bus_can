#ifndef OBSTACLE_SCREEN_H
#define OBSTACLE_SCREEN_H

#include "ScreenBase.h"

class ObstacleScreen : public ScreenBase {
private:
  int distL = -1;
  int distR = -1;
  int ir = -1;

public:
  void update(const struct can_frame& frame) override;
  void draw(U8G2& u8g2) override;
  const char* name() override { return "Obstacle"; }
};

#endif
