#pragma once

#include "consts.h"
#include "config.h"
#include <chrono>

namespace dynamic_programming
{
  class DisturbanceController
  {
  public:
    DisturbanceController();
    ~DisturbanceController();

    const unit3& get_next_disturbance();

  private:
    int m_last_index = 0;
    int m_turns_since_last_change = 0;
    bool m_applyDisturbance = false;
  };
}