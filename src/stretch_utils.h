#pragma once

#include "consts.h"
#include "hybrid_automaton.h"
#include "state_space.h"
#include <boost/log/trivial.hpp>

namespace dynamic_programming
{
  class StretchUtils
  {
  public:

  private:
  };

  unit3 choose_stretch_factor(const StateSpace& state_space, const StateSpace& goal_space, const float x0[6], const std::string& state);

  enum validation_result
  {
    valid,
    unequal_remainder,
    small_space,
    too_big,
    undefined
  };

  validation_result validate_stretch_factor(const StateSpace& state_space, const StateSpace& goal_space, const float x0[6], const unit3& stretch_factor, bool log = false);
}