#pragma once

#include <limits>
#include <string>
#include <stdexcept>
#include <boost/log/trivial.hpp>

#define _DISTURBANCES 1

namespace dynamic_programming {

  typedef int unit;

  const unit DELTA_TIME = 1;
  const unit STEP_SIZE = 1;
  const unit DRAG_FORCE_COEFFICIENT = 0;// 0.25;

  /*
   * DELTA_TIME smaller -> STEP_SIZE must be smaller or MAX_ACCELERATION must be bigger
   * DELTA_TIME bigger -> MAX_ACCELERATION must be smaller
   * STEP_SIZE bigger -> DELTA_TIME must be bigger or MAX_ACCELERATION must be bigger
   * MAX_ACCELERATION smaller -> DELTA_TIME must be bigger or STEP_SIZE must be smaller
   * MAX_ACCELERATION bigger -> DELTA_TIME must be smaller
   */

  void debug_parameters();

  typedef int (*ctrl_ptr)(const unit* x, const int time);

  struct unit3
  {
    unit x;
    unit y;
    unit z;

    unit3() : unit3(0, 0, 0) {}
    unit3(unit _x, unit _y, unit _z) : x(_x), y(_y), z(_z) {}

    unit& operator[](const size_t index)
    {
      if (index == 0)
        return x;
      else if (index == 1)
        return y;
      else if (index == 2)
        return z;
      else
        throw std::range_error("Index must be 0, 1, or 2");
    }

    const unit& operator[](const size_t index) const
    {
      if (index == 0)
        return x;
      else if (index == 1)
        return y;
      else if (index == 2)
        return z;
      else
        throw std::range_error("Index must be 0, 1, or 2");
    }

    friend unit3 operator+(const unit3& lhs, const unit3& rhs)
    {
      return unit3
      {
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z
      };
    }

    friend unit3 operator-(const unit3& lhs, const unit3& rhs)
    {
      return unit3
      {
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z
      };
    }

    friend unit3 operator*(const unit3& lhs, const unit3& rhs)
    {
      return unit3
      {
        lhs.x * rhs.x,
        lhs.y * rhs.y,
        lhs.z * rhs.z
      };
    }

    friend unit3 operator/(const unit3& lhs, const unit3& rhs)
    {
      return unit3
      {
        lhs.x / rhs.x,
        lhs.y / rhs.y,
        lhs.z / rhs.z
      };
    }

    void operator/=(const unit3& rhs)
    {
      x /= rhs.x;
      y /= rhs.y;
      z /= rhs.z;
    }

    friend bool operator==(const unit3& lhs, const unit3& rhs)
    {
      return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    std::string to_string() const
    {
      return std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z);
    }

    static const unit3 ZERO() { return unit3(0, 0, 0); }

    static const unit3 ONE() { return unit3(1, 1, 1); }
  };

  const float R = 1;

  const size_t NUM_INPUTS = 27;

  extern unit3 INPUTS_FINE[NUM_INPUTS];
  extern unit3 INPUTS_COARSE[NUM_INPUTS];

  const unit MAX_ACCELERATION = 10;

  void create_inputs(unit3* dest, unit3 max);

  void fill_input_arrays();

#if _DISTURBANCES == 1
  const size_t NUM_DISTURBANCES = 5;
#else
  const size_t NUM_DISTURBANCES = 1;
#endif

  const unit3 DISTURBANCES[] = {
    {0, 0, 0},
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0}
  };

  int search_disturbances(const unit3& value);
}