#pragma once

#include <stdexcept>
#include <string>
#include "consts.h"

namespace dynamic_programming {
  class Range {
  private:
    unit m_begin;
    unit m_end;
    unit m_step;
  public:
    Range(const unit begin, const unit step, const unit end);
    Range() : Range(0, 1, 1) {};
    Range(const Range& rhs);

    ~Range() {}

    void set_begin(const unit begin);
    void set_end(const unit end);
    void set_step(const unit step);
    const unit& get_begin() const;
    const unit& get_end() const;
    const unit& get_step() const;

    const unit operator[](const size_t i) const;

    int search(const float value) const;
    static int search(const unit begin, const unit step, const unit end, const float value);

    int search_closest(const float value) const;
    static int search_closest(const unit begin, const unit step, const unit end, const float value);

    int search_away_from_zero(const float value) const;
    static int search_away_from_zero(const unit begin, const unit step, const unit end, const float value);

    size_t length() const;

    std::string to_string() const;
  };
}