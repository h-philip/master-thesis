#include "range.h"

dynamic_programming::Range::Range(const unit begin, const unit step, const unit end)
  : m_begin(begin), m_step(0), m_end(0)
{
  set_step(step);
  set_end(end);
}

dynamic_programming::Range::Range(const Range& rhs)
{
  m_begin = rhs.m_begin;
  m_step = rhs.m_step;
  m_end = rhs.m_end;
}

void dynamic_programming::Range::set_begin(const unit begin)
{
  if (begin > m_end)
    throw std::invalid_argument("begin must be smaller than end");
  m_begin = begin;
}

void dynamic_programming::Range::set_end(const unit end)
{
  if (m_begin + m_step > end)
    throw std::invalid_argument("end must be at least as great as begin + one step");
  m_end = m_begin + (end - m_begin) / m_step * m_step;
}

void dynamic_programming::Range::set_step(const unit step)
{
  if (step <= 0)
    throw std::invalid_argument("step must be greater than 0");
  m_step = step;
}

const dynamic_programming::unit& dynamic_programming::Range::get_begin() const
{
  return m_begin;
}

const dynamic_programming::unit& dynamic_programming::Range::get_end() const
{
  return m_end;
}

const dynamic_programming::unit& dynamic_programming::Range::get_step() const
{
  return m_step;
}

const dynamic_programming::unit dynamic_programming::Range::operator[](size_t i) const
{
  if (m_step == 0)
    return m_begin;
  float value = m_begin + m_step * (int)i;
  if (value > m_end)
    throw std::out_of_range("");
  else
    return value;
}

int dynamic_programming::Range::search(const float value) const
{
  return search_away_from_zero(value);
}

int dynamic_programming::Range::search(const unit begin, const unit step, const unit end, const float value)
{
  return search_away_from_zero(begin, step, end, value);
}

int dynamic_programming::Range::search_closest(const float value) const
{
  return search_closest(m_begin, m_step, m_end, value);
}

int dynamic_programming::Range::search_closest(const unit begin, const unit step, const unit end, const float value)
{
  if (value < begin || value > end)
    return -1;
  return unit(round((value - (float)begin) / (float)step));
}

int dynamic_programming::Range::search_away_from_zero(const float value) const
{
  return search_away_from_zero(m_begin, m_step, m_end, value);
}

int dynamic_programming::Range::search_away_from_zero(const unit begin, const unit step, const unit end, const float value)
{
  if (value < begin || value > end)
    return -1;
  if (value > 0)
    return unit(ceilf((value - (float)begin) / (float)step));
  else
    return unit(floorf((value - (float)begin) / (float)step));
}

size_t dynamic_programming::Range::length() const
{
  size_t v = size_t((m_end - m_begin) / m_step);
  return v > 0 ? v + 1 : 0;
}

std::string dynamic_programming::Range::to_string() const
{
  return "Range(" + std::to_string(m_begin) + ":" + std::to_string(m_step) + ":" + std::to_string(m_end) + ")";
}
