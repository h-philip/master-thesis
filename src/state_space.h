#pragma once

#include "consts.h"
#include "range.h"

namespace dynamic_programming
{
  struct StateSpace
  {
    unit begin[6];
    unit step[6];
    unit end[6];

    Range get_range(int index) const
    {
      return Range(begin[index], step[index], end[index]);
    }

    void offset(const unit3& point)
    {
      for (int i = 0; i < 3; i++)
      {
        begin[i] -= point[i];
        end[i] -= point[i];
      }
    }

    bool contains(const int* x) const
    {
      bool contains = true;
      for (int i = 0; i < 6 && contains; i++)
        contains &= begin[i] <= x[i] && end[i] >= x[i];
      return contains;
    }

    bool contains(const float* x) const
    {
      bool contains = true;
      for (int i = 0; i < 6 && contains; i++)
        contains &= begin[i] <= x[i] && end[i] >= x[i];
      return contains;
    }

    void extend_relative(const float &value)
    {
      unit length = 0;

      for (size_t i = 0; i < 6; i++)
      {
        length = end[i] - begin[i];
        begin[i] = begin[i] - length / (2 * value);
        end[i] = end[i] + length / (2 * value);
      }
    }
    
    void extend_absolute(const unit values[6])
    {
      for (size_t i = 0; i < 6; i++)
      {
        begin[i] -= values[i];
        end[i] += values[i];
      }
    }

    void extend_for_stretching(const unit3& stretch_factor)
    {
      for (int i = 0; i < 6; i++)
      {
        int factor = (int)(begin[i] / stretch_factor[i % 3]);
        if (factor * stretch_factor[i % 3] != begin[i])
          begin[i] = stretch_factor[i % 3] * (factor + (begin[i] < 0 ? -1 : 1));
        factor = (int)(end[i] / stretch_factor[i % 3]);
        if (factor * stretch_factor[i % 3] != end[i])
          end[i] = stretch_factor[i % 3] * (factor + (end[i] < 0 ? -1 : 1));
      }
    }

    /**
    *                   p4 _________ p3
    *                  / |          / |
    *                 /  |         /  |
    *                /   |        /   |
    *              p1 ___|_____ p2    |
    *              |     |       |    |
    *              |    p8 ______|__ p7
    *              |   /         |   /
    *              |  /          |  /
    *              | /           | /
    *              |/            |/
    *              p5 _________ p6
    * 
    * p1
    * p2
    * p3
    * p4
    * p1
    * 
    * p5
    * p6
    * p7
    * p8
    * p5
    */
    std::string to_gnuplot_cube() const
    {
      // Create 4 variables of 3d points which contain the four corners of the top face of the first three dimensions of the cube
      std::string p1 = std::to_string(begin[0]) + " " + std::to_string(begin[1]) + " " + std::to_string(end[2]);
      std::string p2 = std::to_string(end[0]) + " " + std::to_string(begin[1]) + " " + std::to_string(end[2]);
      std::string p3 = std::to_string(end[0]) + " " + std::to_string(end[1]) + " " + std::to_string(end[2]);
      std::string p4 = std::to_string(begin[0]) + " " + std::to_string(end[1]) + " " + std::to_string(end[2]);
      // Create 4 variables of 3d points which contain the four corners of the bottom face of the first three dimensions of the cube
      std::string p5 = std::to_string(begin[0]) + " " + std::to_string(begin[1]) + " " + std::to_string(begin[2]);
      std::string p6 = std::to_string(end[0]) + " " + std::to_string(begin[1]) + " " + std::to_string(begin[2]);
      std::string p7 = std::to_string(end[0]) + " " + std::to_string(end[1]) + " " + std::to_string(begin[2]);
      std::string p8 = std::to_string(begin[0]) + " " + std::to_string(end[1]) + " " + std::to_string(begin[2]);

      return p1 + "\n" + p2 + "\n" + p3 + "\n" + p4 + "\n" + p1 + "\n\n" + p5 + "\n" + p6 + "\n" + p7 + "\n" + p8 + "\n" + p5;
    }
  };
}