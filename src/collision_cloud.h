#pragma once

#include "consts.h"
#include <algorithm>
#include <fstream>
#include <functional>

#pragma warning(push, 0)
#include <boost/multi_array.hpp>
#include <boost/algorithm/minmax.hpp>
#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <boost/math/tools/norms.hpp>
#pragma warning(pop)

namespace bg = boost::geometry;

namespace dynamic_programming
{
  class CollisionCloud
  {
  public:
    /// <summary>
    /// In meters.
    /// </summary>
    static const double MIN_DISTANCE_TO_COLLISION;
    typedef boost::multi_array<int8_t, 6> will_collide_array;
    typedef bg::model::d3::point_xyz<int> point3;

    CollisionCloud(const size_t lengths[3], unit step_size);
    CollisionCloud(const size_t& lx, const size_t& ly, const size_t& lz, unit step_size);
    CollisionCloud(const CollisionCloud& rhs);

    void reset_will_collide_array();

    void add_collision(point3 point);

    void add_collisions_from_file(const std::string path, std::function<point3(const unit3&)> converter);

    bool will_collide(const point3& i_old_c, const point3& i_new_c);

    std::vector<point3>& get_collisions() { return m_collisions; }

  private:
    std::vector<point3> m_collisions;
    will_collide_array m_will_collide;
    double m_min_dist;
    double m_min_dist_2;
  };
}