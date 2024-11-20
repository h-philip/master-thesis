#include "collision_cloud.h"

const double dynamic_programming::CollisionCloud::MIN_DISTANCE_TO_COLLISION = 1.5;

dynamic_programming::CollisionCloud::CollisionCloud(const size_t lengths[3], unit step_size) : CollisionCloud(lengths[0], lengths[1], lengths[2], step_size)
{}

dynamic_programming::CollisionCloud::CollisionCloud(const size_t& lx, const size_t& ly, const size_t& lz, unit step_size)
  : m_will_collide(boost::extents[lx][ly][lz][lx][ly][lz]),
  m_min_dist(MIN_DISTANCE_TO_COLLISION / step_size),
  m_min_dist_2(pow(MIN_DISTANCE_TO_COLLISION / step_size, 2))
{
  reset_will_collide_array();
}

dynamic_programming::CollisionCloud::CollisionCloud(const CollisionCloud& rhs)
  : m_min_dist_2(rhs.m_min_dist_2), m_min_dist(rhs.m_min_dist)
{
  m_will_collide = rhs.m_will_collide;
}

void dynamic_programming::CollisionCloud::add_collision(point3 point)
{
  m_collisions.push_back(point);
}

void dynamic_programming::CollisionCloud::reset_will_collide_array()
{
  for (int8_t* it = m_will_collide.origin(); it < (m_will_collide.origin() + m_will_collide.num_elements()); it++)
  {
    *it = -1;
  }
}

bool dynamic_programming::CollisionCloud::will_collide(const point3& i_old_c, const point3& i_new_c)
{
  int8_t& will_collide = m_will_collide[i_old_c.x()][i_old_c.y()][i_old_c.z()][i_new_c.x()][i_new_c.y()][i_new_c.z()];
  if (will_collide == 0)
    return false;
  else if (will_collide == 1)
    return true;

  auto x = boost::minmax(i_old_c.x(), i_new_c.x());
  auto y = boost::minmax(i_old_c.y(), i_new_c.y());
  auto z = boost::minmax(i_old_c.z(), i_new_c.z());

  point3 temp1, temp2;

  for (const point3& collision : m_collisions)
  {
    if (collision.get<0>() < x.get<0>() - m_min_dist * 2 || collision.get<0>() > x.get<1>() + m_min_dist * 2
      || collision.get<1>() < y.get<0>() - m_min_dist * 2 || collision.get<1>() > y.get<1>() + m_min_dist * 2
      || collision.get<2>() < z.get<0>() - m_min_dist * 2 || collision.get<2>() > z.get<1>() + m_min_dist * 2)
      continue;
    // https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    double distance_2 = 0.;
    if (i_old_c.x() == i_new_c.x() && i_old_c.y() == i_new_c.y() && i_old_c.z() == i_new_c.z())
    {
      // old and new x are the same -> distance between to points is calculated
      temp1 = collision;
      bg::subtract_point(temp1, i_old_c);
      distance_2 = pow(temp1.x(), 2) + pow(temp1.y(), 2) + pow(temp1.z(), 2);
    }
    else
    {
      temp1 = i_old_c;
      temp2 = i_new_c;
      bg::subtract_point(temp1, collision);
      bg::subtract_point(temp2, i_old_c);
      double t = -bg::dot_product(temp1, temp2);
      temp1 = i_new_c;
      bg::subtract_point(temp1, i_old_c);
      t /= pow(temp1.x(), 2) + pow(temp1.y(), 2) + pow(temp1.z(), 2);
      if (t <= 0 || t >= 1)
      {
        // get distance to i_old_c or i_new_c but not the line
        temp1 = t <= 0 ? i_old_c : i_new_c;
        bg::subtract_point(temp1, collision);
        distance_2 = pow(temp1.x(), 2) + pow(temp1.y(), 2) + pow(temp1.z(), 2);
      }
      else
      {
        // get distance to line
        distance_2 = pow(i_old_c.x() - collision.x(), 2) + pow(i_old_c.y() - collision.y(), 2) + pow(i_old_c.z() - collision.z(), 2)
          + 2.0 * t * ((i_new_c.x() - i_old_c.x()) * (i_old_c.x() - collision.x()) + (i_new_c.y() - i_old_c.y()) * (i_old_c.y() - collision.y()) + (i_new_c.z() - i_old_c.z()) * (i_old_c.z() - collision.z()))
          + t * t * (pow(i_new_c.x() - i_old_c.x(), 2) + pow(i_new_c.y() - i_old_c.y(), 2) + pow(i_new_c.z() - i_old_c.z(), 2));
      }
    }
    if (distance_2 < m_min_dist_2)
    {
      will_collide = 1;
      return true;
    }
  }
  will_collide = 0;
  return false;
}

void dynamic_programming::CollisionCloud::add_collisions_from_file(const std::string path, std::function<point3(const unit3&)> converter)
{
  std::ifstream file(path);
  if (file.is_open())
  {
    std::string line;
    unit3 collision{};
    while (std::getline(file, line))
    {
      if (line.empty() || line.at(0) == ' ' || line.at(0) == '#')
        continue;
      if (line.compare("end") == 0)
        break;
      std::string::size_type index = line.find(" ");
      collision.x = (unit)std::stof(line.substr(0, index));
      line = line.substr(index + 1);
      index = line.find(" ");
      collision.y = (unit)std::stof(line.substr(0, index));
      collision.z = (unit)std::stof(line.substr(index + 1));
      add_collision(converter(collision));
    }
  }
}
