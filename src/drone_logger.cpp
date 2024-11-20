#include "drone_logger.h"

void dynamic_programming::DroneLogger::flush()
{
  if (m_file)
    m_file_stream.flush();
}

void dynamic_programming::DroneLogger::log_to_file(const std::string file)
{
  
  m_file_stream.open(file, std::ios::out | std::ios::app);
  m_file = true;
}

void dynamic_programming::DroneLogger::on_state_changed(const HybridAutomaton::StateChangedEvent& event)
{
  std::stringstream s;
  s << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(7) << event.new_time << " : State changed from { ";
  s << event.old_state->name() << " } to { " << event.new_state->name() << " }. Next waypoint: [ ";
  s << event.new_point.x << ", " << event.new_point.y << ", " << event.new_point.z << " ]";;
  std::string str = s.str();
  BOOST_LOG_TRIVIAL(info) << str;
  if (m_file)
    m_file_stream << str << std::endl;
}

void dynamic_programming::DroneLogger::on_x_changed(const HybridAutomaton::XChangedEvent& event)
{
  std::stringstream s;
  s << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(7) << event.new_time << " : Drone moved with input [ ";
  s << event.u.x << ", " << event.u.y << ", " << event.u.z << " ] to coordinates [ ";
  s << event.new_x[0] << ", " << event.new_x[1] << ", " << event.new_x[2] << " ] and velocity [ ";
  s << event.new_x[3] << ", " << event.new_x[4] << ", " << event.new_x[5] << " ]. Disturbance is: [ ";
  s << event.d.x << ", " << event.d.y << ", " << event.d.z << " ]";
  std::string str = s.str();
  BOOST_LOG_TRIVIAL(info) << str;
  if (m_file)
    m_file_stream << str << std::endl;
}

std::string dynamic_programming::DroneLogger::units_to_string(const unit* f, const size_t length)
{
  std::string s = std::to_string(f[0]);
  for (size_t i = 1; i < length; i++)
  {
    s += ", " + std::to_string(f[i]);
  }
  return s;
}
