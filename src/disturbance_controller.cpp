#include "disturbance_controller.h"

dynamic_programming::DisturbanceController::DisturbanceController()
{
  std::srand((unsigned)std::time(NULL));
}

dynamic_programming::DisturbanceController::~DisturbanceController()
{
}

const dynamic_programming::unit3& dynamic_programming::DisturbanceController::get_next_disturbance()
{
  Config& config = Config::get_instance();
  if (!config.get<bool>(Config::Key::DISTURBANCE_ON) || !config.get<bool>(Config::Key::APPLY_DISTURBANCE))
  {
    return DISTURBANCES[0];
  }

  int factor = config.get<int>(Config::Key::DISTURBANCE_CHANGE_FACTOR);
  int will_change = std::rand() % factor;
  int index = m_last_index;
  if (will_change < m_turns_since_last_change)
  {
    index += std::rand() % 2 == 0 ? -1 : 1;
    index = index % NUM_DISTURBANCES;
    m_turns_since_last_change = 0;
  }
  m_last_index = index;
  m_turns_since_last_change++;

  return DISTURBANCES[m_last_index];
}
