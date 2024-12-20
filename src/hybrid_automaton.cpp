#include "hybrid_automaton.h"

void dynamic_programming::HybridAutomaton::State::do_flow(float delta_time_sim)
{
  const unit3 u = this->u();
  const unit3& d = m_ha->m_disturbance_controller.get_next_disturbance();
  float old_x[6]{};
  for (int i = 0; i < 3; i++)
  {
    old_x[i] = m_ha->m_x[i]; // Copy coordinates
    old_x[i + 3] = m_ha->m_x[i + 3]; // Copy velocity
    float drag = -DRAG_FORCE_COEFFICIENT * m_ha->m_x[i + 3];
    m_ha->m_x[i + 3] = m_ha->m_x[i + 3] + (u[i] + d[i] + drag) * delta_time_sim; // Get new velocity from acceleration and disturbance
    m_ha->m_x[i] = m_ha->m_x[i] + m_ha->m_x[i + 3] * delta_time_sim; // Get new coordinates from velocity
  }
  m_ha->m_time += delta_time_sim;

  m_ha->notify_x_changed(old_x, m_ha->m_x, u, d, m_ha->m_time);
}

const dynamic_programming::unit3 dynamic_programming::HybridAutomaton::Starting::u()
{
  const unit3& point = m_ha->current_route_point();
  float x0[6]
  {
    m_ha->m_x[0] - point[0],
    m_ha->m_x[1] - point[1],
    m_ha->m_x[2] - point[2],
    m_ha->m_x[3],
    m_ha->m_x[4],
    m_ha->m_x[5]
  };
  if (m_ha->m_dynamic_programming == nullptr)
  {
    // Calculate controller
    StateSpace state_space = Starting::get_state_space(point);
    float next_x0[6]{ (float)point.x, (float)point.y, (float)point.z, 0.f, 0.f, 0.f };
    StateSpace goal_space = Cruising::get_state_space(next_x0, m_ha->m_route.at(m_ha->m_route_counter + 1));
    unit goal_extension[6]{ -2, -2, -2, -1, -1, -1 };
    goal_space.extend_absolute(goal_extension); // make smaller to allow for some rounding or other errors but still accept the final state in the simulation
    state_space.offset(point);
    goal_space.offset(point);
    m_ha->m_dynamic_programming = new DynamicProgramming(
      state_space,
      goal_space,
      delta_time(),
      unit3::ONE(),
      [point](const unit3& world_point)
      {
        return unit3(world_point.x - point.x, world_point.y - point.y, world_point.z - point.z);
      },
      m_ha->m_dp_logger
    );
    long calculation_stopped_at = -1;
    while (true)
    {
      calculation_stopped_at = m_ha->m_dynamic_programming->calculate_controller(x0);
      BOOST_LOG_TRIVIAL(debug) << "calculation_stopped_at: " << calculation_stopped_at;
      if (calculation_stopped_at >= 0)
        break;
      BOOST_LOG_TRIVIAL(warning) << "Could not find path from x0 to 0. Recalculating controller with extended state space.";
      unit space_extension[6]{ 2, 2, 2, 0, 0, 0 };
      state_space.extend_absolute(space_extension);
      if (state_space.begin[2] < -point.z)
        state_space.begin[2] = -point.z;
      m_ha->m_dynamic_programming->reinitialize();
    }
    m_ha->m_major_time_counter = calculation_stopped_at;
  }

  // Use controller
  try
  {
    return m_ha->m_dynamic_programming->get_control(x0, m_ha->m_major_time_counter);
  }
  catch (const std::logic_error& e)
  {
    BOOST_LOG_TRIVIAL(error) << "logic error while getting control: " << e.what();
    m_ha->m_major_time_counter = 0;
    m_ha->m_minor_time_counter = 0;
    delete m_ha->m_dynamic_programming;
    m_ha->m_dynamic_programming = nullptr;
    return u();
  }
}

bool dynamic_programming::HybridAutomaton::Starting::invariant_holds()
{
  StateSpace state_space = Starting::get_state_space(m_ha->current_route_point());
  return state_space.contains(m_ha->m_x);
}

void dynamic_programming::HybridAutomaton::Starting::transition()
{
  const unit3& point = m_ha->current_route_point();
  float next_x0[6]{ (float)point.x, (float)point.y, (float)point.z, 0.f, 0.f, 0.f };
  StateSpace goal_space = Cruising::get_state_space(next_x0, m_ha->m_route.at((size_t)m_ha->m_route_counter + 1));
  if (goal_space.contains(m_ha->m_x))
    m_ha->do_transition<Cruising>(goal_space);
}

dynamic_programming::StateSpace dynamic_programming::HybridAutomaton::Starting::get_state_space(const unit3& point)
{
  return StateSpace
  {
    { point.x - 3, point.y - 3, 0, -5, -5, -5 },
    { STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE },
    { point.x + 3, point.y + 3, point.z + 1, 5, 5, 5 }
  };
}

const dynamic_programming::unit3 dynamic_programming::HybridAutomaton::Cruising::u()
{
  const unit3& point = m_ha->current_route_point();
  float x0[6]
  {
    m_ha->m_x[0] - point[0],
    m_ha->m_x[1] - point[1],
    m_ha->m_x[2] - point[2],
    m_ha->m_x[3],
    m_ha->m_x[4],
    m_ha->m_x[5]
  };

  if (m_ha->m_dynamic_programming == nullptr)
  {
    // Calculate controller
    StateSpace state_space = Cruising::get_state_space(x0, unit3::ZERO());
    StateSpace goal_space;
    // New goal point is cruising point again
    unit3 next = m_ha->m_route.at(m_ha->m_route_counter + 1);
    float next_x0[6]{ point.x, point.y, point.z, 0.f, 0.f, 0.f };
    if (m_ha->m_route.at((size_t)m_ha->m_route_counter + 1).z != 0.f)
    {
      goal_space = Cruising::get_state_space(next_x0, next);
    }
    else
    {
      goal_space = Landing::get_state_space(next_x0, next);
    }
    goal_space.offset(point);
    unit goal_extension[6]{ -1, -1, -1, -1, -1, -1 };
    goal_space.extend_absolute(goal_extension); // make smaller to allow for some rounding or other errors but still accept the final state in the simulation

    unit extend_for_long_distances_faster[6]{};
    for (int i = 0; i < 3; i++)
    {
      unit extend = state_space.get_range(i).length() / 20;
      if (extend > 10)
        extend = 10;
      extend_for_long_distances_faster[i + 3] = extend;
    }
    state_space.extend_absolute(extend_for_long_distances_faster);
    // goal_space.extend_absolute(extend_for_long_distances_faster); // TODO: Can't be only here!

    // Get stretch factor
    unit3 stretch_factor = choose_stretch_factor(state_space, goal_space, x0, this->name());
    // Apply stretching
    if (stretch_factor != unit3::ONE())
    {
      state_space.extend_for_stretching(stretch_factor);
      goal_space.extend_for_stretching(stretch_factor);
    }
    m_ha->m_dynamic_programming = new DynamicProgramming(
      state_space,
      goal_space,
      delta_time(),
      stretch_factor,
      [point](const unit3& world_point)
      {
        return unit3(world_point.x - point.x, world_point.y - point.y, world_point.z - point.z);
      },
      m_ha->m_dp_logger
    );
    long calculation_stopped_at = -1;
    while (true)
    {
      calculation_stopped_at = m_ha->m_dynamic_programming->calculate_controller(x0);
      BOOST_LOG_TRIVIAL(debug) << "calculation_stopped_at: " << calculation_stopped_at;
      if (calculation_stopped_at >= 0)
        break;
      BOOST_LOG_TRIVIAL(warning) << "Could not find path from x0 to 0. Recalculating controller with extended state space.";
      unit space_extension[6]{ 2, 2, 2, 0, 0, 0 };
      state_space.extend_absolute(space_extension);
      state_space.extend_for_stretching(stretch_factor);
      if (state_space.begin[2] < -point.z)
        state_space.begin[2] = -point.z;
      m_ha->m_dynamic_programming->reinitialize();
    }
    m_ha->m_major_time_counter = calculation_stopped_at;
  }

  // Use controller
  try
  {
    return m_ha->m_dynamic_programming->get_control(x0, m_ha->m_major_time_counter);
  }
  catch (const std::logic_error& e)
  {
    BOOST_LOG_TRIVIAL(error) << "logic error while getting control: " << e.what();
    m_ha->m_major_time_counter = 0;
    m_ha->m_minor_time_counter = 0;
    delete m_ha->m_dynamic_programming;
    m_ha->m_dynamic_programming = nullptr;
    return u();
  }
}

bool dynamic_programming::HybridAutomaton::Cruising::invariant_holds()
{
  StateSpace state_space = Cruising::get_state_space(m_ha->m_x, m_ha->current_route_point());
  return state_space.contains(m_ha->m_x);
}

void dynamic_programming::HybridAutomaton::Cruising::transition()
{
  const unit3& point = m_ha->current_route_point();
  float next_x0[6]{ (float)point.x, (float)point.y, (float)point.z, 0.f, 0.f, 0.f };
  // New goal point is cruising point again
  if (m_ha->m_route.at(m_ha->m_route_counter + 1).z != 0.f)
  {
    StateSpace goal_space = Cruising::get_state_space(next_x0, m_ha->m_route.at(m_ha->m_route_counter + 1));
    if (goal_space.contains(m_ha->m_x))
      m_ha->do_transition<Cruising>(goal_space);
  }
  // New goal point is final point
  else if (m_ha->m_route_counter + 1 == m_ha->m_route.size() - 1)
  {
    StateSpace goal_space = Landing::get_state_space(next_x0, m_ha->m_route.at(m_ha->m_route_counter + 1));
    if (goal_space.contains(m_ha->m_x))
      m_ha->do_transition<Landing>(goal_space);
  }
  else
  {
    throw std::logic_error("Route is not valid!");
  }
}

dynamic_programming::StateSpace dynamic_programming::HybridAutomaton::Cruising::get_state_space(const float* x, const unit3& point)
{
  return StateSpace
  {
    {
      std::min(point.x, (unit)x[0]) - 5,
      std::min(point.y, (unit)x[1]) - 5,
      std::min(point.z, (unit)x[2]) - 5,
      -5,
      -5,
      -5
    },
    { STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE },
    {
      std::max(point.x, (unit)x[0]) + 5,
      std::max(point.y, (unit)x[1]) + 5,
      std::max(point.z, (unit)x[2]) + 5,
      5,
      5,
      5 
    }
  };
}

const dynamic_programming::unit3 dynamic_programming::HybridAutomaton::Landing::u()
{
  const unit3& point = m_ha->current_route_point();
  float x0[6]
  {
    m_ha->m_x[0] - point[0],
    m_ha->m_x[1] - point[1],
    m_ha->m_x[2] - point[2],
    m_ha->m_x[3],
    m_ha->m_x[4],
    m_ha->m_x[5]
  };
  if (m_ha->m_dynamic_programming == nullptr)
  {
    // Calculate controller
    StateSpace state_space = Landing::get_state_space(m_ha->m_x, point);
    StateSpace goal_space = Landing::get_goal_space(point);
    // DON'T MAKE GOAL SPACE SMALLER!
    state_space.offset(point);
    goal_space.offset(point);
    m_ha->m_dynamic_programming = new DynamicProgramming(
      state_space,
      goal_space,
      delta_time(),
      unit3::ONE(),
      [point](const unit3& world_point)
      {
        return unit3(world_point.x - point.x, world_point.y - point.y, world_point.z - point.z);
      },
      m_ha->m_dp_logger
    );
    long calculation_stopped_at = -1;
    while (true)
    {
      calculation_stopped_at = m_ha->m_dynamic_programming->calculate_controller(x0);
      BOOST_LOG_TRIVIAL(debug) << "calculation_stopped_at: " << calculation_stopped_at;
      if (calculation_stopped_at >= 0)
        break;
      BOOST_LOG_TRIVIAL(warning) << "Could not find path from x0 to 0. Recalculating controller with extended state space.";
      unit space_extension[6]{ 2, 2, 2, 0, 0, 0 };
      state_space.extend_absolute(space_extension);
      if (state_space.begin[2] < -point.z)
        state_space.begin[2] = -point.z;
      m_ha->m_dynamic_programming->reinitialize();
    }
    m_ha->m_major_time_counter = calculation_stopped_at;
  }

  // Use controller
  try
  {
    return m_ha->m_dynamic_programming->get_control(x0, m_ha->m_major_time_counter);
  }
  catch (const std::logic_error& e)
  {
    BOOST_LOG_TRIVIAL(error) << "logic error while getting control: " << e.what();
    m_ha->m_major_time_counter = 0;
    m_ha->m_minor_time_counter = 0;
    delete m_ha->m_dynamic_programming;
    m_ha->m_dynamic_programming = nullptr;
    return u();
  }
}

bool dynamic_programming::HybridAutomaton::Landing::invariant_holds()
{
  StateSpace state_space = Landing::get_state_space(m_ha->m_x, m_ha->current_route_point());
  return state_space.contains(m_ha->m_x);
}

void dynamic_programming::HybridAutomaton::Landing::transition()
{
  const unit3& point = m_ha->current_route_point();
  StateSpace goal_space = Landing::get_goal_space(point);
  if (goal_space.contains(m_ha->m_x))
    m_ha->do_transition<Done>(goal_space);
}

dynamic_programming::StateSpace dynamic_programming::HybridAutomaton::Landing::get_state_space(const float* x, const unit3& point)
{
  if (x[2] <= 0)
    BOOST_LOG_TRIVIAL(warning) << "The z coordinate of the current state given to Landing::get_state_space is less or equal to 0. This will most likely result in a fatal crash.";
  return StateSpace
  {
    { point.x - 4, point.y - 4, 0, -5, -5, -5 },
    { STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE },
    { point.x + 4, point.y + 4, std::max((unit)x[2], point.z) + 1, 5, 5, 5 }
  };
}

dynamic_programming::StateSpace dynamic_programming::HybridAutomaton::Landing::get_goal_space(const unit3& point)
{
  return StateSpace
  {
    { point.x - 3, point.y - 3, 0, -3, -3, -3 },
    { STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE, STEP_SIZE },
    { point.x + 3, point.y + 3, 3, 3, 3, 3 }
  };
}

dynamic_programming::HybridAutomaton::HybridAutomaton(const std::vector<unit3>& route, DynamicProgramming::RuntimeLogger* dp_logger)
  : m_state(new Starting(this)), m_route(route), m_dp_logger(dp_logger)
{
  validate_route();
  for (int i = 0; i < 3; i++)
    m_x[i] = m_route.front()[i];
  for (int i = 3; i < 6; i++)
    m_x[i] = 0.f;
  m_route_counter++;
}

void dynamic_programming::HybridAutomaton::run_once()
{
  float delta_time_sim = m_state->delta_time() / R;
  m_state->do_flow(delta_time_sim);
  m_minor_time_counter++;
  if (m_minor_time_counter >= R)
  {
    m_minor_time_counter = 0;
    if (Config::get_instance().get(Config::USE_SINGLE_STAGE_CONTROLLER) == "true")
      m_major_time_counter++;
  }
  if(!m_state->invariant_holds())
  {
    BOOST_LOG_TRIVIAL(error) << "invariant does not hold!";
  }
  m_state->transition();
}

void dynamic_programming::HybridAutomaton::run_until_end()
{
  while (typeid(*m_state) != typeid(Done))
  {
    run_once();
  }
}

void dynamic_programming::HybridAutomaton::addEventListener(EventListener* listener)
{
  for (EventListener* l : m_listeners)
    if (l == listener)
      throw std::invalid_argument("EventListener already registered in this EventProducer");
  m_listeners.push_back(listener);
}

void dynamic_programming::HybridAutomaton::removeEventListener(EventListener* listener)
{
  m_listeners.erase(std::remove(m_listeners.begin(), m_listeners.end(), listener));
}

dynamic_programming::unit dynamic_programming::HybridAutomaton::get_delta_time(const State& state)
{
  return state.delta_time();
}

void dynamic_programming::HybridAutomaton::validate_route() const
{
  if (m_route.size() < 4)
    throw std::invalid_argument("Route must have at least four entries");

  const unit3& first = m_route.front();
  const unit3& second = m_route.at(1);
  const unit3& second_last = m_route.at(m_route.size() - 2);
  const unit3& last = m_route.back();

  // Check that the first point has z == 0
  // and that second point has same x and y as the first point
  if (first.z != 0.f)
    throw std::invalid_argument("First point on route must have its z coordinate equal to 0");
  if (first.x != second.x || first.y != second.y)
    throw std::invalid_argument("First and second points on route must have the same x and y coordinates");

  // Check that the last point has z == 0
  // and that the second last point has same x and y as the last point
  if (last.z != 0.f)
    throw std::invalid_argument("Last point on route must have its z coordinate equal to 0");
  if (last.x != second_last.x || last.y != second_last.y)
    throw std::invalid_argument("Last and second last points on route must have the same x and y coordinates");

  // Check that all except the first and second last points have z >= 10
  for (size_t i = 1; i < m_route.size() - 1; i++)
  {
    if (m_route.at(i).z < 10.f)
      throw std::invalid_argument("All except the first and second last points on route must have z >= 10");
  }
}

const dynamic_programming::unit3& dynamic_programming::HybridAutomaton::current_route_point() const
{
  if (m_route_counter < m_route.size())
    return m_route.at(m_route_counter);
  else
    return m_route.at(0);
}

double dynamic_programming::HybridAutomaton::distance_to_current_route_point() const
{
  double distance = 0.;
  for (int i = 0; i < 3; i++)
    distance += pow(m_x[i] - current_route_point()[i], 2);
  return sqrt(distance);
}

void dynamic_programming::HybridAutomaton::notify_state_changed(const State* old_state, const State* new_state, const unit3& new_point, const StateSpace& old_goal_space, const double& new_time)
{
  StateChangedEvent event
  {
    old_state,
    new_state,
    new_point,
    old_goal_space,
    new_time
  };
  for (EventListener* listener : m_listeners)
    listener->on_state_changed(event);
}

void dynamic_programming::HybridAutomaton::notify_x_changed(const float old_x[6], const float new_x[6], const unit3& u, const unit3& d, const double& new_time)
{
  XChangedEvent event
  {
    old_x,
    new_x,
    u,
    d,
    new_time
  };
  for (EventListener* listener : m_listeners)
    listener->on_x_changed(event);
}
