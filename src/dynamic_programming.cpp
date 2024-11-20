#include "dynamic_programming.h"

using namespace std;

dynamic_programming::DynamicProgramming::DynamicProgramming(const StateSpace& state_space, const StateSpace& goal_space, const unit delta_time, const unit3 stretch_factor, std::function<unit3(const unit3&)> world_to_dp_coordinates, RuntimeLogger* logger)
  : m_state_space(state_space),
  m_goal_space(goal_space),
  m_delta_time(delta_time),
  m_world_to_dp_coordinates(world_to_dp_coordinates),
  m_stretch_factor(stretch_factor),
  m_stretching(stretch_factor.x > 1 || stretch_factor.y > 1 || stretch_factor.z > 1)
{
  this->set_runtime_logger(logger);
  reinitialize();
}

dynamic_programming::DynamicProgramming::~DynamicProgramming()
{
  delete m_V;
  delete m_u_opt;
#ifdef INCLUDE_O_IN_COST
  delete m_o_cost;
#endif
  delete m_collision_cloud;
}

void dynamic_programming::DynamicProgramming::reinitialize()
{
  // Stretch factor
  BOOST_LOG_TRIVIAL(debug) << "stretch factor: " << m_stretch_factor.to_string();
  _ASSERT_EXPR(m_stretch_factor.x >= 1, "stretch factor must be equal to or greater than 1 (x)");
  _ASSERT_EXPR(m_stretch_factor.y >= 1, "stretch factor must be equal to or greater than 1 (y)");
  _ASSERT_EXPR(m_stretch_factor.z >= 1, "stretch factor must be equal to or greater than 1 (z)");

  // State space
  size_t num_states = 1;
  BOOST_LOG_TRIVIAL(debug) << "### state space ###";
  for (int i = 0; i < 6; i++)
  {
    m_grids[i] = m_state_space.get_range(i);
    m_grids[i].set_begin(m_grids[i].get_begin() / m_stretch_factor[i % 3]);
    m_grids[i].set_end(m_grids[i].get_end() / m_stretch_factor[i % 3]);
    m_lengths[i] = m_grids[i].length();
    num_states *= m_lengths[i];
    BOOST_LOG_TRIVIAL(debug) << i << ": " << m_grids[i].to_string() << " (length: " << m_lengths[i] << ")";
  }
  BOOST_LOG_TRIVIAL(debug) << "Total number of states per stage: " << num_states;

  // Goal space
  BOOST_LOG_TRIVIAL(debug) << "### goal space ###";
  for (int i = 0; i < 6; i++)
  {
    Range r = m_goal_space.get_range(i);
    r.set_begin(std::min((unit)-r.get_step(), r.get_begin() / m_stretch_factor[i % 3]));
    r.set_end(std::max((unit)r.get_step(), r.get_end() / m_stretch_factor[i % 3]));
    BOOST_LOG_TRIVIAL(debug) << i << ": " << r.to_string();
  }

  bool retry = m_V != nullptr;

  // Delete dynamic memory if allocated
  if (m_V != nullptr)
    delete m_V;
  if (m_u_opt != nullptr)
    delete m_u_opt;
#ifdef INCLUDE_O_IN_COST
  if (m_o_cost != nullptr)
        delete m_o_cost;
#endif
  if (m_collision_cloud != nullptr)
    delete m_collision_cloud;

  // Reset inputs
  unit3 smaller{};
  unit3 larger{};
  for (int i = 0; i < 3; i++)
  {
    unit factor = m_stretch_factor[i];
    if (factor > 2)
      smaller[i] = 1;
    else
      smaller[i] = 2;

    if (factor > 5)
      larger[i] = 1;
    else if (factor > 3)
      larger[i] = 2;
    else if (factor > 2)
      larger[i] = 3;
    else
      larger[i] = 4;
  }
  create_inputs(m_smaller_inputs, smaller);
  create_inputs(m_larger_inputs, larger);

  // Reset disturbances
  for (int i = 0; i < m_num_disturbances; i++)
    m_disturbances[i] = DISTURBANCES[i] / m_stretch_factor;

  // Get number of stages
  Config& config = Config::get_instance();
  int stages = config.get<int>(Config::Key::NUMBER_OF_STAGES);

  // (Re-)create matrices and collision cloud instance
  m_V = new matrix<float>(stages, m_lengths[0], m_lengths[1], m_lengths[2], m_lengths[3], m_lengths[4], m_lengths[5]);
  m_u_opt = new matrix<int>(stages, m_lengths[0], m_lengths[1], m_lengths[2], m_lengths[3], m_lengths[4], m_lengths[5]);
  m_o_cost = new boost::multi_array<float, 3>(boost::extents[m_lengths[0]][m_lengths[1]][m_lengths[2]]);
  m_collision_cloud = new CollisionCloud(m_lengths[0], m_lengths[1], m_lengths[2], STEP_SIZE);
  m_collision_cloud->add_collisions_from_file(Config::get_instance().get(Config::Key::COLLISION_CLOUD_FILE),
    [this](const unit3 &world_point)
    {
      unit3 dp_point = m_world_to_dp_coordinates(world_point);
      dp_point /= m_stretch_factor;
      return CollisionCloud::point3(m_grids[0].search_closest(dp_point.x), m_grids[1].search_closest(dp_point.y), m_grids[2].search_closest(dp_point.z));
    }
  );

  // Reset other variables
  m_i_x0 = nullptr;
  m_initial_region.clear();
  m_break_on_initial_region_covered_fixpoint_reached = config.get<bool>(Config::Key::ENABLE_INITIAL_FIX_POINT);
  m_break_on_norm_fixpoint_reached = config.get<bool>(Config::Key::ENABLE_NORM_FIX_POINT);

  RuntimeLogger::DpStartedEvent event
  {
    num_states,
    retry
  };
  if (m_runtime_logger != nullptr)
    m_runtime_logger->dp_started(event);
}

long dynamic_programming::DynamicProgramming::calculate_controller(float x0[6])
{
  std::chrono::steady_clock::time_point total_begin = std::chrono::steady_clock::now();

  // Get index of x0
  int i_x0[6]{};
  for (int i = 0; i < 6; i++)
    i_x0[i] = m_grids[i].search(x0[i] / m_stretch_factor[i % 3]);

  // Fill terminal costs
  BOOST_LOG_TRIVIAL(debug) << "### final stage ###";
  size_t terminal_states = fill_terminal_costs();
  BOOST_LOG_TRIVIAL(debug) << "Number of states in goal space: " << terminal_states;

#ifdef INCLUDE_O_IN_COST
  // Precalculate m_o_cost
  Config& config = Config::get_instance();
  if (!config.is_set(Config::Key::COLLISION_COST_FACTOR) || config.get<float>(Config::Key::COLLISION_COST_FACTOR) == 0.f)
  {
    m_o_cost_used = false;
    BOOST_LOG_TRIVIAL(debug) << "Collision cost factor is 0. Skipping precalculation of o_cost.";
  }
  else if (m_collision_cloud->get_collisions().empty())
  {
    m_o_cost_used = false;
    BOOST_LOG_TRIVIAL(debug) << "Collision cloud is empty. Skipping precalculation of o_cost.";
  }
  else
  {
    m_o_cost_used = true;
    BOOST_LOG_TRIVIAL(debug) << "### precalculate o_cost ###";
    float factor = Config::get_instance().get<float>(Config::Key::COLLISION_COST_FACTOR);
    for (int i_c1 = 0; i_c1 < m_lengths[0]; i_c1++)
    {
      for (int i_c2 = 0; i_c2 < m_lengths[1]; i_c2++)
      {
        for (int i_c3 = 0; i_c3 < m_lengths[2]; i_c3++)
        {
          // Iterate over all obstacles in the collision cloud to find the one which is closest to c
          std::vector<CollisionCloud::point3>& collisions = m_collision_cloud->get_collisions();
          float min_distance_2 = numeric_limits<float>::max();
          for (CollisionCloud::point3& collision : collisions)
          {
            CollisionCloud::point3 subtraction(i_c1 - collision.x(), i_c2 - collision.y(), i_c3 - collision.z());
            float distance_2 = bg::dot_product(subtraction, subtraction);
            if (distance_2 < min_distance_2)
            {
              min_distance_2 = distance_2;
            }
          }

          // Calculate cost from closest collision
          float cost = 0;
          cost = factor / (sqrt(min_distance_2));
          (*m_o_cost)[i_c1][i_c2][i_c3] = cost;
        }
      }
    }
  }
#endif

  // Get number of stages
  int stages = config.get<int>(Config::Key::NUMBER_OF_STAGES);

  // Set up threads
  thread threads[NUM_THREADS];
  size_t chunk_size = m_lengths[3] / NUM_THREADS;
  size_t rest = m_lengths[3] - chunk_size * NUM_THREADS;

  const unit3* inputs = m_smaller_inputs;

  bool x0_reached = false;
  int finite_states_changed = 0;
  size_t last_finite_states = 0;

  std::vector<std::chrono::nanoseconds> stage_durations;

  BOOST_LOG_TRIVIAL(debug) << "### recursive calculation of optimal cost-to-go ###";
  long i_time = stages - 2;
  for ( ; i_time >= 0; i_time--)
  {
    std::chrono::steady_clock::time_point stage_begin = std::chrono::steady_clock::now();

    if (stages - i_time > INPUTS_SMALLER_STAGES)
      inputs = m_larger_inputs;

    size_t all_finite_states = 0;
    size_t finite_states[NUM_THREADS]{ 0 };

    size_t start = 0;
    size_t end = 0;

    for (size_t i_thread = 0; i_thread < NUM_THREADS; i_thread++)
    {
      start = end;
      end += chunk_size;
      if (i_thread < rest)
        end++;

      threads[i_thread] = thread(&DynamicProgramming::calculate_one_stage_threaded, this, i_time, start, end, &(finite_states[i_thread]), inputs);
    }

    _ASSERT_EXPR(end == m_lengths[3], "Calculation of thread chunk sizes failed");

    for (size_t i_thread = 0; i_thread < NUM_THREADS; i_thread++)
    {
      threads[i_thread].join();
      all_finite_states += finite_states[i_thread];
    }

    std::chrono::nanoseconds duration = std::chrono::steady_clock::now() - stage_begin;
    stage_durations.push_back(duration);
    BOOST_LOG_TRIVIAL(debug) << "Stage " << i_time << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms. Number of states with finite cost-to-go: " << all_finite_states;

    // Check if number of finite states has changed
    if (all_finite_states == last_finite_states)
      finite_states_changed++;
    else
      finite_states_changed = 0;

    // Nothing has changed for three stages
    if (finite_states_changed == 2)
    {
      BOOST_LOG_TRIVIAL(debug) << "Number of finite cost states hasn't changed in three stages. Fix point has probably been reached.";
      if (m_break_on_norm_fixpoint_reached)
      {
        i_time--;
        break;
      }
    }
    if (initial_region_is_covered(i_time, i_x0))
    {
      BOOST_LOG_TRIVIAL(debug) << "Initial region is covered. Shortest path has been calculated.";
      if (m_break_on_initial_region_covered_fixpoint_reached)
      {
        i_time--;
        break;
      }
    }

    last_finite_states = all_finite_states;
  }

  i_time++;

  RuntimeLogger::DpFinishedEvent event
  {
    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - total_begin),
    std::chrono::duration_cast<std::chrono::milliseconds>(stage_durations[0]),
    std::chrono::duration_cast<std::chrono::milliseconds>(std::accumulate(stage_durations.begin(), stage_durations.end(), std::chrono::nanoseconds(0)) / stage_durations.size())
  };
  if (m_runtime_logger != nullptr)
    m_runtime_logger->dp_finished(event);

  if (initial_region_is_covered(i_time, i_x0))
  {
    BOOST_LOG_TRIVIAL(debug) << "Initial region is covered.";
    return i_time;
  }
  else
  {
    BOOST_LOG_TRIVIAL(debug) << "Initial region is not covered.";
    return -1;
  }
}

const dynamic_programming::unit3 dynamic_programming::DynamicProgramming::get_control(const float x[6], long i_time) const
{
  Config& config = Config::get_instance();
  int stages = config.get<int>(Config::Key::NUMBER_OF_STAGES);
  const unit3* inputs = stages - i_time > INPUTS_SMALLER_STAGES ? m_larger_inputs : m_smaller_inputs;
  if (i_time >= stages - 1)
    throw std::logic_error("It took too many stages to reach 0. Controller wasn't calculated that far.");
  int i_x[6]{};
  for (int i = 0; i < 6; i++)
    i_x[i] = m_grids[i].search(x[i] / m_stretch_factor[i % 3]);

  int i_u = m_u_opt->at(i_time, i_x[0], i_x[1], i_x[2], i_x[3], i_x[4], i_x[5]);
  if (i_u < 0 || i_u > NUM_INPUTS)
    throw std::logic_error("Controller returned invalid optimal u index: " + std::to_string(i_u));
  return inputs[i_u] * m_stretch_factor;
}

float dynamic_programming::DynamicProgramming::terminal_cost(const unit x[6]) const
{
  bool contains;
  if (m_stretching)
  {
    unit x_stretched[6]{};
    for (int i = 0; i < 3; i++)
    {
      x_stretched[i] = x[i] * m_stretch_factor[i];
      x_stretched[i + 3] = x[i + 3] * m_stretch_factor[i];
    }
    contains = m_goal_space.contains(x_stretched);
  }
  else
  {
    contains = m_goal_space.contains(x);
  }
  return contains ? 0.f : numeric_limits<float>::max();
}

float dynamic_programming::DynamicProgramming::running_cost(const unit x[6], const unit3& input, const int i_c1, const int i_c2, const int i_c3) const
{
  if (m_stretching)
  {
    unit x_stretched[6]{};
    for (int i = 0; i < 3; i++)
    {
      x_stretched[i] = x[i] * m_stretch_factor[i];
      x_stretched[i + 3] = x[i + 3] * m_stretch_factor[i];
    }
    if (m_goal_space.contains(x_stretched))
      return 0.f;
  }
  else
  {
    if (m_goal_space.contains(x))
      return 0.f;
  }

  float cost = input.x * input.x + input.y * input.y + input.z * input.z;
  for (int i = 0; i < 6; i++)
    cost += x[i] * x[i];
#ifdef INCLUDE_O_IN_COST
  if (m_o_cost_used)
    cost += (*m_o_cost)[i_c1][i_c2][i_c3];
#endif
  return cost * m_delta_time;
}

void dynamic_programming::DynamicProgramming::calculate_one_stage_threaded(const long stage, const size_t start_i_v1, const size_t end_i_v1, size_t* finite_states, const unit3* inputs)
{
  unit drag;
  // Allocate arrays only once to maybe save runtime
  unit new_v1s[NUM_INPUTS][NUM_DISTURBANCES]{};
  int i_new_v1s[NUM_INPUTS][NUM_DISTURBANCES]{};
  unit new_v2s[NUM_INPUTS][NUM_DISTURBANCES]{};
  int i_new_v2s[NUM_INPUTS][NUM_DISTURBANCES]{};
  unit new_v3s[NUM_INPUTS][NUM_DISTURBANCES]{};
  int i_new_v3s[NUM_INPUTS][NUM_DISTURBANCES]{};
  unit new_c1s[NUM_INPUTS][NUM_DISTURBANCES]{};
  int i_new_c1s[NUM_INPUTS][NUM_DISTURBANCES]{};
  unit new_c2s[NUM_INPUTS][NUM_DISTURBANCES]{};
  int i_new_c2s[NUM_INPUTS][NUM_DISTURBANCES]{};
  unit new_c3s[NUM_INPUTS][NUM_DISTURBANCES]{};
  int i_new_c3s[NUM_INPUTS][NUM_DISTURBANCES]{};
  bool valid[NUM_INPUTS][NUM_DISTURBANCES]{};

  // x velocity
  for (size_t i_v1 = start_i_v1; i_v1 < end_i_v1; i_v1++)
  {
    unit v1 = m_grids[3][i_v1];

    for (int i = 0; i < NUM_INPUTS; i++)
      for (int j = 0; j < m_num_disturbances; j++)
      {
        drag = -DRAG_FORCE_COEFFICIENT * v1;
        new_v1s[i][j] = v1 + (inputs[i].x + m_disturbances[j].x + drag) * m_delta_time;
        i_new_v1s[i][j] = m_grids[3].search(new_v1s[i][j]);
      }

    // y velocity
    for (int i_v2 = 0; i_v2 < m_lengths[4]; i_v2++)
    {
      unit v2 = m_grids[4][i_v2];

      for (int i = 0; i < NUM_INPUTS; i++)
        for (int j = 0; j < m_num_disturbances; j++)
        {
          drag = -DRAG_FORCE_COEFFICIENT * v2;
          new_v2s[i][j] = v2 + (inputs[i].y + m_disturbances[j].y + drag) * m_delta_time;
          i_new_v2s[i][j] = m_grids[4].search(new_v2s[i][j]);
        }

      // z velocity
      for (int i_v3 = 0; i_v3 < m_lengths[5]; i_v3++)
      {
        unit v3 = m_grids[5][i_v3];

        for (int i = 0; i < NUM_INPUTS; i++)
          for (int j = 0; j < m_num_disturbances; j++)
          {
            drag = -DRAG_FORCE_COEFFICIENT * v3;
            new_v3s[i][j] = v3 + (inputs[i].z + m_disturbances[j].z + drag) * m_delta_time;
            i_new_v3s[i][j] = m_grids[5].search(new_v3s[i][j]);
          }

        // x coordinate
        for (int i_c1 = 0; i_c1 < m_lengths[0]; i_c1++)
        {
          unit c1 = m_grids[0][i_c1];

          for (int i = 0; i < NUM_INPUTS; i++)
            for (int j = 0; j < m_num_disturbances; j++)
            {
              new_c1s[i][j] = c1 + new_v1s[i][j] * m_delta_time;
              i_new_c1s[i][j] = m_grids[0].search(new_c1s[i][j]);
            }

          // y coordinate
          for (int i_c2 = 0; i_c2 < m_lengths[1]; i_c2++)
          {
            unit c2 = m_grids[1][i_c2];

            for (int i = 0; i < NUM_INPUTS; i++)
              for (int j = 0; j < m_num_disturbances; j++)
              {
                new_c2s[i][j] = c2 + new_v2s[i][j] * m_delta_time;
                i_new_c2s[i][j] = m_grids[1].search(new_c2s[i][j]);
              }

            // z coordinate
            for (int i_c3 = 0; i_c3 < m_lengths[2]; i_c3++)
            {
              unit c3 = m_grids[2][i_c3];

              for (int i = 0; i < NUM_INPUTS; i++)
                for (int j = 0; j < m_num_disturbances; j++)
                {
                  new_c3s[i][j] = c3 + new_v3s[i][j] * m_delta_time;
                  i_new_c3s[i][j] = m_grids[2].search(new_c3s[i][j]);
                }

              bool any_valid = false;
              for (int i = 0; i < NUM_INPUTS; i++)
                for (int j = 0; j < m_num_disturbances; j++)
                {
                  bool v = true;
                  v &= i_new_v1s[i][j] != -1;
                  v &= i_new_v2s[i][j] != -1;
                  v &= i_new_v3s[i][j] != -1;
                  v &= i_new_c1s[i][j] != -1;
                  v &= i_new_c2s[i][j] != -1;
                  v &= i_new_c3s[i][j] != -1;
                  valid[i][j] = v;
                  any_valid |= v;
                }

              //if (i_c1 == 2 && i_c2 == 1 && i_c3 == 4 && i_v1 == 1 && i_v2 == 1 && i_v3 == 4)
              //{
              //  int i_dd = 4;
              //  int timee = 96;
              //}

              if (any_valid)
              {
                float min_cost_to_go = numeric_limits<float>::max();
                int argmin_cost_to_go = -1;
                for (int i = 0; i < NUM_INPUTS; i++)
                {
                  float max_cost_to_go = numeric_limits<float>::lowest();
                  int argmax_cost_to_go = -1;
                  for (int j = 0; j < m_num_disturbances; j++)
                  {
                    float cost_to_go;
                    if (!valid[i][j])
                    {
                      cost_to_go = numeric_limits<float>::max();
                    }
                    else
                    {
                      unit x[6]{ new_c1s[i][j], new_c2s[i][j], new_c3s[i][j], new_v1s[i][j], new_v2s[i][j], new_v3s[i][j] };
                      CollisionCloud::point3 i_old_c((size_t)i_c1, (size_t)i_c2, (size_t)i_c3);
                      CollisionCloud::point3 i_new_c((size_t)i_new_c1s[i][j], (size_t)i_new_c2s[i][j], (size_t)i_new_c3s[i][j]);
                      bool colliding = m_collision_cloud->will_collide(i_old_c, i_new_c);
                      float running_costs = colliding ? numeric_limits<float>::max() : running_cost(x, inputs[i], i_c1, i_c2, i_c3);

                      float next_cost_to_go = m_V->at(stage + 1, i_new_c1s[i][j], i_new_c2s[i][j], i_new_c3s[i][j], i_new_v1s[i][j], i_new_v2s[i][j], i_new_v3s[i][j]);
                      cost_to_go = running_costs + next_cost_to_go;
                    }
                    if (cost_to_go > max_cost_to_go)
                    {
                      max_cost_to_go = cost_to_go;
                      argmax_cost_to_go = j;
                    }
                  }
                  if (max_cost_to_go < min_cost_to_go && argmax_cost_to_go != -1)
                  {
                    min_cost_to_go = max_cost_to_go;
                    argmin_cost_to_go = i;
                  }
                }

                m_V->at(stage, i_c1, i_c2, i_c3, i_v1, i_v2, i_v3) = min_cost_to_go;
                m_u_opt->at(stage, i_c1, i_c2, i_c3, i_v1, i_v2, i_v3) = argmin_cost_to_go;
                if (min_cost_to_go < numeric_limits<float>::max())
                  (*finite_states)++;
              }
              else
              {
                m_V->at(stage, i_c1, i_c2, i_c3, i_v1, i_v2, i_v3) = numeric_limits<float>::max();
                m_u_opt->at(stage, i_c1, i_c2, i_c3, i_v1, i_v2, i_v3) = -1;
              }
            }
          }
        }
      }
    }
  }
}

bool dynamic_programming::DynamicProgramming::initial_region_is_covered(const long i_time, const int i_x0[6])
{
  auto& initial_region = get_initial_region(i_x0);
  for (auto& tuple : initial_region)
    if (m_V->at(i_time, std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple), std::get<3>(tuple), std::get<4>(tuple), std::get<5>(tuple)) >= std::numeric_limits<float>::max())
      return false;
  return true;
}

std::vector<std::tuple<int, int, int, int, int, int>>& dynamic_programming::DynamicProgramming::get_initial_region(const int i_x0[6])
{
  if (m_i_x0 != i_x0 || m_initial_region.empty())
  {
    int r = INITIAL_REGION_RADIUS;

    m_i_x0 = i_x0;
    m_initial_region.clear();
    for (int cx = i_x0[0] - r; cx <= i_x0[0] + r; cx++)
      if (cx >= 0 && cx < m_lengths[0])
        for (int cy = i_x0[1] - r; cy <= i_x0[1] + r; cy++)
          if (cy >= 0 && cy < m_lengths[1])
            for (int cz = i_x0[2] - r; cz <= i_x0[2] + r; cz++)
              if (cz >= 0 && cz < m_lengths[2])
                for (int vx = i_x0[3] - r; vx <= i_x0[3] + r; vx++)
                  if (vx >= 0 && vx < m_lengths[3])
                    for (int vy = i_x0[4] - r; vy <= i_x0[4] + r; vy++)
                      if (vy >= 0 && vy < m_lengths[4])
                        for (int vz = i_x0[5] - r; vz <= i_x0[5] + r; vz++)
                          if (vz >= 0 && vz < m_lengths[5])
                          {
                            m_initial_region.push_back(std::make_tuple(cx, cy, cz, vx, vy, vz));
                          }
  }
  return m_initial_region;
}

size_t dynamic_programming::DynamicProgramming::fill_terminal_costs()
{
  Config& config = Config::get_instance();
  int stages = config.get<int>(Config::Key::NUMBER_OF_STAGES);
  size_t count = 0;
  for (int c1 = 0; c1 < m_lengths[0]; c1++)
  {
    for (int c2 = 0; c2 < m_lengths[1]; c2++)
    {
      for (int c3 = 0; c3 < m_lengths[2]; c3++)
      {
        for (int v1 = 0; v1 < m_lengths[3]; v1++)
        {
          for (int v2 = 0; v2 < m_lengths[4]; v2++)
          {
            for (int v3 = 0; v3 < m_lengths[5]; v3++)
            {
              const unit x[6]{ m_grids[0][c1], m_grids[1][c2], m_grids[2][c3], m_grids[3][v1], m_grids[4][v2], m_grids[5][v3] };
              float c = terminal_cost(x);
              m_V->at(stages - 1, c1, c2, c3, v1, v2, v3) = c;
              if (c == 0.f)
                count++;
            }
          }
        }
      }
    }
  }
  return count;
}
