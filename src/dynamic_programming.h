#pragma once

#include "collision_cloud.h"
#include "consts.h"
#include "matrix.h"
#include "range.h"
#include "state_space.h"
#include "config.h"
#include <boost/log/trivial.hpp>
#include <array>
#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <math.h>
#include <thread>
#include <tuple>

#define INCLUDE_O_IN_COST 1

namespace dynamic_programming {
  class DynamicProgramming
  {
    static const int INITIAL_REGION_RADIUS = 0;

    static const long INPUTS_SMALLER_STAGES = 100;
  public:

    class RuntimeLogger
    {
    public:
      struct DpStartedEvent
      {
        const size_t& num_states;
        const bool retry;
      };
      struct DpFinishedEvent
      {
        const std::chrono::seconds& total_duration;
        const std::chrono::milliseconds& first_stage_duration;
        const std::chrono::milliseconds& avg_stage_duration;
      };
      virtual void dp_started(const DpStartedEvent& event) = 0;
      virtual void dp_finished(const DpFinishedEvent& event) = 0;
    };

    DynamicProgramming(const StateSpace& state_space, const StateSpace& goal_space, const unit delta_time, const unit3 stretch_factor, std::function<unit3(const unit3&)> world_to_dp_coordinates, RuntimeLogger* logger);
    ~DynamicProgramming();

    void set_runtime_logger(RuntimeLogger* runtime_logger)
    {
      m_runtime_logger = runtime_logger;
    }

    void reinitialize();

    long calculate_controller(float x0[6]);

    const unit3 get_control(const float x[6], long i_time) const;

  private:
    float terminal_cost(const unit x[6]) const;

    float running_cost(const unit x[6], const unit3 &input, const int i_c1, const int i_c2, const int i_c3) const;

    void calculate_one_stage_threaded(const long stage, const size_t start_i_v1, const size_t end_i_v1, size_t* finite_states, const unit3* inputs);

    static const size_t NUM_THREADS = 16;

    bool initial_region_is_covered(const long i_time, const int i_x0[6]);

    std::vector<std::tuple<int, int, int, int, int, int>>& get_initial_region(const int i_x0[6]);

    size_t fill_terminal_costs();

    RuntimeLogger* m_runtime_logger = nullptr;
    int m_num_disturbances = Config::get_instance().get(Config::DISTURBANCE_ON) == "true" ? NUM_DISTURBANCES : 1;
    const int* m_i_x0 = nullptr;
    std::vector<std::tuple<int, int, int, int, int, int>> m_initial_region;
    Range m_grids[6];
    size_t m_lengths[6];
    matrix<float>* m_V = nullptr;
    matrix<int>* m_u_opt = nullptr;
#ifdef INCLUDE_O_IN_COST
    boost::multi_array<float, 3>* m_o_cost = nullptr;
    bool m_o_cost_used;
#endif
    const StateSpace& m_state_space;
    const StateSpace& m_goal_space;
    const float m_delta_time;
    CollisionCloud* m_collision_cloud = nullptr;
    std::function<unit3(unit3)> m_world_to_dp_coordinates;
    const unit3 m_stretch_factor;
    const bool m_stretching = false;
    unit3 m_smaller_inputs[NUM_INPUTS]{};
    unit3 m_larger_inputs[NUM_INPUTS]{};
    unit3 m_disturbances[NUM_DISTURBANCES]{};
    bool m_break_on_initial_region_covered_fixpoint_reached;
    bool m_break_on_norm_fixpoint_reached;
  };
}