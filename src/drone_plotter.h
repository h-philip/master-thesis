#pragma once

#pragma warning(push, 0)
#include "gnuplot-iostream.h"
#pragma warning(pop)


#include "hybrid_automaton.h"
#include "collision_cloud.h"
#include "config.h"
#include <boost/log/trivial.hpp>
#include <unordered_set>
#include <unordered_map>

namespace dynamic_programming
{
  class DronePlotter : public HybridAutomaton::EventListener
  {
  public:
    DronePlotter(HybridAutomaton* ha, const std::string& directory_name);
    ~DronePlotter();

    void init_gnuplot();

    void flush();

    void on_state_changed(const HybridAutomaton::StateChangedEvent& event) override;

    void on_x_changed(const HybridAutomaton::XChangedEvent& event) override;

  private:
    void plot_values(Gnuplot& gp);

    void plot_path(Gnuplot& gp);

    void plot_2d_path(Gnuplot& gp);

    HybridAutomaton* m_ha;
    Gnuplot* m_gp = nullptr;
    Gnuplot* m_gp_values = nullptr;
    Gnuplot* m_gp_path = nullptr;
    Gnuplot* m_gp_2dpath = nullptr;
    FILE* m_file;
    FILE* m_file_values;
    FILE* m_file_path;
    FILE* m_file_2dpath;
    std::vector<std::pair<double, float>> m_disturbance_x;
    std::vector<std::pair<double, float>> m_disturbance_y;
    std::vector<std::pair<double, float>> m_disturbance_z;
    std::vector<std::pair<double, unit>> m_accel_x;
    std::vector<std::pair<double, unit>> m_accel_y;
    std::vector<std::pair<double, unit>> m_accel_z;
    std::vector<std::pair<double, float>> m_vel_x;
    std::vector<std::pair<double, float>> m_vel_y;
    std::vector<std::pair<double, float>> m_vel_z;
    std::vector<std::pair<double, float>> m_coord_x;
    std::vector<std::pair<double, float>> m_coord_y;
    std::vector<std::pair<double, float>> m_coord_z;
    std::vector<boost::tuple<float, float, float>> m_coordinates;
    std::vector<StateSpace> m_goal_spaces;
    std::string m_noxtime;
    std::string m_time;
    const std::string& m_out_path;
  };
}