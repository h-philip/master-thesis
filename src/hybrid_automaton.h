#pragma once

#include "consts.h"
#include "disturbance_controller.h"
#include "dynamic_programming.h"
#include "stretch_utils.h"
#include "state_space.h"
#include <boost/log/trivial.hpp>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace dynamic_programming
{
  class HybridAutomaton
  {
  public:

    struct State
    {
      State(HybridAutomaton* ha) : m_ha(ha) {}
      virtual ~State() {}
      virtual const unit3 u() = 0;
      virtual void do_flow(float delta_time_sim);
      virtual bool invariant_holds() = 0;
      virtual void transition() = 0;
      virtual unit delta_time() const = 0;
      virtual std::string name() const { return "State"; }

    protected:
      HybridAutomaton* m_ha;
    };
    struct Starting : public State
    {
      Starting(HybridAutomaton* ha) : State(ha) {}
      ~Starting() override {}
      const unit3 u() override;
      bool invariant_holds() override;
      void transition() override;
      unit delta_time() const override { return 1; }
      std::string name() const override { return "Starting"; }
      static StateSpace get_state_space(const unit3& point);
    };
    struct Cruising : public State
    {
      Cruising(HybridAutomaton* ha) : State(ha) {}
      ~Cruising() override {}
      const unit3 u() override;
      bool invariant_holds() override;
      void transition() override;
      unit delta_time() const override { return 1; }
      std::string name() const override { return "Cruising"; }
      static StateSpace get_state_space(const float* x, const unit3& point);
    };
    struct Landing : public State
    {
      Landing(HybridAutomaton* ha) : State(ha) {}
      ~Landing() override {}
      const unit3 u() override;
      bool invariant_holds() override;
      void transition() override;
      unit delta_time() const override { return 1; }
      std::string name() const override { return "Landing"; }
      static StateSpace get_state_space(const float* x, const unit3& point);
      static StateSpace get_goal_space(const unit3& point);
    };
    struct Done : public State
    {
      Done(HybridAutomaton* ha) : State(ha) {}
      ~Done() override {}
      const unit3 u() override { return unit3::ZERO(); }
      void do_flow(float delta_time_sim) override { (void)delta_time_sim; }
      bool invariant_holds() override { return false; }
      void transition() override {}
      std::string name() const override { return "Done"; }
      unit delta_time() const override { return 0; }

    };

    struct StateChangedEvent
    {
      const State* old_state;
      const State* new_state;
      const unit3& new_point;
      const StateSpace& old_goal_space;
      const double& new_time;
    };
    struct XChangedEvent
    {
      const float* old_x;
      const float* new_x;
      const unit3& u;
      const unit3& d;
      const double& new_time;
    };
    class EventListener
    {
    public:
      virtual void on_state_changed(const StateChangedEvent& event) = 0;
      virtual void on_x_changed(const XChangedEvent& event) = 0;
    };

    HybridAutomaton(const std::vector<unit3>& route, DynamicProgramming::RuntimeLogger* dp_logger);

    ~HybridAutomaton()
    {
      delete m_state;
    };

    void run_once();

    void run_until_end();

    void addEventListener(EventListener* listener);

    void removeEventListener(EventListener* listener);

    const std::vector<unit3>& get_route() const { return m_route; }

  private:
    static unit get_delta_time(const State& state);

    void validate_route() const;

    const unit3& current_route_point() const;

    double distance_to_current_route_point() const;

    template <typename T>
    void do_transition(const StateSpace& old_goal_space);

    void notify_state_changed(const State* old_state, const State* new_state, const unit3& new_point, const StateSpace& new_goal_space, const double& new_time);

    void notify_x_changed(const float old_x[6], const float new_x[6], const unit3& u, const unit3& d, const double& new_time);

    HybridAutomaton::State* m_state;
    DynamicProgramming::RuntimeLogger* m_dp_logger;
    float m_x[6]{};
    double m_time = 0.;
    const std::vector<unit3>& m_route;
    size_t m_route_counter = 0u;
    DynamicProgramming* m_dynamic_programming;
    long m_major_time_counter = 0;
    long m_minor_time_counter = 0;
    std::vector<EventListener*> m_listeners = std::vector<EventListener*>();
    DisturbanceController m_disturbance_controller;
  };

  template <typename T>
  inline void HybridAutomaton::do_transition(const StateSpace& old_goal_space)
  {
    static_assert(std::is_base_of<State, T>::value, "T must be a State");

    State* old_state = m_state;

    if (std::is_same<T, Starting>::value)
      m_state = new Starting(this);
    else if (std::is_same<T, Cruising>::value)
      m_state = new Cruising(this);
    else if (std::is_same<T, Landing>::value)
      m_state = new Landing(this);
    else
      m_state = new Done(this);

    m_route_counter++;
    m_major_time_counter = 0;
    m_minor_time_counter = 0;

    notify_state_changed(old_state, m_state, current_route_point(), old_goal_space, m_time);

    delete old_state;
    delete m_dynamic_programming;
    m_dynamic_programming = nullptr;
  }
}