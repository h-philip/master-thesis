#pragma once

#include "hybrid_automaton.h"
#include <boost/log/trivial.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>

namespace dynamic_programming
{
  class DroneLogger : public HybridAutomaton::EventListener
  {
  public:
    DroneLogger(HybridAutomaton *ha) : m_ha(ha)
    {
      m_ha->addEventListener(this);
    }

    ~DroneLogger()
    {
      m_ha->removeEventListener(this);
      if (m_file)
      {
        m_file_stream.close();
      }
    }

    void flush();

    void log_to_file(const std::string file);

    void on_state_changed(const HybridAutomaton::StateChangedEvent& event) override;

    void on_x_changed(const HybridAutomaton::XChangedEvent& event) override;

  private:

    static std::string units_to_string(const unit* f, const size_t length);
    
    HybridAutomaton *m_ha;
    std::ofstream m_file_stream = std::ofstream();
    bool m_file = false;
  };
}