#pragma once

#include "dynamic_programming.h"
#include <iostream>
#include "windows.h"
#include "psapi.h"

namespace dynamic_programming
{
  class DpStats : public DynamicProgramming::RuntimeLogger
  {
  public:
    DpStats(const std::string& directory_path)
      : m_directory_path(directory_path)
    {
      m_file.open(directory_path + "dp_stats.txt");
    }

    ~DpStats()
    {
      m_file.close();
    }

    void dp_started(const DpStartedEvent& event) override;

    void dp_finished(const DpFinishedEvent& event) override;

  private:
    void resource_usage();

    const std::string m_directory_path;
    std::ofstream m_file;
    bool m_end_thread = false;
    std::thread m_thread;
  };
}