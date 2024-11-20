#include "dp_stats.h"

void dynamic_programming::DpStats::dp_started(const DpStartedEvent& event)
{
  if (!event.retry)
  {
    m_file << "##################################################" << std::endl;
    m_file << "DP started" << std::endl;
    m_file << "num_states=" << event.num_states << std::endl;
  }
  else
  {
    m_file << "DP started (retry)" << std::endl;
    m_file << "num_states=" << event.num_states << std::endl;
  }
  // Start new thread
  m_end_thread = false;
  m_thread = std::thread(&DpStats::resource_usage, this);
}

void dynamic_programming::DpStats::dp_finished(const DpFinishedEvent& event)
{
  m_end_thread = true;
  m_thread.join();

  m_file << "DP finished" << std::endl;
  m_file << "total_duration_s=" << event.total_duration.count() << std::endl;
  m_file << "total_duration_m=" << std::chrono::duration_cast<std::chrono::minutes>(event.total_duration).count() << std::endl;
  m_file << "first_stage_duration_ms=" << event.first_stage_duration.count() << std::endl;
  m_file << "first_stage_duration_s=" << std::chrono::duration_cast<std::chrono::seconds>(event.first_stage_duration).count() << std::endl;
  m_file << "avg_stage_duration_ms=" << event.avg_stage_duration.count() << std::endl;
  m_file << "avg_stage_duration_s=" << std::chrono::duration_cast<std::chrono::seconds>(event.avg_stage_duration).count() << std::endl;
}

void dynamic_programming::DpStats::resource_usage()
{
  size_t vmem_max = 0;
  std::vector<size_t> vmem;

  while (!m_end_thread)
  {
    // Get current virtual memory usage using GetProcessMemoryInfo
    PROCESS_MEMORY_COUNTERS pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
    SIZE_T virtualMemUsedByMe = pmc.PeakWorkingSetSize;
    vmem.push_back(virtualMemUsedByMe);
    if (virtualMemUsedByMe > vmem_max)
    {
      vmem_max = virtualMemUsedByMe;
    }

    // Sleep for 5 second
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  // Print usage to file
  m_file << "max_virtual_memory_b=" << vmem_max << std::endl;
  m_file << "max_virtual_memory_mb=" << vmem_max / ((size_t)1024 * 1024) << std::endl;
  size_t avg = std::accumulate(vmem.begin(), vmem.end(), (size_t)0) / vmem.size();
  m_file << "avg_virtual_memory_b=" << avg << std::endl;
  m_file << "avg_virtual_memory_mb=" << avg / ((size_t)1024 * 1024) << std::endl;
}
