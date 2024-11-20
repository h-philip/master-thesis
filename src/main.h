#pragma once

#include "drone_logger.h"
#include "drone_plotter.h"
#include "dynamic_programming.h"
#include "hybrid_automaton.h"
#include "range.h"
#include "config.h"
#include "dp_stats.h"
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/expressions.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <ctime>
#include <filesystem>
#include <iterator>
#include <vector>

namespace dynamic_programming
{
  BOOL WINAPI CtrlHandler(DWORD fdwCtrlType);

  std::vector<unit3> get_route(const std::string path);

  std::string get_output_directory(const std::string& description);

  void setup_logging(const std::string& directory);
}