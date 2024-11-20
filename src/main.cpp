#include "main.h"

using namespace std;
using namespace dynamic_programming;

int main(int argc, char* argv[])
{
  fill_input_arrays();
  debug_parameters();

  // Load config
  Config& config = Config::get_instance();
  config.parse_args(argc, argv);
  if (!config.validate_data())
  {
    return -1;
  }

  // Create output directory
  std::string out_dir = get_output_directory(config.get(Config::Key::DESCRIPTION));

  // Setup logging
  setup_logging(out_dir);

  // Save config to file
  config.save_to_file(out_dir + "config.txt");

  // Copy route and collision cloud to output directory
  std::filesystem::copy(config.get(Config::Key::ROUTE_FILE), out_dir + "route.txt");
  std::filesystem::copy(config.get(Config::Key::COLLISION_CLOUD_FILE), out_dir + "collision_cloud.txt");

  // Load route
  vector<unit3> route = get_route(config.get(Config::Key::ROUTE_FILE));


  // Run hybrid automaton
  try
  {
    DpStats dp_stats(out_dir);

    HybridAutomaton hybrid_automaton = HybridAutomaton(route, &dp_stats);
    DroneLogger logger(&hybrid_automaton);
    logger.log_to_file(out_dir + "log.txt");
    DronePlotter plotter(&hybrid_automaton, out_dir);
    hybrid_automaton.run_until_end();
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_TRIVIAL(fatal) << "Setting up hybrid automaton failed: " << e.what();
    return -1;
  }
  catch (...)
  {
    std::exception_ptr eptr = std::current_exception();
    BOOST_LOG_TRIVIAL(fatal) << "Unexpected error: " << boost::current_exception_diagnostic_information() << std::endl;
  }

  BOOST_LOG_TRIVIAL(info) << "Done.";
  return 0;
}

/// <summary>
/// Get route from file
/// </summary>
/// <param name="path">Path to file</param>
/// <returns>Route</returns>
std::vector<unit3> dynamic_programming::get_route(const std::string path)
{
  std::vector<unit3> route;

  if (true)
  {
    ifstream file(path);
    if (file.is_open())
    {
      unit3 point;
      string line;
      while (getline(file, line))
      {
        // Skip empty lines and comments
        if (line.empty() || line.at(0) == ' ' || line.at(0) == '#')
          continue;

        // Check for end of route
        if (line.compare("end") == 0)
          break;

        // Parse point
        string::size_type index = line.find(" ");
        point.x = (unit)stof(line.substr(0, index));
        line = line.substr(index + 1);
        index = line.find(" ");
        point.y = (unit)stof(line.substr(0, index));
        point.z = (unit)stof(line.substr(index + 1));

        // Add point to route
        route.push_back(point);
      }
    }
    else
    {
      string err = "Could not open file ";
      err.append(path);
      BOOST_LOG_TRIVIAL(error) << err;
      throw invalid_argument(err);
    }
  }
  else
  {
    // Old testing code
    route.push_back({ 0, 0, 0 });
    route.push_back({ 0, 0, 10 });
    route.push_back({ 0, 0, 20 });
    route.push_back({ 10, 0, 20 });
    route.push_back({ 10, 0, 10 });
    route.push_back({ 10, 0, 0 });
  }
  
  return route;
}

std::string dynamic_programming::get_output_directory(const std::string& description)
{
  // Create output directory name
  time_t now = time(NULL);
  constexpr size_t str_length = size("yyyy-mm-dd_hh-mm-ss");
  char time_str[str_length];
  tm lt;
  localtime_s(&lt, &now);
  strftime(data(time_str), size(time_str), "%F_%H-%M-%S", &lt);

  // Create output directory
  std::string out_dir = "./";
  out_dir.append(time_str, str_length - 1);
  if (!description.empty())
  {
    // Add description to output directory name
    out_dir.append(" (");
    out_dir.append(description);
    out_dir.append(")");
  }
  out_dir.append("/");
  std::filesystem::create_directories(out_dir);

  return out_dir;
}

void dynamic_programming::setup_logging(const std::string& directory)
{
  // Set boost trivial logging to file and console

  // Add severity to boost global logging namespace
  boost::log::register_simple_formatter_factory< boost::log::trivial::severity_level, char >("Severity");

  std::string format = "[%TimeStamp%] [%ThreadID%] [%Severity%] %Message%";

  boost::log::add_file_log(
    boost::log::keywords::file_name = directory + "run.log",
    boost::log::keywords::auto_flush = true,
    boost::log::keywords::format = format
  );
  boost::log::add_console_log(
    std::cout,
    boost::log::keywords::format = format
  );
  boost::log::add_common_attributes();
}
