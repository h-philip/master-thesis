#include "config.h"

void dynamic_programming::Config::load_from_file(const std::string& file)
{
  std::ifstream in(file);
  std::string line;
  while (std::getline(in, line))
  {
    // Skip empty lines and comments
    if (line.empty() || line.at(0) == ' ' || line.at(0) == '#')
      continue;

    // Split line by '='
    std::string delimiter = "=";
    size_t pos = 0;
    std::string token;
    std::string key;
    std::string value;
    while ((pos = line.find(delimiter)) != std::string::npos)
    {
      token = line.substr(0, pos);
      line.erase(0, pos + delimiter.length());
      key = token;
      value = line;
    }

    // Set key-value pair
    for (auto& pair : m_key_names)
    {
      if (pair.second == key)
      {
        m_key_value[pair.first] = value;
      }
    }
  }
}

void dynamic_programming::Config::parse_args(int argc, char* argv[])
{
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    // Print arg
    BOOST_LOG_TRIVIAL(debug) << "Arg: " << arg;
    // Check if argument is the config file
    std::string key_arg = "--config_file=";
    if (arg.starts_with(key_arg))
    {
      load_from_file(arg.substr(key_arg.size()));
      continue;
    }

    for (auto& pair : m_key_names)
    {
      std::string key = pair.second;
      std::string key_arg = "--" + key + "=";
      if (arg.starts_with(key_arg))
      {
        set(pair.first, arg.substr(key_arg.size()));
        continue;
      }
    }
  }
}

std::string dynamic_programming::Config::get(const Key& key) const
{
  // Check if key is set
  if (m_key_value.find(key) == m_key_value.end())
  {
    // Check if default value is set
    if (m_default_values.find(key) != m_default_values.end())
    {
      return m_default_values.at(key);
    }
    return "";
  }
  return m_key_value.at(key);
}

void dynamic_programming::Config::set(const Key& key, const std::string& value)
{
  m_key_value[key] = value;
}

bool dynamic_programming::Config::is_set(const Key& key) const
{
  return m_key_value.find(key) != m_key_value.end();
}

void dynamic_programming::Config::save_to_file(const std::string& file)
{
  std::ofstream out(file);
  for (auto& pair : m_key_names)
  {
    out << pair.second << "=" << get(pair.first) << std::endl;
  }
  out.close();
}

bool dynamic_programming::Config::validate_data()
{
  // Check if all required keys are set
  // ROUTE_FILE and COLLISION_CLOUD_FILE are required
  if (!is_set(Key::ROUTE_FILE) || !is_set(Key::COLLISION_CLOUD_FILE))
  {
    BOOST_LOG_TRIVIAL(error) << "Route file or collision cloud file not set";
    return false;
  }
  // Check if files exist
  if (!std::filesystem::exists(get(Key::ROUTE_FILE)) || !std::filesystem::exists(get(Key::COLLISION_CLOUD_FILE)))
  {
    BOOST_LOG_TRIVIAL(error) << "Route file or collision cloud file does not exist";
    return false;
  }
  // Check if files are regular files
  if (!std::filesystem::is_regular_file(get(Key::ROUTE_FILE)) || !std::filesystem::is_regular_file(get(Key::COLLISION_CLOUD_FILE)))
  {
    BOOST_LOG_TRIVIAL(error) << "Route file or collision cloud file is not a regular file";
    return false;
  }
  
  // Check other keys

  // Check if NUMBER_OF_STAGES is an int
  if (!is_int(get(Key::NUMBER_OF_STAGES), "NUMBER_OF_STAGES"))
  {
    return false;
  }

  // Check if COLLISION_COST_FACTOR is a float
  if (!is_float(get(Key::COLLISION_COST_FACTOR), "COLLISION_COST_FACTOR"))
  {
    return false;
  }

  // Check if DISTURBANCE_ON is a bool
  if (get(Key::DISTURBANCE_ON) != "true" && get(Key::DISTURBANCE_ON) != "false")
  {
    BOOST_LOG_TRIVIAL(error) << "DISTURBANCE_ON is not a bool";
    return false;
  }

  // Check if DISTURBANCE_CHANGE_FACTOR is an int
  if (!is_int(get(Key::DISTURBANCE_CHANGE_FACTOR), "DISTURBANCE_CHANGE_FACTOR"))
  {
    return false;
  }

  return true;
}

bool dynamic_programming::Config::is_int(const std::string& s, const std::string& key)
{
  try
  {
    (void)std::stoi(s);
  }
  catch (const std::out_of_range& e)
  {
    BOOST_LOG_TRIVIAL(error) << key << " is out of range";
    return false;
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_TRIVIAL(error) << key << " is not an int";
    return false;
  }
}

bool dynamic_programming::Config::is_float(const std::string& s, const std::string& key)
{
  try
  {
    (void)std::stof(s);
  }
  catch (const std::out_of_range& e)
  {
    BOOST_LOG_TRIVIAL(error) << key << " is out of range";
    return false;
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_TRIVIAL(error) << key << " is not a float";
    return false;
  }
}
