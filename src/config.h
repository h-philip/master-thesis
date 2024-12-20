#pragma once
#include <string>
#include <fstream>
#include <map>
#include <map>
#include <boost/log/trivial.hpp>
#include <filesystem>

namespace dynamic_programming
{
  // Singleton
  class Config
  {
  public:
    static Config& get_instance()
    {
      static Config instance;
      return instance;
    }

    // Delete copy constructor and assignment operator
    Config(Config const&) = delete;
    void operator=(Config const&) = delete;

    /// <summary>
    /// Enum of keys
    /// </summary>
    enum Key
    {
      DESCRIPTION,
      COLLISION_CLOUD_FILE,
      ROUTE_FILE,
      NUMBER_OF_STAGES,
      COLLISION_COST_FACTOR,
      DISTURBANCE_ON,
      APPLY_DISTURBANCE,
      DISTURBANCE_CHANGE_FACTOR,
      ENABLE_NORM_FIX_POINT,
      ENABLE_INITIAL_FIX_POINT,
      USE_SINGLE_STAGE_CONTROLLER
    };

    void load_from_file(const std::string& file);

    void parse_args(int argc, char* argv[]);

    std::string get(const Key& key) const;
    void set(const Key& key, const std::string& value);

    // Generic get for different return types
    template <typename T>
    T get(const Key& key) const;

    // Specialization for int
    template <>
    int get<int>(const Key& key) const
    {
      return std::stoi(get(key));
    }

    // Specialization for float
    template <>
    float get<float>(const Key& key) const
    {
      return std::stof(get(key));
    }

    // Specialization for bool
    template <>
    bool get<bool>(const Key& key) const
    {
      return get(key) == "true";
    }

    // Specialization for string
    template <>
    std::string get<std::string>(const Key& key) const
    {
      return get(key);
    }

    bool is_set(const Key& key) const;

    void save_to_file(const std::string& file);

    bool validate_data();

  private:
    Config()
    {
      // Initialize key names and default values if needed

      m_key_names[DESCRIPTION] = "description";

      m_key_names[COLLISION_CLOUD_FILE] = "collision_cloud_file";

      m_key_names[ROUTE_FILE] = "route_file";

      m_key_names[NUMBER_OF_STAGES] = "number_of_stages";
      m_default_values[NUMBER_OF_STAGES] = "30";

      m_key_names[COLLISION_COST_FACTOR] = "collision_cost_factor";
      m_default_values[COLLISION_COST_FACTOR] = "0.0";

      m_key_names[DISTURBANCE_ON] = "disturbance_on";
      m_default_values[DISTURBANCE_ON] = "true";

      m_key_names[APPLY_DISTURBANCE] = "apply_disturbance";
      m_default_values[APPLY_DISTURBANCE] = "true";

      m_key_names[DISTURBANCE_CHANGE_FACTOR] = "disturbance_change_factor";
      m_default_values[DISTURBANCE_CHANGE_FACTOR] = "10";

      m_key_names[ENABLE_NORM_FIX_POINT] = "enable_norm_fix_point";
      m_default_values[ENABLE_NORM_FIX_POINT] = "false";

      m_key_names[ENABLE_INITIAL_FIX_POINT] = "enable_initial_fix_point";
      m_default_values[ENABLE_INITIAL_FIX_POINT] = "false";

      m_key_names[USE_SINGLE_STAGE_CONTROLLER] = "use_single_stage_controller";
      m_default_values[USE_SINGLE_STAGE_CONTROLLER] = "false";
    }

    bool is_int(const std::string& s, const std::string& key);
    bool is_float(const std::string& s, const std::string& key);

    /// <summary>
    /// Map of key names
    /// </summary>
    std::map<Key, std::string> m_key_names;

    /// <summary>
    /// Map of key value pairs
    /// </summary>
    std::map<Key, std::string> m_key_value;

    /// <summary>
    /// Default values
    /// </summary>
    std::map<Key, std::string> m_default_values;
  };
}