#include "consts.h"

void dynamic_programming::debug_parameters()
{
  BOOST_LOG_TRIVIAL(debug) << "parameters: DELTA_TIME = " << DELTA_TIME << "; STEP_SIZE = " << STEP_SIZE;
}

dynamic_programming::unit3 dynamic_programming::INPUTS_FINE[NUM_INPUTS] = {};
dynamic_programming::unit3 dynamic_programming::INPUTS_COARSE[NUM_INPUTS] = {};

void dynamic_programming::create_inputs(unit3* dest, unit3 max)
{
  size_t counter = 0;
  for (unit x = -max.x; x <= max.x; x += max.x)
    for (unit y = -max.y; y <= max.y; y += max.y)
      for (unit z = -max.z; z <= max.z; z += max.z)
        dest[counter++] = { x, y, z };
}

void dynamic_programming::fill_input_arrays()
{
  create_inputs(INPUTS_FINE, { 2, 2, 2 });
  create_inputs(INPUTS_COARSE, { 4, 4, 4 });
}

int dynamic_programming::search_disturbances(const unit3& value)
{
  double min = std::numeric_limits<double>::max();
  int argmin = -1;
  for (int i = 0; i < NUM_DISTURBANCES; i++)
  {
    const unit3& entry = DISTURBANCES[i];
    double distance2 = pow(entry.x - value.x, 2) + pow(entry.y - value.y, 2) + pow(entry.z - value.z, 2);
    if (distance2 < min)
    {
      min = distance2;
      argmin = i;
    }
  }
  return argmin;
}
