#include "stretch_utils.h"

dynamic_programming::unit3 dynamic_programming::choose_stretch_factor(const StateSpace& state_space, const StateSpace& goal_space, const float x0[6], const std::string& state)
{
  // Exclude some states directly
  // TODO: use state more
  if (state.compare("Cruising") != 0 || true)
    return unit3::ONE();

  // If state space is small anyway, return 1
  size_t lengths[3]
  {
    state_space.get_range(0).length(),
    state_space.get_range(1).length(),
    state_space.get_range(2).length()
  };
  if (lengths[0] <= 15 && lengths[1] <= 15 && lengths[2] <= 15)
    return unit3::ONE();

  unit3 factor = unit3::ONE();
  validation_result result = validate_stretch_factor(state_space, goal_space, x0, factor);
  if (result != valid)
  {
    BOOST_LOG_TRIVIAL(error) << "Validating stretch factor of 1 failed (" << result << ")!";
    return factor;
  }
  for (int i = 0; i < 3; i++)
  {
    result = undefined;
    for (int s = 10; s > 0; s--)
    {
      factor[i] = (unit)s;
      result = validate_stretch_factor(state_space, goal_space, x0, factor);
      if (result == valid)
        break;
    }
    if (result != valid)
      factor[i] = 1;
  }
  return factor;
}

dynamic_programming::validation_result dynamic_programming::validate_stretch_factor(const StateSpace& state_space, const StateSpace& goal_space, const float x0[6], const unit3& stretch_factor, bool log)
{
  validation_result result = valid;

  // Validate state space
  for (int i = 0; i < 6; i++)
  {
    //// begin
    //long remainder = (long)state_space.begin[i] % (long)stretch_factor[i % 3];
    //if (remainder != 0)
    //{
    //  if (log)
    //    BOOST_LOG_TRIVIAL(info) << "The stretch factor resulted in a remainder unequal to 0. state space begin at " << i << " is " << remainder;
    //  result = unequal_remainder;
    //}

    //// end
    //remainder = (long)state_space.end[i] % (long)stretch_factor[i % 3];
    //if (remainder != 0)
    //{
    //  if (log)
    //    BOOST_LOG_TRIVIAL(info) << "The stretch factor resulted in a remainder unequal to 0. state space end at " << i << " is " << remainder;
    //  result = unequal_remainder;
    //}

    // length
    if (state_space.end[i] / stretch_factor[i % 3] - state_space.begin[i] / stretch_factor[i % 3] < 6)
    {
      if (log)
        BOOST_LOG_TRIVIAL(warning) << "The stretch factor resulted in a small state space (length <= 6) in dimension number " << i << ". Dynamic programming may fail.";
      if (result == valid)
        result = small_space;
    }
  }
  
  // Validate goal space
  for (int i = 0; i < 6; i++)
  {
    //// begin
    //long remainder = (long)goal_space.begin[i] % (long)stretch_factor[i % 3];
    //if (remainder != 0)
    //{
    //  if (log)
    //    BOOST_LOG_TRIVIAL(info) << "The stretch factor resulted in a remainder unequal to 0. goal space begin at " << i << " is " << remainder;
    //  result = unequal_remainder;
    //}

    //// end
    //remainder = (long)goal_space.end[i] % (long)stretch_factor[i % 3];
    //if (remainder != 0)
    //{
    //  if (log)
    //    BOOST_LOG_TRIVIAL(info) << "The stretch factor resulted in a remainder unequal to 0. goal space end at " << i << " is " << remainder;
    //  result = unequal_remainder;
    //}

    // length
    if (goal_space.end[i] / stretch_factor[i % 3] - goal_space.begin[i] / stretch_factor[i % 3] < 6)
    {
      if (log)
        BOOST_LOG_TRIVIAL(warning) << "The stretch factor resulted in a small goal space (length <= 6) in dimension number " << i << ". Dynamic programming may fail.";
      if (result == valid)
        result = small_space;
    }
  }

  //// Validate x0
  //for (int i = 0; i < 6; i++)
  //{
  //  long remainder = (long)x0[i] % (long)stretch_factor[i % 3];
  //  if (remainder != 0)
  //  {
  //    if (log)
  //      BOOST_LOG_TRIVIAL(info) << "The stretch factor resulted in a remainder unequal to 0. x0 at " << i << " is " << remainder;
  //    result = unequal_remainder;
  //  }
  //}

  // Max size
  for (int i = 0; i < 3; i++)
    if (stretch_factor[i] > 10)
    {
      result = too_big;
          if (log)
            BOOST_LOG_TRIVIAL(info) << "The stretch factor was too big: at " << i << " is " << stretch_factor[i];
    }

  return result;
}
