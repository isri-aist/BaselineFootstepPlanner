/* Author: Masaki Murooka */

#pragma once

#include <BaselineFootstepPlanner/FootstepEnv.h>

namespace BFP
{
/** \brief Configuration of environment for footstep planning set via ROS parameter. */
struct FootstepEnvConfigRos : public FootstepEnvConfig
{
  /** \brief Constructor. */
  FootstepEnvConfigRos();
};
} // namespace BFP
