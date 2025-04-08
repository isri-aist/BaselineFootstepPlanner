/* Author: Masaki Murooka */

#pragma once
#include <string>

#include <string>

namespace BFP
{
/** \brief Foot. */
enum class Foot
{
  //! Left foot
  LEFT = 0,

  //! Right foot
  RIGHT
};
} // namespace BFP

namespace std
{
using BFP::Foot;

/** \brief Convert foot to string.
    \param foot foot
*/
inline std::string to_string(Foot foot)
{
  if(foot == Foot::LEFT)
  {
    return std::string("left");
  }
  else // if(foot == Foot::RIGHT)
  {
    return std::string("right");
  }
}
} // namespace std

namespace BFP
{
/** \brief Get opposite foot.
    \param foot foot
*/
inline Foot opposite(Foot foot)
{
  if(foot == Foot::LEFT)
  {
    return Foot::RIGHT;
  }
  else // if(foot == Foot::RIGHT)
  {
    return Foot::LEFT;
  }
}

/** \brief Discretized footstep action. */
struct FootstepAction
{
  //! Discretized X position
  int x;

  //! Discretized Y position
  int y;

  //! Discretized orientation
  int theta;

  /** \brief Constructor.
      \param _x discretized X position
      \param _y discretized Y position
      \param _theta discretized orientation
  */
  FootstepAction(int _x, int _y, int _theta) : x(_x), y(_y), theta(_theta) {}
};

/** \brief Continuous footstep action. */
struct FootstepActionCont
{
  //! Continuous X position [m]
  double x;

  /** \brief Continuous Y position [m]

      \note Y position is the value subtracted by nominal distance between left and right feet.
  */
  double y;

  //! Continuous orientation [rad]
  double theta;

  /** \brief Constructor.
      \param _x continuous X position [m]
      \param _y continuous Y position [m]
      \param _theta continuous orientation [rad]

      \note Y position is the value subtracted by nominal distance between left and right feet.
  */
  FootstepActionCont(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
};
} // namespace BFP
