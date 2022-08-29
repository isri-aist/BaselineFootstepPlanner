/* Author: Masaki Murooka */

#pragma once

#include <cmath>

namespace BFP
{
/** \brief Array index. */
enum Index
{
  //! X
  X = 0,

  //! Y
  Y
};

/** \brief Convert angle from degree to radian.
    \param deg angle [deg]
*/
inline constexpr double degToRad(double deg)
{
  return deg * M_PI / 180.0;
}

/** \brief Calculate mod of angle.
    \param angle angle [rad]
*/
inline double radMod(double angle)
{
  return angle - 2.0 * M_PI * std::floor(angle / (2.0 * M_PI));
}
} // namespace BFP
