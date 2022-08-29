/* Author: Masaki Murooka */

#pragma once

#include <memory>

#include <BaselineFootstepPlanner/FootstepTypes.h>

namespace BFP
{
/** \brief Footstep state. */
class FootstepState
{
public:
  /** \brief Constructor.
      \param x discretized X position
      \param y discretized Y position
      \param theta discretized orientation
      \param foot foot
  */
  FootstepState(int x, int y, int theta, Foot foot) : x_(x), y_(y), theta_(theta), foot_(foot) {}

  /** \brief Print state. */
  void print() const;

  /** \brief Calculate XY distance to other state.
      \param other other state
  */
  double calcDistanceXy(const std::shared_ptr<FootstepState> & other) const;

  /** \brief Calculate orientation distance to other state.
      \param other other state
      \param theta_divide_num division number to discretize orientation
  */
  double calcDistanceTheta(const std::shared_ptr<FootstepState> & other, int theta_divide_num) const;

public:
  //! Discretized X position
  int x_;

  //! Discretized Y position
  int y_;

  //! Discretized orientation
  int theta_;

  //! Foot
  Foot foot_;

  //! State id
  int id_ = -1;
};
} // namespace BFP

// necessary for std::unordered_map
// ref. https://qiita.com/izmktr/items/8e0fd1b6e37de59a9bd0
// ref. https://odan3240.hatenablog.com/entry/2016/03/31/223906
namespace std
{
template<>
struct equal_to<std::shared_ptr<BFP::FootstepState>>
{
  bool operator()(const std::shared_ptr<BFP::FootstepState> & lhs,
                  const std::shared_ptr<BFP::FootstepState> & rhs) const
  {
    return lhs->x_ == rhs->x_ && lhs->y_ == rhs->y_ && lhs->theta_ == rhs->theta_ && lhs->foot_ == rhs->foot_;
  }
};

template<>
struct hash<std::shared_ptr<BFP::FootstepState>>
{
  size_t operator()(const std::shared_ptr<BFP::FootstepState> & state) const
  {
    size_t seed = 0;
    seed ^= hash<int>()(state->x_) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash<int>()(state->y_) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash<int>()(state->theta_) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash<int>()(static_cast<int>(state->foot_)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
} // namespace std
