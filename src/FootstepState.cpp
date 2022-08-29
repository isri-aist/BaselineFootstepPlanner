/* Author: Masaki Murooka */

#include <iostream>

#include <ros/console.h>

#include <BaselineFootstepPlanner/FootstepState.h>

using namespace BFP;

void FootstepState::print() const
{
  ROS_INFO("FootstepState[id: %d, x: %d, y: %d, theta: %d, foot: %s]", id_, x_, y_, theta_,
           std::to_string(foot_).c_str());
}

double FootstepState::calcDistanceXy(const std::shared_ptr<FootstepState> & other) const
{
  double dx = static_cast<double>(x_ - other->x_);
  double dy = static_cast<double>(y_ - other->y_);
  return std::sqrt(dx * dx + dy * dy);
}

double FootstepState::calcDistanceTheta(const std::shared_ptr<FootstepState> & other, int theta_divide_num) const
{
  double dtheta = std::abs(static_cast<double>(theta_ - other->theta_));
  if(dtheta > theta_divide_num / 2.0)
  {
    dtheta = theta_divide_num - dtheta;
  }
  return dtheta;
}
