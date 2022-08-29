/* Author: Masaki Murooka */

#pragma once

#include <cmath>

#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

#include <BaselineFootstepPlanner/FootstepPlanner.h>

namespace BFP
{
/** \brief ROS node for footstep planner. */
class FootstepPlannerNode
{
public:
  /** \brief Constructor. */
  FootstepPlannerNode();

  /** \brief Run. */
  void run();

protected:
  /** \brief Set start and goal. */
  void setStartGoal();

  /** \brief Publish footstep sequence. */
  void publishFootstepSeq();

  /** \brief Publish expanded states. */
  void publishExpandedStates();

  /** \brief Publish empty path. */
  void publishEmptyPath();

  /** \brief Callback for start pose. */
  void startPoseCallback(const geometry_msgs::Pose2D::ConstPtr & pose_msg);

  /** \brief Callback for goal pose. */
  void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr & pose_msg);

protected:
  //! Footstep planner
  std::shared_ptr<FootstepPlanner> planner_;

  //! Start pose
  std::array<double, 3> start_pose_ = {std::nan(""), std::nan(""), std::nan("")};

  //! Goal pose
  std::array<double, 3> goal_pose_ = {std::nan(""), std::nan(""), std::nan("")};

  //! Accumulated duration for planning [sec]
  double accumulated_duration_ = 0.0;

  //! ROS node handle
  ros::NodeHandle nh_;

  //! Publisher of footstep sequence
  ros::Publisher footstep_seq_pub_;

  //! Publisher of expanded states
  ros::Publisher expanded_states_pub_;

  //! Subscriber of start pose
  ros::Subscriber start_pose_sub_;

  //! Subscriber of goal pose
  ros::Subscriber goal_pose_sub_;
};
} // namespace BFP
