/* Author: Masaki Murooka */

#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>

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
  void startPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose_msg);

  /** \brief Callback for goal pose. */
  void goalPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose_msg);

protected:
  //! Footstep planner
  std::shared_ptr<FootstepPlanner> planner_;

  //! Start pose
  std::array<double, 3> start_pose_ = {0.0, 0.0, 0.0};

  //! Goal pose
  std::array<double, 3> goal_pose_ = {0.0, 0.0, 0.0};

  //! Accumulated duration for planning [sec]
  double accumulated_duration_ = 0.0;

  //! ROS node handle
  std::shared_ptr<rclcpp::Node> nh_ = rclcpp::Node::make_shared("footstep_planner");

  //! Publisher of footstep sequence
  rclcpp::Publisher<baseline_footstep_planner::msg::FootstepSequence2DStamped>::SharedPtr footstep_seq_pub_;

  //! Publisher of expanded states
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr expanded_states_pub_;

  //! Subscriber of start pose
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr start_pose_sub_;

  //! Subscriber of goal pose
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_pose_sub_;
};
} // namespace BFP
