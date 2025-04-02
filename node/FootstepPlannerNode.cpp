/* Author: Masaki Murooka */

#include <chrono>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <baseline_footstep_planner/msg/footstep_sequence2_d_stamped.hpp>

#include <BaselineFootstepPlanner/FootstepEnvConfigRos.h>

#include "FootstepPlannerNode.h"

using namespace BFP;

FootstepPlannerNode::FootstepPlannerNode()
{
  planner_ = std::make_shared<FootstepPlanner>(std::make_shared<FootstepEnvConfigRos>());

  footstep_seq_pub_ =
      nh_->create_publisher<baseline_footstep_planner::msg::FootstepSequence2DStamped>("footstep_sequence", 1);
  expanded_states_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud>("expanded_states", 1);
  start_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::Pose2D>(
      "start_pose", 1, std::bind(&FootstepPlannerNode::startPoseCallback, this, std::placeholders::_1));
  goal_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::Pose2D>(
      "goal_pose", 1, std::bind(&FootstepPlannerNode::goalPoseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(nh_->get_logger(), "[FootstepPlannerNode] Debug");
  nh_->declare_parameter<std::vector<double>>("start_pose", {0.0, 0.0, 0.0});
  nh_->declare_parameter<std::vector<double>>("goal_pose", {2.0, 1.5, 0.0});

  if(nh_->has_parameter("start_pose"))
  {
    std::vector<double> start_pose_vec(3);
    nh_->get_parameter("start_pose", start_pose_vec);
    RCLCPP_INFO(nh_->get_logger(), "[FootstepPlannerNode] Start pose: [%f, %f, %f]",
                start_pose_vec[0], start_pose_vec[1], start_pose_vec[2]);
    std::copy(start_pose_vec.begin(), start_pose_vec.end(), start_pose_.begin());
  }
  if(nh_->has_parameter("goal_pose"))
  {
    std::vector<double> goal_pose_vec(3);
    nh_->get_parameter("goal_pose", goal_pose_vec);
    std::copy(goal_pose_vec.begin(), goal_pose_vec.end(), goal_pose_.begin());
  }

  setStartGoal();
}

void FootstepPlannerNode::run()
{
  constexpr double max_planning_duration = 0.01; // [sec]
  constexpr double initial_heuristics_weight = 10.0;

  rclcpp::Rate rate(1.0 / max_planning_duration);
  while(rclcpp::ok())
  {
    // backup previous solution
    std::vector<int> prev_solution_id_list = planner_->solution_.id_list;

    // replan
    auto start_time = std::chrono::system_clock::now();
    planner_->run(false, max_planning_duration, initial_heuristics_weight);
    double plan_duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start_time).count();
    accumulated_duration_ += plan_duration;

    // when new solution found
    if(planner_->solution_.is_solved && planner_->solution_.id_list != prev_solution_id_list)
    {
      publishFootstepSeq();

      RCLCPP_INFO(
          nh_->get_logger(),
          "[FootstepPlannerNode] New path found. time: %.2lf, cost: %d, eps: %.2lf, footsteps num: %ld, expanded "
          "states: %d.",
          accumulated_duration_, planner_->solution_.path_cost, planner_->solution_.heuristics_weight,
          planner_->solution_.id_list.size(), planner_->env_->stateNum());
    }
    publishExpandedStates();

    rclcpp::spin_some(nh_);
    rate.sleep();
  }
}

void FootstepPlannerNode::setStartGoal()
{
  RCLCPP_INFO(nh_->get_logger(), "[FootstepPlannerNode] Update start and goal.");

  publishEmptyPath();

  planner_->setStartGoal(planner_->env_->makeStateFromMidpose(start_pose_, Foot::LEFT),
                         planner_->env_->makeStateFromMidpose(start_pose_, Foot::RIGHT),
                         planner_->env_->makeStateFromMidpose(goal_pose_, Foot::LEFT),
                         planner_->env_->makeStateFromMidpose(goal_pose_, Foot::RIGHT));

  accumulated_duration_ = 0.0;
}

void FootstepPlannerNode::publishFootstepSeq()
{
  if(footstep_seq_pub_->get_subscription_count() == 0)
  {
    return;
  }

  baseline_footstep_planner::msg::FootstepSequence2DStamped footstep_seq_msg;
  rclcpp::Clock clock;
  rclcpp::Time time_now = clock.now();
  footstep_seq_msg.header.stamp = time_now;
  footstep_seq_msg.sequence.footsteps.resize(planner_->solution_.id_list.size());
  for(unsigned int i = 0; i < planner_->solution_.id_list.size(); i++)
  {
    const std::shared_ptr<FootstepState> & state = planner_->env_->getStateFromId(planner_->solution_.id_list[i]);
    baseline_footstep_planner::msg::Footstep2D & footstep_msg = footstep_seq_msg.sequence.footsteps[i];
    footstep_msg.foot_lr = static_cast<char>(state->foot_);
    footstep_msg.foot_pose.x = planner_->env_->discToContXy(state->x_);
    footstep_msg.foot_pose.y = planner_->env_->discToContXy(state->y_);
    footstep_msg.foot_pose.theta = planner_->env_->discToContTheta(state->theta_);
  }

  footstep_seq_pub_->publish(footstep_seq_msg);
}

void FootstepPlannerNode::publishExpandedStates()
{
  if(expanded_states_pub_->get_subscription_count() == 0)
  {
    return;
  }

  sensor_msgs::msg::PointCloud cloud_msg;
  rclcpp::Clock clock;
  rclcpp::Time time_now = clock.now();
  cloud_msg.header.stamp = time_now;
  cloud_msg.header.frame_id = "world";
  constexpr int expanded_states_pub_interval = 1000;
  for(int i = 0; i < planner_->env_->stateNum(); i += expanded_states_pub_interval)
  {
    const std::shared_ptr<FootstepState> & state = planner_->env_->getStateFromId(i);
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(planner_->env_->discToContXy(state->x_));
    point.y = static_cast<float>(planner_->env_->discToContXy(state->y_));
    point.z = -0.1f;
    cloud_msg.points.push_back(point);
  }

  expanded_states_pub_->publish(cloud_msg);
}

void FootstepPlannerNode::publishEmptyPath()
{
  if(footstep_seq_pub_->get_subscription_count() > 0)
  {
    baseline_footstep_planner::msg::FootstepSequence2DStamped footstep_seq_msg;
    rclcpp::Clock clock;
    rclcpp::Time time_now = clock.now();
    footstep_seq_msg.header.stamp = time_now;
    footstep_seq_pub_->publish(footstep_seq_msg);
  }

  if(expanded_states_pub_->get_subscription_count() > 0)
  {
    sensor_msgs::msg::PointCloud cloud_msg;
    rclcpp::Clock clock;
    rclcpp::Time time_now = clock.now();
    cloud_msg.header.stamp = time_now;
    cloud_msg.header.frame_id = "world";
    expanded_states_pub_->publish(cloud_msg);
  }
}

void FootstepPlannerNode::startPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose_msg)
{
  std::array<double, 3> new_start_pose = {pose_msg->x, pose_msg->y, radMod(pose_msg->theta)};
  if(start_pose_ != new_start_pose)
  {
    start_pose_ = new_start_pose;
    setStartGoal();
  }
}

void FootstepPlannerNode::goalPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose_msg)
{
  std::array<double, 3> new_goal_pose = {pose_msg->x, pose_msg->y, radMod(pose_msg->theta)};
  if(goal_pose_ != new_goal_pose)
  {
    goal_pose_ = new_goal_pose;
    setStartGoal();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  FootstepPlannerNode footstep_planner;

  footstep_planner.run();

  return 0;
}
