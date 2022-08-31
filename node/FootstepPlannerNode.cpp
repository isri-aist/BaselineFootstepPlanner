/* Author: Masaki Murooka */

#include <chrono>

#include <sensor_msgs/PointCloud.h>
#include <baseline_footstep_planner/FootstepSequence2DStamped.h>

#include <BaselineFootstepPlanner/FootstepEnvConfigRos.h>
#include <BaselineFootstepPlanner/MathUtils.h>

#include "FootstepPlannerNode.h"

using namespace BFP;

FootstepPlannerNode::FootstepPlannerNode()
: planner_(std::make_shared<FootstepPlanner>(std::make_shared<FootstepEnvConfigRos>()))
{
  footstep_seq_pub_ = nh_.advertise<baseline_footstep_planner::FootstepSequence2DStamped>("footstep_sequence", 1, true);
  expanded_states_pub_ = nh_.advertise<sensor_msgs::PointCloud>("expanded_states", 1, true);
  start_pose_sub_ =
      nh_.subscribe<geometry_msgs::Pose2D>("start_pose", 1, &FootstepPlannerNode::startPoseCallback, this);
  goal_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("goal_pose", 1, &FootstepPlannerNode::goalPoseCallback, this);

  if(pnh_.hasParam("start_pose"))
  {
    std::vector<double> start_pose_vec(3);
    pnh_.getParam("start_pose", start_pose_vec);
    std::copy(start_pose_vec.begin(), start_pose_vec.end(), start_pose_.begin());
  }
  if(pnh_.hasParam("goal_pose"))
  {
    std::vector<double> goal_pose_vec(3);
    pnh_.getParam("goal_pose", goal_pose_vec);
    std::copy(goal_pose_vec.begin(), goal_pose_vec.end(), goal_pose_.begin());
  }

  setStartGoal();
}

void FootstepPlannerNode::run()
{
  constexpr double max_planning_duration = 0.01; // [sec]
  constexpr double initial_heuristics_weight = 10.0;

  ros::Rate rate(1.0 / max_planning_duration);
  while(ros::ok())
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

      ROS_INFO("[FootstepPlannerNode] new path found. time: %.2lf, cost: %d, eps: %.2lf, path len: %ld, states: %d.",
               accumulated_duration_, planner_->solution_.path_cost, planner_->solution_.heuristics_weight,
               planner_->solution_.id_list.size(), planner_->env_->stateNum());
    }
    publishExpandedStates();

    ros::spinOnce();
  }
}

void FootstepPlannerNode::setStartGoal()
{
  planner_->setStartGoal(planner_->env_->makeStateFromMidpose(start_pose_, Foot::LEFT),
                         planner_->env_->makeStateFromMidpose(start_pose_, Foot::RIGHT),
                         planner_->env_->makeStateFromMidpose(goal_pose_, Foot::LEFT),
                         planner_->env_->makeStateFromMidpose(goal_pose_, Foot::RIGHT));

  accumulated_duration_ = 0.0;

  publishEmptyPath();
}

void FootstepPlannerNode::publishFootstepSeq()
{
  if(footstep_seq_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  baseline_footstep_planner::FootstepSequence2DStamped footstep_seq_msg;
  footstep_seq_msg.header.stamp = ros::Time::now();
  footstep_seq_msg.sequence.footsteps.resize(planner_->solution_.id_list.size());
  for(unsigned int i = 0; i < planner_->solution_.id_list.size(); i++)
  {
    const std::shared_ptr<FootstepState> & state = planner_->env_->getStateFromId(planner_->solution_.id_list[i]);
    baseline_footstep_planner::Footstep2D & footstep_msg = footstep_seq_msg.sequence.footsteps[i];
    footstep_msg.foot_lr = static_cast<int>(state->foot_);
    footstep_msg.foot_pose.x = planner_->env_->discToContXy(state->x_);
    footstep_msg.foot_pose.y = planner_->env_->discToContXy(state->y_);
    footstep_msg.foot_pose.theta = planner_->env_->discToContTheta(state->theta_);
  }

  footstep_seq_pub_.publish(footstep_seq_msg);
}

void FootstepPlannerNode::publishExpandedStates()
{
  if(expanded_states_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  sensor_msgs::PointCloud cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  constexpr int expanded_states_pub_interval = 1000;
  for(int i = 0; i < planner_->env_->stateNum(); i += expanded_states_pub_interval)
  {
    const std::shared_ptr<FootstepState> & state = planner_->env_->getStateFromId(i);
    geometry_msgs::Point32 point;
    point.x = planner_->env_->discToContXy(state->x_);
    point.y = planner_->env_->discToContXy(state->y_);
    point.z = -0.1;
    cloud_msg.points.push_back(point);
  }

  expanded_states_pub_.publish(cloud_msg);
}

void FootstepPlannerNode::publishEmptyPath()
{
  if(footstep_seq_pub_.getNumSubscribers() > 0)
  {
    baseline_footstep_planner::FootstepSequence2DStamped footstep_seq_msg;
    footstep_seq_msg.header.stamp = ros::Time::now();
    footstep_seq_pub_.publish(footstep_seq_msg);
  }

  if(expanded_states_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    expanded_states_pub_.publish(cloud_msg);
  }
}

void FootstepPlannerNode::startPoseCallback(const geometry_msgs::Pose2D::ConstPtr & pose_msg)
{
  std::array<double, 3> new_start_pose = {pose_msg->x, pose_msg->y, radMod(pose_msg->theta)};
  if(start_pose_ != new_start_pose)
  {
    start_pose_ = new_start_pose;
    setStartGoal();
  }
}

void FootstepPlannerNode::goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr & pose_msg)
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
  ros::init(argc, argv, "footstep_planner");

  FootstepPlannerNode footstep_planner;

  footstep_planner.run();

  return 0;
}
