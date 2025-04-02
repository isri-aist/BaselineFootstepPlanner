/* Author: Masaki Murooka */

#include <rclcpp/rclcpp.hpp>

#include <BaselineFootstepPlanner/FootstepEnvConfigRos.h>

using namespace BFP;

FootstepEnvConfigRos::FootstepEnvConfigRos()
{
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("footstep_planner_env_config_ros");

  nh->declare_parameter<int>("theta_divide_num", theta_divide_num);
  nh->declare_parameter<double>("xy_divide_step", xy_divide_step);

  nh->declare_parameter<double>("cost_scale", cost_scale);
  nh->declare_parameter<double>("cost_theta_scale", cost_theta_scale);
  nh->declare_parameter<double>("step_cost", step_cost);

  nh->declare_parameter<std::string>("heuristic_type", "default");
  nh->declare_parameter<std::vector<double>>("r2l_action_cont_list", std::vector<double>(18, 0.0));
  nh->declare_parameter<std::vector<double>>("r2l_reachable_min", std::vector<double>(3, 0.0));
  nh->declare_parameter<std::vector<double>>("r2l_reachable_max", std::vector<double>(3, 0.0));

  std::vector<double> rect_obst_list_param;
  nh->declare_parameter<std::vector<double>>("rect_obstacle_list", std::vector<double>(8, 0.0));

  if(nh->has_parameter("heuristic_type"))
  {
    std::string heuristic_type_str;
    nh->get_parameter("heuristic_type", heuristic_type_str);
    heuristic_type = strToHeuristicType(heuristic_type_str);
  }
  nh->declare_parameter<double>("dijkstra_path_heuristic_expand_scale", dijkstra_path_heuristic_expand_scale);

  std::ostringstream oss_heuristic_type;
  oss_heuristic_type << "[FootstepEnvConfigRos] Heuristic type: " << std::to_string(heuristic_type);

  nh->declare_parameter<double>("nominal_foot_separation", nominal_foot_separation);

  if(nh->has_parameter("r2l_action_cont_list"))
  {
    r2l_action_cont_list.clear();
    std::vector<double> r2l_action_cont_list_param;
    nh->get_parameter("r2l_action_cont_list", r2l_action_cont_list_param);

    if(r2l_action_cont_list_param.size() % 3 != 0)
    {
      throw std::invalid_argument("Invalid r2l_action_cont_list. Size of each element should be 3.");
    }

    r2l_action_cont_list.reserve(r2l_action_cont_list_param.size() / 3);

    for(size_t i = 0; i < r2l_action_cont_list_param.size(); i += 3)
    {
      r2l_action_cont_list.emplace_back(r2l_action_cont_list_param[i], r2l_action_cont_list_param[i + 1],
                                        r2l_action_cont_list_param[i + 2]);
    }
  }
  std::ostringstream number_of_action;

  if(nh->has_parameter("r2l_reachable_min"))
  {
    std::vector<double> r2l_reachable_min_param;
    nh->get_parameter("r2l_reachable_min", r2l_reachable_min_param);
    if(r2l_reachable_min_param.size() != 3)
    {
      throw std::invalid_argument("Invalid r2l_reachable_min. Size of element should be 3.");
    }
    r2l_reachable_min =
        FootstepActionCont(r2l_reachable_min_param[0], r2l_reachable_min_param[1], r2l_reachable_min_param[2]);
  }
  if(nh->has_parameter("r2l_reachable_max"))
  {
    std::vector<double> r2l_reachable_max_param;
    nh->get_parameter("r2l_reachable_max", r2l_reachable_max_param);
    if(r2l_reachable_max_param.size() != 3)
    {
      throw std::invalid_argument("Invalid r2l_reachable_max. Size of element should be 3.");
    }
    r2l_reachable_max =
        FootstepActionCont(r2l_reachable_max_param[0], r2l_reachable_max_param[1], r2l_reachable_max_param[2]);
  }

  if(nh->has_parameter("rect_obstacle_list"))
  {
    rect_obst_list.clear();
    nh->get_parameter("rect_obstacle_list", rect_obst_list_param);

    if(rect_obst_list_param.size() % 4 != 0)
    {
      RCLCPP_ERROR(nh->get_logger(), "Invalid rect_obstacle_list. Size of each element should be 4.");
      return;
    }

    rect_obst_list.reserve(rect_obst_list_param.size() / 4);

    for(size_t i = 0; i < rect_obst_list_param.size(); i += 4)
    {
      rect_obst_list.emplace_back(rect_obst_list_param[i], rect_obst_list_param[i + 1], rect_obst_list_param[i + 2],
                                  rect_obst_list_param[i + 3]);
    }
  }
}
