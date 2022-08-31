/* Author: Masaki Murooka */

#include <ros/ros.h>

#include <BaselineFootstepPlanner/FootstepEnvConfigRos.h>

using namespace BFP;

namespace
{
double getDoubleValueFromRosparam(const XmlRpc::XmlRpcValue & param)
{
  double value;
  if(param.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    value = static_cast<int>(param);
  }
  else if(param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    value = static_cast<double>(param);
  }
  else
  {
    throw std::invalid_argument("Param type is not scalar");
  }
  return value;
}
} // namespace

FootstepEnvConfigRos::FootstepEnvConfigRos()
{
  ros::NodeHandle nh;

  nh.param("theta_divide_num", theta_divide_num, theta_divide_num);
  nh.param("xy_divide_step", xy_divide_step, xy_divide_step);

  nh.param("cost_scale", cost_scale, cost_scale);
  nh.param("cost_theta_scale", cost_theta_scale, cost_theta_scale);
  nh.param("step_cost", step_cost, step_cost);

  if(nh.hasParam("heuristic_type"))
  {
    std::string heuristic_type_str;
    nh.getParam("heuristic_type", heuristic_type_str);
    heuristic_type = strToHeuristicType(heuristic_type_str);
  }
  nh.param("dijkstra_path_heuristic_expand_scale", dijkstra_path_heuristic_expand_scale,
           dijkstra_path_heuristic_expand_scale);
  ROS_DEBUG_STREAM("[FootstepEnvConfigRos] Heuristic type: " << std::to_string(heuristic_type));

  nh.param("nominal_foot_separation", nominal_foot_separation, nominal_foot_separation);

  if(nh.hasParam("r2l_action_cont_list"))
  {
    r2l_action_cont_list.clear();
    XmlRpc::XmlRpcValue r2l_action_cont_list_param;
    nh.getParam("r2l_action_cont_list", r2l_action_cont_list_param);
    for(int i = 0; i < r2l_action_cont_list_param.size(); i++)
    {
      const XmlRpc::XmlRpcValue & r2l_action_cont_param = r2l_action_cont_list_param[i];
      if(r2l_action_cont_param.size() != 3)
      {
        throw std::invalid_argument("Invalid r2l_action_cont_list. Size of each element should be 3.");
      }
      r2l_action_cont_list.emplace_back(getDoubleValueFromRosparam(r2l_action_cont_param[0]),
                                        getDoubleValueFromRosparam(r2l_action_cont_param[1]),
                                        getDoubleValueFromRosparam(r2l_action_cont_param[2]));
    }
  }
  ROS_INFO_STREAM("[FootstepEnvConfigRos] Number of actions: " << r2l_action_cont_list.size());

  if(nh.hasParam("r2l_reachable_min"))
  {
    XmlRpc::XmlRpcValue r2l_reachable_min_param;
    nh.getParam("r2l_reachable_min", r2l_reachable_min_param);
    if(r2l_reachable_min_param.size() != 3)
    {
      throw std::invalid_argument("Invalid r2l_reachable_min. Size of element should be 3.");
    }
    r2l_reachable_min = FootstepActionCont(getDoubleValueFromRosparam(r2l_reachable_min_param[0]),
                                           getDoubleValueFromRosparam(r2l_reachable_min_param[1]),
                                           getDoubleValueFromRosparam(r2l_reachable_min_param[2]));
  }
  if(nh.hasParam("r2l_reachable_max"))
  {
    XmlRpc::XmlRpcValue r2l_reachable_max_param;
    nh.getParam("r2l_reachable_max", r2l_reachable_max_param);
    if(r2l_reachable_max_param.size() != 3)
    {
      throw std::invalid_argument("Invalid r2l_reachable_max. Size of element should be 3.");
    }
    r2l_reachable_max = FootstepActionCont(getDoubleValueFromRosparam(r2l_reachable_max_param[0]),
                                           getDoubleValueFromRosparam(r2l_reachable_max_param[1]),
                                           getDoubleValueFromRosparam(r2l_reachable_max_param[2]));
  }

  if(nh.hasParam("rect_obstacle_list"))
  {
    rect_obst_list.clear();
    XmlRpc::XmlRpcValue rect_obst_list_param;
    nh.getParam("rect_obstacle_list", rect_obst_list_param);
    for(int i = 0; i < rect_obst_list_param.size(); i++)
    {
      const XmlRpc::XmlRpcValue & rect_obst_param = rect_obst_list_param[i];
      if(rect_obst_param.size() != 4)
      {
        throw std::invalid_argument("Invalid rect_obstacle_list. Size of each element should be 4.");
      }
      rect_obst_list.emplace_back(
          getDoubleValueFromRosparam(rect_obst_param[0]), getDoubleValueFromRosparam(rect_obst_param[1]),
          getDoubleValueFromRosparam(rect_obst_param[2]), getDoubleValueFromRosparam(rect_obst_param[3]));
    }
  }
  ROS_INFO_STREAM("[FootstepEnvConfigRos] Number of obstacles: " << rect_obst_list.size());
}
