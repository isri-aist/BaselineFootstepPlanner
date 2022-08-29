/* Author: Masaki Murooka */

#include <ros/ros.h>

#include <BaselineFootstepPlanner/FootstepEnvConfigRos.h>

using namespace BFP;

namespace
{
double getDoubleValueFromRosparam(XmlRpc::XmlRpcValue & param)
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

  nh.param("env/theta_divide_num", theta_divide_num, theta_divide_num);
  nh.param("env/xy_divide_step", xy_divide_step, xy_divide_step);

  nh.param("cost/cost_scale", cost_scale, cost_scale);
  nh.param("cost/step_cost", step_cost, step_cost);
  nh.param("cost/theta_scale", cost_theta_scale, cost_theta_scale);

  if(nh.hasParam("heuristic/heuristic_type"))
  {
    std::string heuristic_type_str;
    nh.getParam("heuristic/heuristic_type", heuristic_type_str);
    heuristic_type = strToHeuristicType(heuristic_type_str);
  }
  nh.param("heuristic/dijkstra_path_heuristic_expand_scale", dijkstra_path_heuristic_expand_scale,
           dijkstra_path_heuristic_expand_scale);
  ROS_INFO_STREAM("[FootstepEnvConfigRos] Heuristic type: " << std::to_string(heuristic_type));

  nh.param("robot/nominal_foot_separation", nominal_foot_separation, nominal_foot_separation);

  if(nh.hasParam("robot/r2l_action_cont_list"))
  {
    r2l_action_cont_list.clear();

    XmlRpc::XmlRpcValue r2l_action_cont_list_param;
    nh.getParam("robot/r2l_action_cont_list", r2l_action_cont_list_param);
    for(int i = 0; i < r2l_action_cont_list_param.size(); i++)
    {
      // get param
      XmlRpc::XmlRpcValue r2l_action_cont_param = r2l_action_cont_list_param[i];
      if(r2l_action_cont_param.size() != 3)
      {
        throw std::invalid_argument("Invalid robot/r2l_action_cont_list. Size of each element should be 3.");
      }

      // set action list
      r2l_action_cont_list.emplace_back(getDoubleValueFromRosparam(r2l_action_cont_param[0]),
                                        getDoubleValueFromRosparam(r2l_action_cont_param[1]),
                                        getDoubleValueFromRosparam(r2l_action_cont_param[2]));
    }
  }
  ROS_INFO_STREAM("[FootstepEnvConfigRos] Number of actions: " << r2l_action_cont_list.size());

  if(nh.hasParam("robot/r2l_reachable_min"))
  {
    XmlRpc::XmlRpcValue r2l_reachable_min_param;
    nh.getParam("robot/r2l_reachable_min", r2l_reachable_min_param);
    if(r2l_reachable_min_param.size() != 3)
    {
      throw std::invalid_argument("Invalid robot/r2l_reachable_min. Size of element should be 3.");
    }

    r2l_reachable_min = FootstepActionCont(getDoubleValueFromRosparam(r2l_reachable_min_param[0]),
                                           getDoubleValueFromRosparam(r2l_reachable_min_param[1]),
                                           getDoubleValueFromRosparam(r2l_reachable_min_param[2]));
  }
  if(nh.hasParam("robot/r2l_reachable_max"))
  {
    XmlRpc::XmlRpcValue r2l_reachable_max_param;
    nh.getParam("robot/r2l_reachable_max", r2l_reachable_max_param);
    if(r2l_reachable_max_param.size() != 3)
    {
      throw std::invalid_argument("Invalid robot/r2l_reachable_max. Size of element should be 3.");
    }

    r2l_reachable_max = FootstepActionCont(getDoubleValueFromRosparam(r2l_reachable_max_param[0]),
                                           getDoubleValueFromRosparam(r2l_reachable_max_param[1]),
                                           getDoubleValueFromRosparam(r2l_reachable_max_param[2]));
  }

  if(nh.hasParam("env/rect_obstacles"))
  {
    // get param
    std::vector<double> rect_obsts_vec;
    nh.getParam("env/rect_obstacles", rect_obsts_vec);
    if(rect_obsts_vec.size() % 4 != 0)
    {
      throw std::invalid_argument("Invalid env/rect_obstacles");
    }

    // set obstacle list
    rect_obsts.clear();
    for(unsigned int i = 0; i < rect_obsts_vec.size(); i += 4)
    {
      rect_obsts.emplace_back(rect_obsts_vec[i], rect_obsts_vec[i + 1], rect_obsts_vec[i + 2], rect_obsts_vec[i + 3]);
    }
  }
  ROS_INFO_STREAM("[FootstepEnvConfigRos] Number of obstacles: " << rect_obsts.size());
}
