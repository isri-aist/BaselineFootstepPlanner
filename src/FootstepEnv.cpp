/* Author: Masaki Murooka */

#include <algorithm>

#include <ros/console.h>

#include <BaselineFootstepPlanner/FootstepDijkstraPathHeuristic.h>
#include <BaselineFootstepPlanner/FootstepEnv.h>

using namespace BFP;

HeuristicType BFP::strToHeuristicType(const std::string & heuristic_type_str)
{
  if(heuristic_type_str == "Euclidean")
  {
    return HeuristicType::Euclidean;
  }
  else if(heuristic_type_str == "DijkstraPath")
  {
    return HeuristicType::DijkstraPath;
  }
  else
  {
    throw std::runtime_error("Unsupported heuristic type: " + heuristic_type_str);
  }
}

FootstepEnv::FootstepEnv(const std::shared_ptr<FootstepEnvConfig> & config)
: DiscreteSpaceInformation(), config_(config)
{
  setupAction();
}

bool FootstepEnv::InitializeEnv(const char * sEnvFile)
{
  if((start_left_id_ == -1) || (start_right_id_ == -1) || (goal_left_id_ == -1) || (goal_right_id_ == -1))
  {
    throw std::runtime_error("Start or goal is uninitialized.");
  }

  if(config_->heuristic_type == HeuristicType::DijkstraPath)
  {
    setupDijkstraPathHeuristic();
  }

  return true;
}

bool FootstepEnv::InitializeMDPCfg(MDPConfig * MDPCfg)
{
  if(start_left_id_ < 0 || goal_left_id_ < 0)
  {
    throw std::runtime_error("Start or goal is uninitialized.");
  }

  // treat left foot as start and goal
  MDPCfg->startstateid = start_left_id_;
  MDPCfg->goalstateid = goal_left_id_;

  return true;
}

int FootstepEnv::GetFromToHeuristic(int from_id, int to_id)
{
  if(config_->heuristic_type == HeuristicType::Euclidean)
  {
    return calcEuclideanHeuristic(from_id, to_id);
  }
  else if(config_->heuristic_type == HeuristicType::DijkstraPath)
  {
    return calcDijkstraPathHeuristic(from_id, to_id);
  }
  else
  {
    throw std::runtime_error("Invalid heuristic type: " + std::to_string(config_->heuristic_type));
  }

  return 0;
}

int FootstepEnv::GetGoalHeuristic(int id)
{
  // treat left foot as goal
  return GetFromToHeuristic(id, goal_left_id_);
}

void FootstepEnv::GetSuccs(int source_id, std::vector<int> * succ_id_list, std::vector<int> * cost_list)
{
  // treat left foot as goal
  if(source_id == goal_left_id_)
  {
    return;
  }

  // get the current state
  // not sure why, but it doesn't work with reference
  const std::shared_ptr<FootstepState> current_state = id_to_state_list_[source_id];

  // if the source state is the goal of right left (i.e., opposite foot of true goal),
  // set the goal of left foot to the successors
  if(source_id == goal_right_id_)
  {
    int succ_id = goal_left_id_;
    const std::shared_ptr<FootstepState> & succ_state = id_to_state_list_[succ_id];
    succ_id_list->push_back(succ_id);
    cost_list->push_back(calcCost(current_state, succ_state));
    return;
  }
  // if the source state is reachable to the goal of opposite foot,
  // set the goal of opposite foot to the successors
  else if(reachableToGoal(current_state))
  {
    int succ_id;
    if(current_state->foot_ == Foot::LEFT)
    {
      succ_id = goal_right_id_;
    }
    else // if(current_state->foot_ == Foot::RIGHT)
    {
      succ_id = goal_left_id_;
    }
    const std::shared_ptr<FootstepState> & succ_state = id_to_state_list_[succ_id];
    succ_id_list->push_back(succ_id);
    cost_list->push_back(calcCost(current_state, succ_state));
    return;
  }

  // get the actions of corresponding foot and theta
  std::vector<FootstepAction> * actions;
  if(current_state->foot_ == Foot::LEFT)
  {
    // from the left foot to the right foot
    actions = &(l2r_theta_to_actions_[current_state->theta_]);
  }
  else // if(current_state->foot_ == Foot::RIGHT)
  {
    actions = &(r2l_theta_to_actions_[current_state->theta_]);
  }

  // apply each action
  for(const auto & action : *actions)
  {
    expand_cnt_++;

    // create new state
    const std::shared_ptr<FootstepState> & succ_state = std::make_shared<FootstepState>(
        current_state->x_ + action.x, current_state->y_ + action.y,
        (current_state->theta_ + action.theta + config_->theta_divide_num) % config_->theta_divide_num,
        opposite(current_state->foot_));

    if(succ_state->theta_ < 0 || succ_state->theta_ >= config_->theta_divide_num)
    {
      throw std::runtime_error("Invalid theta in GetSuccs: " + std::to_string(succ_state->theta_));
    }

    // check validity
    if(!checkStateValid(succ_state))
    {
      continue;
    }

    // push
    int succ_id = addStateIfNotExists(succ_state);
    succ_id_list->push_back(succ_id);
    cost_list->push_back(calcCost(current_state, succ_state));
  }
}

void FootstepEnv::clear()
{
  id_to_state_list_.clear();
  state_to_id_map_.clear();
}

int FootstepEnv::contToDiscXy(double cx) const
{
  int x = std::round(cx / config_->xy_divide_step);

  return x;
}

int FootstepEnv::contToDiscTheta(double ctheta) const
{
  double theta_divide_step = 2 * M_PI / config_->theta_divide_num;
  int theta = std::floor(radMod(ctheta) / theta_divide_step);

  // if (theta < 0 || theta >= config_->theta_divide_num) {
  //   throw std::runtime_error("Invalid theta in contToDiscTheta: " + std::to_string(theta));
  // }

  return theta;
}

int FootstepEnv::calcCost(const std::shared_ptr<FootstepState> & current_state,
                          const std::shared_ptr<FootstepState> & succ_state) const
{
  double dist_xy = discToContXy<double>(current_state->calcDistanceXy(succ_state)); // [m]
  double dist_theta =
      discToContTheta<double>(current_state->calcDistanceTheta(succ_state, config_->theta_divide_num)); // [rad]

  return static_cast<int>(config_->cost_scale
                          * (dist_xy + config_->cost_theta_scale * dist_theta + config_->step_cost));
}

bool FootstepEnv::setStart(const std::shared_ptr<FootstepState> & left_state,
                           const std::shared_ptr<FootstepState> & right_state)
{
  if(!(checkStateValid(left_state) && checkStateValid(right_state)))
  {
    ROS_WARN("[FootstepEnv] Invalid start.");
    return false;
  }

  start_left_id_ = addStateIfNotExists(left_state);
  start_right_id_ = addStateIfNotExists(right_state);

  return true;
}

bool FootstepEnv::setGoal(const std::shared_ptr<FootstepState> & left_state,
                          const std::shared_ptr<FootstepState> & right_state)
{
  if(!(checkStateValid(left_state) && checkStateValid(right_state)))
  {
    ROS_WARN("[FootstepEnv] Invalid goal.");
    return false;
  }

  goal_left_id_ = addStateIfNotExists(left_state);
  goal_right_id_ = addStateIfNotExists(right_state);

  return true;
}

std::shared_ptr<FootstepState> FootstepEnv::getStateFromId(int id) const
{
  if(!(0 <= id && static_cast<unsigned int>(id) < id_to_state_list_.size()))
  {
    throw std::runtime_error("Invalid id: " + std::to_string(id));
  }

  return id_to_state_list_[id];
}

void FootstepEnv::printState(const std::shared_ptr<FootstepState> & state) const
{
  ROS_INFO("FootstepState[id: %d, x: %5.2lf, y: %5.2lf, theta: %5.2lf, foot: %s]", state->id_, discToContXy(state->x_),
           discToContXy(state->y_), discToContTheta(state->theta_), std::to_string(state->foot_).c_str());
}

void FootstepEnv::setupAction()
{
  r2l_theta_to_actions_.resize(config_->theta_divide_num);
  l2r_theta_to_actions_.resize(config_->theta_divide_num);
  for(int theta = 0; theta < config_->theta_divide_num; theta++)
  {
    // get constants for rotation
    double ctheta = discToContTheta(theta);
    double cos_theta = std::cos(ctheta);
    double sin_theta = std::sin(ctheta);

    // "caction" means continuous action
    for(const auto & caction : config_->r2l_action_cont_list)
    {
      // add footstep action from right to left
      {
        // get the delta xy of action, which is represented in local frame
        double delta_cx = caction.x;
        double delta_cy = caction.y + config_->nominal_foot_separation; // add foot separation
        double delta_ctheta = caction.theta;

        // rotate the delta xy in local frame and get the delta xy in world frame
        double delta_cx_world = cos_theta * delta_cx - sin_theta * delta_cy;
        double delta_cy_world = sin_theta * delta_cx + cos_theta * delta_cy;

        r2l_theta_to_actions_[theta].emplace_back(contToDiscXy(delta_cx_world), contToDiscXy(delta_cy_world),
                                                  contToDiscTheta(delta_ctheta));
      }
      // add footstep action from left to right
      {
        // get the delta xy of action, which is represented in local frame
        double delta_cx = caction.x;
        double delta_cy = -1 * (caction.y + config_->nominal_foot_separation); // add foot separation
        double delta_ctheta = -1 * caction.theta;

        // rotate the delta xy in local frame and get the delta xy in world frame
        double delta_cx_world = cos_theta * delta_cx - sin_theta * delta_cy;
        double delta_cy_world = sin_theta * delta_cx + cos_theta * delta_cy;

        l2r_theta_to_actions_[theta].emplace_back(contToDiscXy(delta_cx_world), contToDiscXy(delta_cy_world),
                                                  contToDiscTheta(delta_ctheta));
      }
    }
  }

  // set max step
  step_xy_max_ = 0;
  step_theta_max_ = 0;
  for(const auto & caction : config_->r2l_action_cont_list)
  {
    // footstep action from right to left
    double delta_cx = caction.x;
    double delta_cy = caction.y + config_->nominal_foot_separation; // add foot separation
    double delta_ctheta = caction.theta;
    step_xy_max_ = std::max(step_xy_max_, std::sqrt(delta_cx * delta_cx + delta_cy * delta_cy));
    step_theta_max_ = std::max(step_theta_max_, std::abs(delta_ctheta));
  }
  ROS_DEBUG("[FootstepEnv::setupAction] step_xy_max: %lf, step_theta_max: %lf", step_xy_max_, step_theta_max_);
}

void FootstepEnv::setupDijkstraPathHeuristic()
{
  // get start and goal state
  const std::shared_ptr<FootstepState> & start_left_state = getStateFromId(start_left_id_);
  const std::shared_ptr<FootstepState> & start_right_state = getStateFromId(start_right_id_);
  const std::shared_ptr<FootstepState> & goal_left_state = getStateFromId(goal_left_id_);
  const std::shared_ptr<FootstepState> & goal_right_state = getStateFromId(goal_right_id_);

  // get min and max position
  std::vector<double> cx_list = {discToContXy(start_left_state->x_), discToContXy(start_right_state->x_),
                                 discToContXy(goal_left_state->x_), discToContXy(goal_right_state->x_)};
  std::vector<double> cy_list = {discToContXy(start_left_state->y_), discToContXy(start_right_state->y_),
                                 discToContXy(goal_left_state->y_), discToContXy(goal_right_state->y_)};
  std::array<double, 2> cxy_min = {*std::min_element(cx_list.begin(), cx_list.end()),
                                   *std::min_element(cy_list.begin(), cy_list.end())};
  std::array<double, 2> cxy_max = {*std::max_element(cx_list.begin(), cx_list.end()),
                                   *std::max_element(cy_list.begin(), cy_list.end())};
  std::array<double, 2> cxy_mid = {(cxy_min[X] + cxy_max[X]) / 2, (cxy_min[Y] + cxy_max[Y]) / 2};
  double cxy_range_offset = 1.0; // [m]
  std::array<double, 2> cxy_range = {cxy_max[X] - cxy_min[X] + cxy_range_offset,
                                     cxy_max[Y] - cxy_min[Y] + cxy_range_offset};

  // expand min and max position
  std::array<double, 2> cxy_min_expanded = {
      cxy_mid[X] - config_->dijkstra_path_heuristic_expand_scale * cxy_range[X] / 2,
      cxy_mid[Y] - config_->dijkstra_path_heuristic_expand_scale * cxy_range[Y] / 2};
  std::array<double, 2> cxy_max_expanded = {
      cxy_mid[X] + config_->dijkstra_path_heuristic_expand_scale * cxy_range[X] / 2,
      cxy_mid[Y] + config_->dijkstra_path_heuristic_expand_scale * cxy_range[Y] / 2};

  // get arguments
  std::array<int, 2> divide_num = {
      static_cast<int>(std::ceil((cxy_max_expanded[X] - cxy_min_expanded[X]) / config_->xy_divide_step)),
      static_cast<int>(std::ceil((cxy_max_expanded[Y] - cxy_min_expanded[Y]) / config_->xy_divide_step))};

  // make FootstepDijkstraPathHeuristic instance
  dijkstra_path_heuristic_ =
      std::make_shared<FootstepDijkstraPathHeuristic>(this, config_->xy_divide_step, cxy_min_expanded, divide_num);

  dijkstra_path_heuristic_->setupGridMap();
  dijkstra_path_heuristic_->setupPathDistance(start_left_id_, goal_left_id_);
}

bool FootstepEnv::checkRectCollision(double cx, double cy, const Rect & rect_obst) const
{
  double x_center = rect_obst.x_center;
  double y_center = rect_obst.y_center;
  double x_half_length = rect_obst.x_half_length;
  double y_half_length = rect_obst.y_half_length;
  return (x_center - x_half_length <= cx) && (cx <= x_center + x_half_length) && (y_center - y_half_length <= cy)
         && (cy <= y_center + y_half_length);
}

bool FootstepEnv::checkXyValid(double cx, double cy) const
{
  for(const auto & rect_obst : config_->rect_obst_list)
  {
    if(checkRectCollision(cx, cy, rect_obst))
    {
      return false;
    }
  }
  return true;
}

bool FootstepEnv::checkStateValid(const std::shared_ptr<FootstepState> & state) const
{
  double cx = discToContXy(state->x_);
  double cy = discToContXy(state->y_);

  return checkXyValid(cx, cy);
}

int FootstepEnv::calcEuclideanHeuristic(int from_id, int to_id) const
{
  if(!(0 <= from_id && static_cast<unsigned int>(from_id) < id_to_state_list_.size()))
  {
    throw std::runtime_error("Invalid from_id: " + std::to_string(from_id));
  }
  if(!(0 <= to_id && static_cast<unsigned int>(to_id) < id_to_state_list_.size()))
  {
    throw std::runtime_error("Invalid to_id: " + std::to_string(to_id));
  }

  const std::shared_ptr<FootstepState> & from_state = id_to_state_list_[from_id];
  const std::shared_ptr<FootstepState> & to_state = id_to_state_list_[to_id];
  double dist_xy = discToContXy(from_state->calcDistanceXy(to_state)); // [m]
  double dist_theta = discToContTheta(from_state->calcDistanceTheta(to_state, config_->theta_divide_num)); // [rad]
  int step_num = std::ceil(std::max((dist_xy / step_xy_max_), (dist_theta / step_theta_max_)));

  return static_cast<int>(config_->cost_scale
                          * (dist_xy + config_->cost_theta_scale * dist_theta + config_->step_cost * step_num));
}

int FootstepEnv::calcDijkstraPathHeuristic(int from_id, int to_id) const
{
  return dijkstra_path_heuristic_->calcHeuristic(from_id, to_id);
}

bool FootstepEnv::reachableToGoal(const std::shared_ptr<FootstepState> & state)
{
  // 1. get nominal state of next foot (i.e., opposite foot)
  // current state
  double cx = discToContXy(state->x_);
  double cy = discToContXy(state->y_);
  double ctheta = discToContTheta(state->theta_);

  // the offset for the nominal foot separation from right to left
  // rotate the 2D vector (0; nominal_foot_separation) by ctheta
  double cx_separation_offset = -1 * std::sin(ctheta) * config_->nominal_foot_separation;
  double cy_separation_offset = std::cos(ctheta) * config_->nominal_foot_separation;

  int offset_sign;
  if(state->foot_ == Foot::LEFT)
  {
    // from left to right
    offset_sign = -1;
  }
  else // if(state->foot_ == Foot::RIGHT)
  {
    // from right to left
    offset_sign = 1;
  }

  // next nominal state
  double nominal_cx = cx + offset_sign * cx_separation_offset;
  double nominal_cy = cy + offset_sign * cy_separation_offset;
  double nominal_ctheta = ctheta;

  // 2. get goal of next foot (i.e., opposite foot)
  int goal_id;
  if(state->foot_ == Foot::LEFT)
  {
    goal_id = goal_right_id_;
  }
  else // if(state->foot_ == Foot::RIGHT)
  {
    goal_id = goal_left_id_;
  }
  const std::shared_ptr<FootstepState> & goal_state = id_to_state_list_[goal_id];

  double goal_cx = discToContXy(goal_state->x_);
  double goal_cy = discToContXy(goal_state->y_);
  double goal_ctheta = discToContTheta(goal_state->theta_);

  // 3. evaluate whether the difference is within threshold
  double delta_cx_world = goal_cx - nominal_cx;
  double delta_cy_world = goal_cy - nominal_cy;
  double delta_ctheta = goal_ctheta - nominal_ctheta;

  // rotate the delta xy in world frame and get the delta xy in local frame
  double cos_nominal_theta = std::cos(-1 * nominal_ctheta);
  double sin_nominal_theta = std::sin(-1 * nominal_ctheta);
  double delta_cx_local = cos_nominal_theta * delta_cx_world - sin_nominal_theta * delta_cy_world;
  double delta_cy_local = sin_nominal_theta * delta_cx_world + cos_nominal_theta * delta_cy_world;

  // flip y and theta if transit from left to right
  if(state->foot_ == Foot::LEFT)
  {
    delta_cy_local *= -1;
    delta_ctheta *= -1;
  }

  if((config_->r2l_reachable_min.x <= delta_cx_local) && (delta_cx_local <= config_->r2l_reachable_max.x)
     && (config_->r2l_reachable_min.y <= delta_cy_local) && (delta_cy_local <= config_->r2l_reachable_max.y)
     && (config_->r2l_reachable_min.theta <= delta_ctheta) && (delta_ctheta <= config_->r2l_reachable_max.theta))
  {
    return true;
  }

  return false;
}

int FootstepEnv::addStateIfNotExists(const std::shared_ptr<FootstepState> & state)
{
  if(state_to_id_map_.find(state) == state_to_id_map_.end())
  {
    // create new state
    std::shared_ptr<FootstepState> new_state(state);
    int new_id = id_to_state_list_.size();
    new_state->id_ = new_id;

    // add to list
    id_to_state_list_.push_back(new_state);

    // add to map
    state_to_id_map_[state] = new_id;

    // add to index map (defined inside sbpl)
    int * entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for(int i = 0; i < NUMOFINDICES_STATEID2IND; ++i)
    {
      StateID2IndexMapping[new_id][i] = -1;
    }
    if(StateID2IndexMapping.size() - 1 != static_cast<unsigned int>(new_id))
    {
      throw std::runtime_error("Size of StateID2IndexMapping and new_id does not match");
    }

    return new_id;
  }

  // return existing id
  return state_to_id_map_[state];
}

std::string std::to_string(const HeuristicType & heuristic_type)
{
  if(heuristic_type == HeuristicType::Euclidean)
  {
    return std::string("Euclidean");
  }
  else if(heuristic_type == HeuristicType::DijkstraPath)
  {
    return std::string("DijkstraPath");
  }
  else
  {
    throw std::runtime_error("Unsupported heuristic type: " + std::to_string(static_cast<int>(heuristic_type)));
  }
}
