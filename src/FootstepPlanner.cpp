/* Author: Masaki Murooka */

#include <BaselineFootstepPlanner/FootstepPlanner.h>

using namespace BFP;

void FootstepPlanner::Solution::reset()
{
  is_solved = false;
  id_list.clear();
  state_list.clear();
  path_cost = 0;
  heuristics_weight = 0;
}

FootstepPlanner::FootstepPlanner(const std::shared_ptr<FootstepEnvConfig> & env_config)
: env_config_(env_config), env_(std::make_shared<FootstepEnv>(env_config_)),
  serach_(std::make_shared<ARAPlanner>(env_.get(), true))
{
}

bool FootstepPlanner::setStartGoal(const std::shared_ptr<FootstepState> & start_left_state,
                                   const std::shared_ptr<FootstepState> & start_right_state,
                                   const std::shared_ptr<FootstepState> & goal_left_state,
                                   const std::shared_ptr<FootstepState> & goal_right_state)
{
  // construct environment and planner
  env_ = std::make_shared<FootstepEnv>(env_config_);
  serach_ = std::make_shared<ARAPlanner>(env_.get(), true);

  // set start and goal
  if(!env_->setStart(start_left_state, start_right_state))
  {
    return false;
  }
  if(!env_->setGoal(goal_left_state, goal_right_state))
  {
    return false;
  }

  // setup environment
  // need to call these methods after setStart and setGoal because start and goal IDs are necessary
  MDPConfig mdp_config;
  env_->InitializeEnv(nullptr); // DijkstraPathHeuristic is initialized here
  env_->InitializeMDPCfg(&mdp_config);

  // setup planner
  if(serach_->set_start(mdp_config.startstateid) == 0)
  {
    throw std::runtime_error("Failed to set start state.");
  }
  if(serach_->set_goal(mdp_config.goalstateid) == 0)
  {
    throw std::runtime_error("Failed to set goal state.");
  }

  // reset solution
  solution_.reset();

  return true;
}

bool FootstepPlanner::run(bool continue_until_solved, double max_planning_duration, double initial_heuristics_weight)
{
  if(!env_->is_inited_)
  {
    return false;
  }

  // plan
  serach_->set_initialsolution_eps(initial_heuristics_weight);
  serach_->set_search_mode(continue_until_solved);
  solution_.is_solved = serach_->replan(max_planning_duration, &solution_.id_list, &solution_.path_cost);

  // add the start and goal right footsteps because the only left footsteps are assumed to be start and goal
  if(solution_.id_list[1] != env_->start_right_id_)
  {
    solution_.id_list.insert(solution_.id_list.begin(), env_->start_right_id_);
  }
  if(solution_.id_list[solution_.id_list.size() - 2] != env_->goal_right_id_)
  {
    solution_.id_list.push_back(env_->goal_right_id_);
  }

  // set solution
  for(const auto & solution_id : solution_.id_list)
  {
    solution_.state_list.push_back(env_->getStateFromId(solution_id));
  }
  solution_.heuristics_weight = serach_->get_solution_eps();

  return solution_.is_solved;
}
