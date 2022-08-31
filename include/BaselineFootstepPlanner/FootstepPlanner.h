/* Author: Masaki Murooka */

#pragma once

#include <BaselineFootstepPlanner/FootstepEnv.h>

namespace BFP
{
/** \brief Footstep planner. */
class FootstepPlanner
{
public:
  /** \brief Solution for footstep planning. */
  struct Solution
  {
    //! Whether a solution has been found
    bool is_solved = false;

    //! State ID list
    std::vector<int> id_list;

    //! State list
    std::vector<std::shared_ptr<FootstepState>> state_list;

    //! Cost
    int path_cost = 0;

    //! Heuristics weight
    double heuristics_weight = 0;

    /** \brief Reset solution. */
    void reset();
  };

public:
  /** \brief Constructor.
      \param env_config configuration of environment for footstep planning
  */
  FootstepPlanner(const std::shared_ptr<FootstepEnvConfig> & env_config);

  /** \brief Set start and goal footsteps.
      \param start_left_state state of start left footstep
      \param start_right_state state of start right footstep
      \param goal_left_state state of goal left footstep
      \param goal_right_state state of goal right footstep
      \return true if start and goal are valid
  */
  bool setStartGoal(const std::shared_ptr<FootstepState> & start_left_state,
                    const std::shared_ptr<FootstepState> & start_right_state,
                    const std::shared_ptr<FootstepState> & goal_left_state,
                    const std::shared_ptr<FootstepState> & goal_right_state);

  /** \brief Run planning.
      \param continue_until_solved whether to continue planning until solution is found
      \param max_planning_duration maximum planning duration [sec]
      \param initial_heuristics_weight initial heuristic weight
      \return whether a solution has been found

      \note If continue_until_solved is true, max_planning_duration is ignored.
  */
  bool run(bool continue_until_solved = true,
           double max_planning_duration = 1.0,
           double initial_heuristics_weight = 10.0);

public:
  //! Solution for footstep planning
  Solution solution_;

  //! Configuration of environment for footstep planning
  std::shared_ptr<FootstepEnvConfig> env_config_;

  //! Environment for footstep planning
  std::shared_ptr<FootstepEnv> env_;

  //! Graph search
  std::shared_ptr<ARAPlanner> serach_;
};
} // namespace BFP
