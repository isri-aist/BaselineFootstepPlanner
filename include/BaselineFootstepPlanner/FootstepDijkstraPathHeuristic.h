/* Author: Masaki Murooka */

#pragma once

#include <memory>

#include <sbpl/headers.h>

namespace BFP
{
class FootstepEnv;

/** \brief Dijkstra path based heuristics for footstep planning. */
class FootstepDijkstraPathHeuristic
{
public:
  /** \brief Constructor.
      \param env environment for footstep planning
      \param divide_step step to discretize the XY position [m]
      \param min minimum XY position [m]
      \param divide_num division number of XY position
  */
  FootstepDijkstraPathHeuristic(FootstepEnv * env,
                                double divide_step,
                                std::array<double, 2> min,
                                std::array<int, 2> divide_num);

  /** \brief Destructor. */
  ~FootstepDijkstraPathHeuristic();

  /** \brief Print settings. */
  void printSettings();

  /** \brief Setup grid map. */
  void setupGridMap();

  /** \brief Setup grid map.
      \param start_id ID of start state
      \param goal_id ID of goal state
   */
  void setupPathDistance(int start_id, int goal_id);

  /** \brief Calculate heuristics.
      \param from_id ID of "from" state
      \param to_id ID of "to" state
  */
  int calcHeuristic(int from_id, int to_id);

protected:
  /** \brief Convert continuous position to grid position.
      \param cxy continuous position [m]
  */
  std::array<int, 2> contToGrid(std::array<double, 2> cxy) const;

  /** \brief Convert grid position to continuous position.
      \param gxy grid position
  */
  std::array<double, 2> gridToCont(std::array<int, 2> gxy) const;

  /** \brief Get grid position from state ID
      \param state_id state ID
  */
  std::array<int, 2> stateIdToGrid(int state_id) const;

protected:
  //! Environment for footstep planning
  FootstepEnv * env_;

  //! Grid search
  std::shared_ptr<SBPL2DGridSearch> planner_;

  //! Grid map
  unsigned char ** grid_map_;

  //! Step to discretize the XY position [m]
  double divide_step_;

  //! Minimum XY position [m]
  std::array<double, 2> min_;

  //! Division number of XY position
  std::array<int, 2> divide_num_;

  //! Goal state ID
  int goal_id_;

  //! Whether to dump distance field
  bool dump_distance_field_ = false;
};
} // namespace BFP
