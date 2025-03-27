/* Author: Masaki Murooka */

#pragma once

#include <unordered_map>
#include <vector>

#include <sbpl/headers.h>

#include <BaselineFootstepPlanner/FootstepState.h>
#include <BaselineFootstepPlanner/MathUtils.h>

namespace BFP
{
/** \brief Heuristic type. */
enum class HeuristicType
{
  //! Euclidean
  Euclidean = 0,

  //! Dijkstra path
  DijkstraPath
};

/** \brief Convert string to heuristic type.
    \param heuristic_type_str heuristic type string
*/
HeuristicType strToHeuristicType(const std::string & heuristic_type_str);

/** \brief Rectangle shape. */
struct Rect
{
  //! Center X position [m]
  double x_center;

  //! Center Y position [m]
  double y_center;

  //! X half length [m]
  double x_half_length;

  //! Y half length [m]
  double y_half_length;

  /** \brief Constructor.
      \param _x_center center X position [m]
      \param _y_center center Y position [m]
      \param _x_half_length X half length [m]
      \param _y_half_length Y half length [m]
  */
  Rect(double _x_center, double _y_center, double _x_half_length, double _y_half_length)
  : x_center(_x_center), y_center(_y_center), x_half_length(_x_half_length), y_half_length(_y_half_length)
  {
  }
};

/** \brief Configuration of environment for footstep planning. */
struct FootstepEnvConfig
{
  //! Division number to discretize orientation
  int theta_divide_num = 64;

  //! Step to discretize the XY position [m]
  double xy_divide_step = 0.01;

  /** \brief Cost scale

      This is the scale by which cost is multiplied before rounding it to an int-type value.
  */
  double cost_scale = 1e3;

  //! Scale for converting orientation distance to position distance in cost calculation [m/rad]
  double cost_theta_scale = 0.0;

  //! Cost of one step (unit correspond to [m])
  double step_cost = 1.0;

  //! Heuristics type
  HeuristicType heuristic_type = HeuristicType::DijkstraPath;

  //! Expansion scale of grid map for Dijkstra path based heuristics
  double dijkstra_path_heuristic_expand_scale = 5.0;

  //! Nominal distance between left and right feet [m]
  double nominal_foot_separation = 0.2;

  //! List of feasible footsteps of left foot relative to right foot
  std::vector<FootstepActionCont> r2l_action_cont_list;

  /** \brief Foot reachable region of left foot relative to right foot

      This is used to improve goal convergence.

      @{
  */
  FootstepActionCont r2l_reachable_min = FootstepActionCont(-0.05, 0.0, degToRad(-10));
  FootstepActionCont r2l_reachable_max = FootstepActionCont(0.1, 0.05, degToRad(10));
  //! @}

  //! Rectangle obstacle list
  std::vector<Rect> rect_obst_list;
};

class FootstepDijkstraPathHeuristic;

/** \brief Environment for footstep planning. */
class FootstepEnv : public DiscreteSpaceInformation
{
  friend class FootstepDijkstraPathHeuristic;

public:
  /** \brief Constructor.
      \param config configuration
  */
  FootstepEnv(const std::shared_ptr<FootstepEnvConfig> & config);

  /** \brief Initialize environment. */
  virtual bool InitializeEnv(const char * sEnvFile) override;

  /** \brief Initialize MDP configuration. */
  virtual bool InitializeMDPCfg(MDPConfig * MDPCfg) override;

  /** \brief Get heuristics between states.
      \param from_id ID of "from" state
      \param to_id ID of "to" state
  */
  virtual int GetFromToHeuristic(int from_id, int to_id) override;

  /** \brief Get heuristics to goal state.
      \param id ID of "from" state
  */
  virtual int GetGoalHeuristic(int id) override;

  inline virtual int GetStartHeuristic(int // id
                                       ) override
  {
    throw std::runtime_error("FootstepEnv::GetStartHeuristic is not expected to be called.");
    return 0;
  }

  /** \brief Get successor states.
      \param source_id ID of source state
      \param succ_id_list ID list of successor states
      \param cost_list cost list from source state to each successor state
  */
  virtual void GetSuccs(int source_id, std::vector<int> * succ_id_list, std::vector<int> * cost_list) override;

  inline virtual void GetPreds(int, // target_id
                               std::vector<int> *, // prev_id_list
                               std::vector<int> * // cost_list
                               ) override
  {
    throw std::runtime_error("FootstepEnv::GetPreds is not expected to be called.");
  }

  inline virtual void SetAllActionsandAllOutcomes(CMDPSTATE * // state
                                                  ) override
  {
    throw std::runtime_error("FootstepEnv::SetAllActionsandAllOutcomes is not expected to be called.");
  }

  inline virtual void SetAllPreds(CMDPSTATE * // state
                                  ) override
  {
    throw std::runtime_error("FootstepEnv::SetAllPreds is not expected to be called.");
  }

  /** \brief Get the number of created states. */
  inline virtual int SizeofCreatedEnv() override
  {
    return static_cast<int>(id_to_state_list_.size());
  }

  /** \brief Print state. */
  inline virtual void PrintState(int, // id
                                 bool, // verbose
                                 FILE * // f_out = nullptr
                                 ) override
  {
    // do nothing
  }

  inline virtual void PrintEnv_Config(FILE * // f_out
                                      ) override
  {
    throw std::runtime_error("FootstepEnv::PrintEnv_Config is not expected to be called.");
  }

  /** \brief Const accessor to the configuration. */
  inline const std::shared_ptr<FootstepEnvConfig> & config() const
  {
    return config_;
  }

  /** \brief Clear all states. */
  void clear();

  /** \brief Convert continuous position to discretized position.
      \param cx continuous X or Y position [m]
  */
  int contToDiscXy(double cx) const;

  /** \brief Convert continuous orientation to discretized orientation.
      \param ctheta continuous orientation [rad]
  */
  int contToDiscTheta(double ctheta) const;

  /** \brief Convert discretized position to continuous position.
      \param x discretized X or Y position
  */
  template<typename ScalarType = int>
  inline double discToContXy(ScalarType x) const
  {
    double cx = x * config_->xy_divide_step;

    return cx;
  }

  /** \brief Convert discretized orientation to continuous orientation.
      \param theta discretized orientation
  */
  template<typename ScalarType = int>
  inline double discToContTheta(ScalarType theta) const
  {
    double theta_divide_step = 2 * M_PI / config_->theta_divide_num;
    double ctheta = (theta + 0.5) * theta_divide_step;

    // if (ctheta < 0 || ctheta >= 2 * M_PI) {
    //   throw std::runtime_error("Invalid ctheta in discToContTheta: " + std::to_string(ctheta));
    // }

    return ctheta;
  }

  /** \brief Calculate cost from current state to successor state.
      \param current_state current state
      \param succ_state successor state
  */
  int calcCost(const std::shared_ptr<FootstepState> & current_state,
               const std::shared_ptr<FootstepState> & succ_state) const;

  /** \brief Make state from feet midpose.
      \param midpose feet midpose (x [m], y [m], theta [rad])
      \param foot foot
  */
  template<typename ArrayType>
  inline std::shared_ptr<FootstepState> makeStateFromMidpose(const ArrayType & midpose, Foot foot) const
  {
    // the offset for the nominal foot separation from right to left
    // rotate the 2D vector (0; nominal_foot_separation / 2.0) by theta
    double cx_separation_half_offset = -1 * std::sin(midpose[2]) * config_->nominal_foot_separation / 2.0;
    double cy_separation_half_offset = std::cos(midpose[2]) * config_->nominal_foot_separation / 2.0;

    int offset_sign = (foot == Foot::LEFT ? 1 : -1);
    return std::make_shared<FootstepState>(contToDiscXy(midpose[0] + offset_sign * cx_separation_half_offset),
                                           contToDiscXy(midpose[1] + offset_sign * cy_separation_half_offset),
                                           contToDiscTheta(midpose[2]), foot);
  }

  /** \brief Set start footstep.
      \param left_state state of start left footstep
      \param right_state state of start right footstep
      \return true if start footstep is valid
  */
  bool setStart(const std::shared_ptr<FootstepState> & left_state, const std::shared_ptr<FootstepState> & right_state);

  /** \brief Set goal footstep.
      \param left_state state of goal left footstep
      \param right_state state of goal right footstep
      \return true if goal footstep is valid
  */
  bool setGoal(const std::shared_ptr<FootstepState> & left_state, const std::shared_ptr<FootstepState> & right_state);

  /** \brief Get state from state ID.
      \param id state ID
  */
  std::shared_ptr<FootstepState> getStateFromId(int id) const;

  /** \brief Print state.
      \param state state

      \note Unlike FootstepState::print, it prints continuous position and orientation.
  */
  void printState(const std::shared_ptr<FootstepState> & state) const;

  /** \brief Get the number of created states. */
  inline int stateNum() const
  {
    return static_cast<int>(id_to_state_list_.size());
  }

protected:
  /** \brief Setup ROS parameters. */
  void setupRosParam();

  /** \brief Setup footstep action. */
  void setupAction();

  /** \brief Setup Dijkstra path heuristics. */
  void setupDijkstraPathHeuristic();

  /** \brief Check collision with rectangle obstacle.
      \param cx continuous X position of footstep [m]
      \param cy continuous Y position of footstep [m]
      \param rect_obst rectangle obstacle
      \return true if collision
  */
  bool checkRectCollision(double cx, double cy, const Rect & rect_obst) const;

  /** \brief Check if footstep position is valid.
      \param cx continuous X position of footstep [m]
      \param cy continuous Y position of footstep [m]
      \return true if valid
  */
  bool checkXyValid(double cx, double cy) const;

  /** \brief Check if state is valid.
      \param state state
      \return true if valid
  */
  bool checkStateValid(const std::shared_ptr<FootstepState> & state) const;

  /** \brief Calculate Euclidean heuristics.
      \param from_id ID of "from" state
      \param to_id ID of "to" state
  */
  int calcEuclideanHeuristic(int from_id, int to_id) const;

  /** \brief Calculate Dijkstra path heuristics.
      \param from_id ID of "from" state
      \param to_id ID of "to" state
  */
  int calcDijkstraPathHeuristic(int from_id, int to_id) const;

  /** \brief Check if state is reachable to goal state.
      \param state state

      This is used to improve goal convergence.
  */
  bool reachableToGoal(const std::shared_ptr<FootstepState> & state);

  /** \brief Add state if it is not added yet.
      \param state state
  */
  int addStateIfNotExists(const std::shared_ptr<FootstepState> & state);

public:
  //! Whether start and goal are initialized
  bool is_inited_ = false;

  //! State ID of start left footstep
  int start_left_id_ = -1;

  //! State ID of start right footstep
  int start_right_id_ = -1;

  //! State ID of goal left footstep
  int goal_left_id_ = -1;

  //! State ID of goal right footstep
  int goal_right_id_ = -1;

  //! Number of state expansion
  int expand_cnt_ = 0;

protected:
  //! Configuration
  std::shared_ptr<FootstepEnvConfig> config_;

  /** \brief State list

      The list index is state ID.
  */
  std::vector<std::shared_ptr<FootstepState>> id_to_state_list_;

  //! Map from state to state ID
  std::unordered_map<std::shared_ptr<FootstepState>, int> state_to_id_map_;

  /** \brief Footstep action list for each discretized theta

      @{
  */
  std::vector<std::vector<FootstepAction>> r2l_theta_to_actions_;
  std::vector<std::vector<FootstepAction>> l2r_theta_to_actions_;
  //! @}

  /** \brief Maximum footstep distance

      This is calculated from ::Configuration::r2l_action_cont_list automatically.

      @{ 
  */
  double step_xy_max_;
  double step_theta_max_;
  //! @}

  //! Dijkstra path based heuristics
  std::shared_ptr<FootstepDijkstraPathHeuristic> dijkstra_path_heuristic_;
};
} // namespace BFP

namespace std
{
/** \brief Convert heuristic type to string.
    \param heuristic_type heuristic type
*/
std::string to_string(const BFP::HeuristicType & heuristic_type);
} // namespace std
