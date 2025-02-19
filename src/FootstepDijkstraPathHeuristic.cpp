/* Author: Masaki Murooka */

#include <chrono>
#include <fstream>

#include <BaselineFootstepPlanner/FootstepDijkstraPathHeuristic.h>
#include <BaselineFootstepPlanner/FootstepEnv.h>
#include <BaselineFootstepPlanner/console.h>

using namespace BFP;

FootstepDijkstraPathHeuristic::FootstepDijkstraPathHeuristic(FootstepEnv * env,
                                                             double divide_step,
                                                             std::array<double, 2> min,
                                                             std::array<int, 2> divide_num)
: env_(env), planner_(std::make_shared<SBPL2DGridSearch>(divide_num[X], divide_num[Y], divide_step)),
  divide_step_(divide_step), min_(min), divide_num_(divide_num)
{
  printSettings();
}

FootstepDijkstraPathHeuristic::~FootstepDijkstraPathHeuristic()
{
  // clear planner
  planner_->destroy();

  // clear map
  for(int x = 0; x < divide_num_[X]; x++)
  {
    if(grid_map_[x])
    {
      delete[] grid_map_[x];
      grid_map_[x] = nullptr;
    }
  }
  delete[] grid_map_;
}

void FootstepDijkstraPathHeuristic::printSettings()
{
  std::array<double, 2> max = {min_[X] + divide_num_[X] * divide_step_, min_[Y] + divide_num_[Y] * divide_step_};
  BFP_DEBUG("[FootstepDijkstraPathHeuristic] Initialize FootstepDijkstraPathHeuristic.");
  BFP_DEBUG("  - min: (%lf, %lf)  max: (%lf, %lf)  divide_num: (%d, %d).", min_[X], min_[Y], max[X], max[Y],
            divide_num_[X], divide_num_[Y]);
}

void FootstepDijkstraPathHeuristic::setupGridMap()
{
  int width = divide_num_[X];
  int height = divide_num_[Y];

  // alloc map
  grid_map_ = new unsigned char *[width];
  for(int x = 0; x < width; x++)
  {
    grid_map_[x] = new unsigned char[height];
  }

  // setup map
  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      std::array<int, 2> gxy = {x, y};
      std::array<double, 2> cxy = gridToCont(gxy);
      if(env_->checkXyValid(cxy[X], cxy[Y]))
      {
        grid_map_[x][y] = 0;
      }
      else
      {
        grid_map_[x][y] = 254;
      }
    }
  }
}

void FootstepDijkstraPathHeuristic::setupPathDistance(int start_id, int goal_id)
{
  auto start_time = std::chrono::system_clock::now();

  // set start and goal
  goal_id_ = goal_id;

  std::array<int, 2> start_gxy = stateIdToGrid(start_id);
  std::array<int, 2> goal_gxy = stateIdToGrid(goal_id);
  BFP_DEBUG("[FootstepDijkstraPathHeuristic] start grid: (%d, %d), goal grid: (%d, %d)", start_gxy[X], start_gxy[Y],
            goal_gxy[X], goal_gxy[Y]);

  // search
  constexpr int obst_thre = 100;
  SBPL_2DGRIDSEARCH_TERM_CONDITION term_cond = SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS;
  // search from the goal to the start
  planner_->search(grid_map_, obst_thre, goal_gxy[X], goal_gxy[Y], // from
                   start_gxy[X], start_gxy[Y], // to
                   term_cond);

  double search_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start_time).count();
  BFP_DEBUG("[FootstepDijkstraPathHeuristic] calcPathDistance took %lf [sec]", search_duration);

  // dump the distance field
  if(dump_distance_field_)
  {
    std::string dijkstra_filename = "/tmp/dijkstra_distance_field.dat";
    std::ofstream ofs_path(dijkstra_filename);
    for(int y = 0; y < divide_num_[Y]; y++)
    {
      for(int x = 0; x < divide_num_[X]; x++)
      {
        int lower_cost = planner_->getlowerboundoncostfromstart_inmm(x, y);
        if(lower_cost == INFINITECOST)
        {
          // unreachable grid
          ofs_path << -1 << " ";
        }
        else
        {
          ofs_path << lower_cost << " ";
        }
      }
      ofs_path << std::endl;
    }
    std::stringstream ss;
    ss << "Run the following commands in Python to visualize Dijkstra distance field:\n"
                    << "  import numpy as np\n"
                    << "  import matplotlib.pyplot as plt\n"
                    << "  dist_field = np.loadtxt(\"" << dijkstra_filename << "\")\n"
                    << "  plt.imshow(dist_field, origin=\"lower\", interpolation=\"none\")\n"
                    << "  plt.show()";
    BFP_INFO_STREAM(ss.str());
  }
}

int FootstepDijkstraPathHeuristic::calcHeuristic(int from_id, int to_id)
{
  if(goal_id_ != to_id)
  {
    BFP_INFO("[FootstepDijkstraPathHeuristic] Goal changed. Need to reset FootstepDijkstraPathHeuristic.");
  }

  const std::shared_ptr<FootstepState> & from_state = env_->id_to_state_list_[from_id];
  const std::shared_ptr<FootstepState> & to_state = env_->id_to_state_list_[to_id];

  std::array<int, 2> gxy;
  try
  {
    gxy = stateIdToGrid(from_id);
  }
  catch(const std::runtime_error & e)
  {
    BFP_INFO_THROTTLE(1,
                      "[FootstepDijkstraPathHeuristic] The heuristic of an out-of-map state was requested: (%lf, %lf)",
                      env_->discToContXy(from_state->x_), env_->discToContXy(from_state->y_));
    constexpr int out_of_map_heuristic = 100000000;
    return out_of_map_heuristic;
  }

  double dist_xy = 1e-3 * planner_->getlowerboundoncostfromstart_inmm(gxy[X], gxy[Y]); // [m]
  double dist_theta =
      env_->discToContTheta(from_state->calcDistanceTheta(to_state, env_->config_->theta_divide_num)); // [rad]
  int step_num =
      static_cast<int>(std::ceil(std::max((dist_xy / env_->step_xy_max_), (dist_theta / env_->step_theta_max_))));

  return static_cast<int>(
      env_->config_->cost_scale
      * (dist_xy + env_->config_->cost_theta_scale * dist_theta + env_->config_->step_cost * step_num));
}

std::array<int, 2> FootstepDijkstraPathHeuristic::contToGrid(std::array<double, 2> cxy) const
{
  int gx = static_cast<int>(std::floor((cxy[X] - min_[X]) / divide_step_));
  int gy = static_cast<int>(std::floor((cxy[Y] - min_[Y]) / divide_step_));
  return std::array<int, 2>{gx, gy};
}

std::array<double, 2> FootstepDijkstraPathHeuristic::gridToCont(std::array<int, 2> gxy) const
{
  double cx = (gxy[X] + 0.5) * divide_step_ + min_[X];
  double cy = (gxy[Y] + 0.5) * divide_step_ + min_[Y];
  return std::array<double, 2>{cx, cy};
}

std::array<int, 2> FootstepDijkstraPathHeuristic::stateIdToGrid(int state_id) const
{
  const std::shared_ptr<FootstepState> & state = env_->id_to_state_list_[state_id];
  std::array<double, 2> cxy = {env_->discToContXy(state->x_), env_->discToContXy(state->y_)};
  std::array<int, 2> gxy = contToGrid(cxy);

  if((gxy[X] < 0) || (gxy[X] >= divide_num_[X]) || (gxy[Y] < 0) || (gxy[Y] >= divide_num_[Y]))
  {
    throw std::runtime_error("Invalid grid position.");
  }

  return gxy;
}
