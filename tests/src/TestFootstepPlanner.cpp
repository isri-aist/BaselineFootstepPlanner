/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <BaselineFootstepPlanner/FootstepPlanner.h>

TEST(TestFootstepPlanner, Test1)
{
  auto env_config = std::make_shared<BFP::FootstepEnvConfig>();
  // clang-format off
  env_config->r2l_action_cont_list = {
    BFP::FootstepActionCont(0.2, 0, 0),
    BFP::FootstepActionCont(-0.1, 0, 0),
    BFP::FootstepActionCont(0, 0.15, 0),
    BFP::FootstepActionCont(0, -0.05, 0),
    BFP::FootstepActionCont(0, 0, 0.3),
    BFP::FootstepActionCont(0, 0, -0.3)
  };
  // clang-format on
  env_config->r2l_reachable_min = BFP::FootstepActionCont(-0.05, 0.0, -0.2);
  env_config->r2l_reachable_max = BFP::FootstepActionCont(0.1, 0.1, 0.2);
  env_config->rect_obst_list = {BFP::Rect(0.5, 0.0, 0.15, 0.75)};

  auto planner = std::make_shared<BFP::FootstepPlanner>(env_config);

  std::array<double, 3> start_pos = {0.0, 0.0, 0.0};
  std::array<double, 3> goal_pos = {1.0, 0.5, 1.0};
  auto start_left_state = planner->env_->makeStateFromMidpose(start_pos, BFP::Foot::LEFT);
  auto start_right_state = planner->env_->makeStateFromMidpose(start_pos, BFP::Foot::RIGHT);
  auto goal_left_state = planner->env_->makeStateFromMidpose(goal_pos, BFP::Foot::LEFT);
  auto goal_right_state = planner->env_->makeStateFromMidpose(goal_pos, BFP::Foot::RIGHT);
  planner->setStartGoal(start_left_state, start_right_state, goal_left_state, goal_right_state);

  planner->run();

  for(const auto & state : planner->solution_.state_list)
  {
    planner->env_->printState(state);
  }

  EXPECT_TRUE(planner->solution_.is_solved);
  EXPECT_GE(planner->solution_.id_list.size(), 20);
  EXPECT_LE(planner->solution_.id_list.size(), 25);
  // clang-format off
  EXPECT_TRUE((planner->solution_.state_list[0] == start_left_state &&
               planner->solution_.state_list[1] == start_right_state) ||
              (planner->solution_.state_list[0] == start_right_state &&
               planner->solution_.state_list[1] == start_left_state));
  EXPECT_TRUE((planner->solution_.state_list[planner->solution_.id_list.size() - 2] == goal_left_state &&
               planner->solution_.state_list[planner->solution_.id_list.size() - 1] == goal_right_state) ||
              (planner->solution_.state_list[planner->solution_.id_list.size() - 2] == goal_right_state &&
               planner->solution_.state_list[planner->solution_.id_list.size() - 1] == goal_left_state));
  // clang-format on
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
