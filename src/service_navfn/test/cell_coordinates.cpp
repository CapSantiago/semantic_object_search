// SPDX-License-Identifier: Apache-2.0
#include "service_navfn/planner.hpp"
#include <iostream>
#include <stdexcept>

template<class Base> class Harness : public Base
{
public:
  explicit Harness(nav2_costmap_2d::Costmap2D & grid)
  {
    this->costmap_ = &grid;
    this->clock_ = std::make_shared<rclcpp::Clock>();
    this->global_frame_ = "map";
    this->tolerance_ = .5;
    this->allow_unknown_ = false;
    this->use_astar_ = false;
    this->use_final_approach_orientation_ = false;
    this->planner_ = std::make_unique<nav2_navfn_planner::NavFn>(grid.getSizeInCellsX(), grid.getSizeInCellsY());
  }
};
geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";p.pose.position.x = x;p.pose.position.y = y;p.pose.orientation.w = 1.;
  return p;
}
void require(bool value, const char * message) {if (!value) {throw std::runtime_error(message);}}
int main()
{
  nav2_costmap_2d::Costmap2D grid(80, 80, .05, -1., -1., 0);
  for (unsigned y = 0; y < 80; ++y) {grid.setCost(40, y, 254);}
  Harness<nav2_navfn_planner::NavfnPlanner> original(grid);
  Harness<service_navfn::CellCenteredNavfn> fixed(grid);
  const auto start = pose(.25, .25), goal = pose(.976, .326);
  auto old = original.createPlan(start, goal), corrected = fixed.createPlan(start, goal);
  require(!old.poses.empty() && !corrected.poses.empty(), "Both solvers must find a path");
  const auto endpoint = corrected.poses.back().pose.position;
  require(std::hypot(endpoint.x - goal.pose.position.x, endpoint.y - goal.pose.position.y) < 1e-8,
    "Reachable requested endpoint must not move to neighbour cell");
  require(std::hypot(old.poses.back().pose.position.x - goal.pose.position.x,
    old.poses.back().pose.position.y - goal.pose.position.y) > .04, "Fixture must reproduce native round fallback");
  for (const auto & p : corrected.poses) {
    unsigned x, y;
    require(grid.worldToMap(p.pose.position.x, p.pose.position.y, x, y), "Every path point stays in bounds");
    require(grid.getCost(x, y) < 253, "Corrected path must not enter wall");
  }
  require(fixed.createPlan(start, pose(1.025, .326)).poses.back().pose.position.x < 1.,
    "A truly occupied goal still uses explicit tolerance fallback");
  require(fixed.createPlan(start, pose(100., 0.)).poses.empty(), "Off-map rejected");
  require(fixed.createPlan(start, pose(NAN, 0.)).poses.empty(), "Nonfinite rejected");
  require(fixed.createPlan(pose(-.99, -.99), pose(-.98, -.98)).poses.size() > 0,
    "First half cell is valid");
  std::cout << "native_goal_x=" << old.poses.back().pose.position.x << " corrected_goal_x=" << endpoint.x
    << " path_points=" << corrected.poses.size() << " all_checks=passed\n";
}
