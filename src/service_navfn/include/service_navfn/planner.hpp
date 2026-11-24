// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "nav2_navfn_planner/navfn_planner.hpp"

namespace service_navfn
{
// Keep the installed NavFn solver and lifecycle/parameters; only coordinate
// conversion differs from Humble NavfnPlanner's round/grid-line convention.
class CellCenteredNavfn : public nav2_navfn_planner::NavfnPlanner
{
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
};
}  // namespace service_navfn
