// SPDX-License-Identifier: Apache-2.0
#include "service_navfn/odom_memory.hpp"
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(service_navfn::OdomObstacleLayer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(service_navfn::OdomVoxelLayer, nav2_costmap_2d::Layer)
