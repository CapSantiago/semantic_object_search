// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <limits>
#include <memory>
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/voxel_layer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace service_navfn
{
// Native sensor buffers, raytracing and persistent obstacle/voxel storage use
// odom. Project their occupied cells into the canonical map at each update,
// rather than retaining marks made with different historical AMCL transforms.
// The global StaticLayer, output geometry, inflation and local controller stay
// native. This does not replace real low obstacles with a simplified map.
template<class Native> class OdomMemoryLayer : public Native
{
public:
  void onInitialize() override
  {
    map_frame_ = this->layered_costmap_->getGlobalFrameID();
    if (map_frame_ == "odom" || this->layered_costmap_->isRolling()) {
      throw std::runtime_error("OdomMemoryLayer requires a nonrolling map-frame master");
    }
    memory_context_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      "odom", false, this->layered_costmap_->isTrackingUnknown());
    resizeMemoryContext();
    // Native initialization binds ObservationBuffer/MessageFilter to its
    // context frame. Give it the odom cache context only during initialization
    // and sizing; restore the real master before the plugin can be activated.
    withMemoryContext([&]() {Native::onInitialize();});
    RCLCPP_INFO(this->logger_, "%s: native sensor memory in odom, projected into %s",
      this->name_.c_str(), map_frame_.c_str());
  }

  void matchSize() override
  {
    if (!memory_context_) {Native::matchSize(); return;}
    if (this->layered_costmap_ != memory_context_.get()) {resizeMemoryContext();}
    withMemoryContext([&]() {Native::matchSize();});
  }

  void updateBounds(double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    transform_valid_ = false;
    if (!this->enabled_) {return;}
    auto master = this->layered_costmap_->getCostmap();
    // Rebuild the complete projection, including its former bounds. An AMCL
    // correction must not leave a second copy at the old map position.
    *min_x = std::min(*min_x, master->getOriginX());
    *min_y = std::min(*min_y, master->getOriginY());
    *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
    *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
    try {
      const auto message = this->tf_->lookupTransform("odom", map_frame_, tf2::TimePointZero);
      tf2::fromMsg(message.transform, map_to_memory_);
      for (double cx : {master->getOriginX(), master->getOriginX() + master->getSizeInMetersX()}) {
        for (double cy : {master->getOriginY(), master->getOriginY() + master->getSizeInMetersY()}) {
          const auto p = map_to_memory_ * tf2::Vector3(cx, cy, 0.);
          unsigned int ix, iy;
          if (!this->worldToMap(p.x(), p.y(), ix, iy)) {
            throw std::runtime_error("Map projection exceeds odom memory extent");
          }
        }
      }
      const auto p = map_to_memory_ * tf2::Vector3(x, y, 0.);
      double a = 1e30, b = 1e30, c = -1e30, d = -1e30;
      Native::updateBounds(p.x(), p.y(), yaw + tf2::getYaw(map_to_memory_.getRotation()), &a, &b, &c, &d);
      transform_valid_ = true;
    } catch (const std::exception & error) {
      this->current_ = false;  // Native planner must wait for a current costmap.
      RCLCPP_ERROR_THROTTLE(this->logger_, *this->clock_, 2000, "%s: %s", this->name_.c_str(), error.what());
    }
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*this->getMutex());
    if (!this->enabled_ || !transform_valid_) {return;}
    if (this->footprint_clearing_enabled_) {
      this->setConvexPolygonCost(this->transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
    }
    for (int y = min_j; y < max_j; ++y) {
      for (int x = min_i; x < max_i; ++x) {
        double wx, wy;
        master.mapToWorld(x, y, wx, wy);
        const auto p = map_to_memory_ * tf2::Vector3(wx, wy, 0.);
        unsigned int sx, sy;
        if (this->worldToMap(p.x(), p.y(), sx, sy) &&
          this->getCost(sx, sy) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          master.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
        // Unknown/unobserved/free sensor cells cannot erase static obstacles
        // or turn unknown prior cells into free space. Inflation runs later.
      }
    }
  }

protected:
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> memory_context_;
  std::string map_frame_;
  tf2::Transform map_to_memory_;
  bool transform_valid_{false};

private:
  template<class F> void withMemoryContext(F work)
  {
    auto actual = this->layered_costmap_;
    this->layered_costmap_ = memory_context_.get();
    try {work();} catch (...) {this->layered_costmap_ = actual; throw;}
    this->layered_costmap_ = actual;
  }
  void resizeMemoryContext()
  {
    auto master = this->layered_costmap_->getCostmap();
    // Extra storage outside the canonical map is only for real sensor rays
    // and modest localization corrections, not extra navigable map space.
    const unsigned int pad = static_cast<unsigned int>(std::ceil(4. / master->getResolution()));
    memory_context_->resizeMap(master->getSizeInCellsX() + 2 * pad,
      master->getSizeInCellsY() + 2 * pad, master->getResolution(),
      master->getOriginX() - pad * master->getResolution(),
      master->getOriginY() - pad * master->getResolution());
  }
};
class OdomObstacleLayer : public OdomMemoryLayer<nav2_costmap_2d::ObstacleLayer> {};
class OdomVoxelLayer : public OdomMemoryLayer<nav2_costmap_2d::VoxelLayer> {};
}  // namespace service_navfn
