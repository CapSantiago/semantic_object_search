// SPDX-License-Identifier: Apache-2.0
#include "service_navfn/odom_memory.hpp"
#include <iostream>
#include "sensor_msgs/point_cloud2_iterator.hpp"

void require(bool value, const char * message) {if (!value) {throw std::runtime_error(message);}}
template<class Native> class Harness : public service_navfn::OdomMemoryLayer<Native>
{
public:
  std::string memoryFrame() {return this->global_frame_;}
  unsigned int buffers() {return this->observation_buffers_.size();}
  void ingest(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    auto buffer = this->observation_buffers_.at(0);
    buffer->lock();buffer->bufferCloud(cloud);buffer->unlock();
  }
  void forgetBuffers() {this->marking_buffers_.clear(); this->clearing_buffers_.clear();}
};
template<class Native> void check(const std::string & name)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>(name);
  node->declare_parameter("track_unknown_space", true);
  node->declare_parameter("transform_tolerance", .2);
  node->declare_parameter("sensed.observation_sources", "points");
  node->declare_parameter("sensed.points.topic", "/test_cloud");
  node->declare_parameter("sensed.points.data_type", "PointCloud2");
  node->declare_parameter("sensed.points.min_obstacle_height", .02);
  node->declare_parameter("sensed.points.max_obstacle_height", 1.1);
  nav2_costmap_2d::LayeredCostmap parent("map", false, true);
  parent.resizeMap(50, 50, .1, 0., 0.);
  tf2_ros::Buffer buffer(node->get_clock());
  auto transform = geometry_msgs::msg::TransformStamped();
  transform.header.frame_id = "odom";transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.;
  buffer.setTransform(transform, "test", true);
  Harness<Native> layer;
  layer.initialize(&parent, "sensed", &buffer, node, nullptr);
  require(layer.memoryFrame() == "odom", "Native observation frame must be continuous odom");
  require(layer.buffers() == 1, "Native source buffer retained");
  unsigned int sx, sy;
  require(layer.worldToMap(1.05, 1.05, sx, sy), "Memory point in bounds");
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "odom";cloud.header.stamp = node->now();
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> cx(cloud, "x"), cy(cloud, "y"), cz(cloud, "z");
  *cx=1.05f;*cy=1.05f;*cz=.05f;
  layer.ingest(cloud);
  auto master = parent.getCostmap();
  auto update = [&]() {
      double a = 1e30, b = 1e30, c = -1e30, d = -1e30;
      layer.updateBounds(2., 2., 0., &a, &b, &c, &d);
      require(layer.isCurrent(), "Transform and native sources current");
      master->resetMap(0, 0, 50, 50);
      layer.updateCosts(*master, 0, 0, 50, 50);
    };
  update();
  require(master->getCost(10, 10) == 254, "Occupied sensor cell projected");
  require(master->getCost(0, 0) == 255, "Unobserved remains unknown");
  transform.transform.translation.x = .5;
  buffer.setTransform(transform, "test", true);
  update();
  require(master->getCost(5, 10) == 254, "Current localization moves projection");
  require(master->getCost(10, 10) == 255, "Old projection leaves no ghost");
  require(layer.getCost(sx, sy) == 254, "Actual native obstacle memory retained");
  master->setCost(0, 0, 254);
  layer.updateCosts(*master, 0, 0, 50, 50);
  require(master->getCost(0, 0) == 254, "Static obstacle cannot be erased");
  transform.transform.translation.x = 2.;
  transform.transform.rotation.z = std::sin(M_PI/4.);
  transform.transform.rotation.w = std::cos(M_PI/4.);
  buffer.setTransform(transform, "test", true);update();
  require(master->getCost(10, 9) == 254, "Rotation uses current inverse sampling");
  require(master->getCost(5, 10) == 255, "No old translation projection retained");
  layer.forgetBuffers();
  layer.setCost(sx, sy, 0);update();
  require(master->getCost(10, 9) == 255, "Native-cleared obstacle disappears from projection");
  require(master->getSizeInCellsX() == 50 && master->getOriginX() == 0., "Canonical geometry unchanged");
  std::cout << name << " projection/memory/unknown/static/clearing checks passed\n";
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  check<nav2_costmap_2d::ObstacleLayer>("obstacle_memory_test");
  check<nav2_costmap_2d::VoxelLayer>("voxel_memory_test");
  rclcpp::shutdown();
}
