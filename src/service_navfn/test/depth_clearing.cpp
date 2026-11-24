// SPDX-License-Identifier: Apache-2.0
// Native Humble regression: ground returns clear old depth without marking ground.
#include <iostream>
#include "nav2_costmap_2d/voxel_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
void require(bool value, const char * text) {if (!value) {throw std::runtime_error(text);}}
class Probe : public nav2_costmap_2d::VoxelLayer
{
public:
  void ingest(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    for (auto buffer : observation_buffers_) {
      buffer->lock();buffer->bufferCloud(cloud);buffer->unlock();
    }
  }
};
void check(bool separate)
{
  auto node=std::make_shared<nav2_util::LifecycleNode>(separate ? "split_clear_test" : "legacy_clear_test");
  node->declare_parameter("track_unknown_space",true);
  node->declare_parameter("transform_tolerance",0.);  // Static, synchronous fixture TF.
  node->declare_parameter("depth.footprint_clearing_enabled",false);
  node->declare_parameter("depth.origin_z",0.);
  node->declare_parameter("depth.z_resolution",.1);
  node->declare_parameter("depth.z_voxels",16);
  node->declare_parameter("depth.mark_threshold",0);
  node->declare_parameter("depth.observation_sources",separate ? "mark clear" : "mark");
  for (const auto & name : (separate ? std::vector<std::string>{"mark","clear"} : std::vector<std::string>{"mark"})) {
    const auto prefix="depth."+name+".";
    node->declare_parameter(prefix+"topic","/native_clearing_test/unused");
    node->declare_parameter(prefix+"data_type","PointCloud2");
    node->declare_parameter(prefix+"sensor_frame","sensor_origin");
    node->declare_parameter(prefix+"min_obstacle_height",name=="clear" ? -.05 : .08);
    node->declare_parameter(prefix+"max_obstacle_height",1.1);
    node->declare_parameter(prefix+"marking",name=="mark");
    node->declare_parameter(prefix+"clearing",!separate || name=="clear");
  }
  nav2_costmap_2d::LayeredCostmap parent("map",false,true);parent.resizeMap(50,50,.1,0.,0.);
  tf2_ros::Buffer tf(node->get_clock());
  // Fixture supplies every static transform before ingesting, so no wait on new TF.
  tf.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped origin;
  origin.header.frame_id="map";origin.child_frame_id="sensor_origin";origin.transform.rotation.w=1.;
  origin.transform.translation.x=.5;origin.transform.translation.y=1.05;origin.transform.translation.z=1.;
  tf.setTransform(origin,"test",true);
  Probe layer;layer.initialize(&parent,"depth",&tf,node,nullptr);
  auto feed=[&](float x,float z) {
    sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id="map";cloud.header.stamp=node->now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> cx(cloud,"x"),cy(cloud,"y"),cz(cloud,"z");*cx=x;*cy=1.05f;*cz=z;
    layer.ingest(cloud);double a=1e30,b=1e30,c=-1e30,d=-1e30;layer.updateBounds(.5,1.05,0.,&a,&b,&c,&d);
  };
  feed(1.55f,.15f);require(layer.getCost(15,10)==254,"Visible low obstacle must be marked");
  feed(1.0f,0.f);require(layer.getCost(15,10)==254,"Unobserved space must not expire or be cleared");
  feed(1.70f,0.f);  // After native endpoint shortening, the ray crosses voxel(15,10,1).
  require(layer.getCost(15,10)==(separate ? 0 : 254),"Ground ray must clear only with separate clearing source");
  require(layer.getCost(25,10)==255,"Space beyond observed endpoint stays unknown");
  auto master=parent.getCostmap();master->setCost(15,10,254);layer.updateCosts(*master,0,0,50,50);
  require(master->getCost(15,10)==254,"Sensor clearing must not erase static master occupancy");
  feed(1.55f,.15f);require(layer.getCost(15,10)==254,"Reappearing real obstacle must be marked again");
  std::cout << (separate ? "split" : "legacy") << " marking/occlusion/floor/unknown/static/reappearance passed\n";
}
int main(int argc,char **argv) {rclcpp::init(argc,argv);check(false);check(true);rclcpp::shutdown();}
