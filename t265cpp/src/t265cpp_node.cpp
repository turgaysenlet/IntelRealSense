#include <cstdio>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/buffer_core.h"
#include "tf2/convert.h"
#include "realsense/rs_t265.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  // auto t265cpp_node = std::make_shared<realsense::RealSenseNodeFactory>();
  std::shared_ptr<rclcpp::Node> t265cpp_node = rclcpp::Node::make_shared("t265cpp");
  RCLCPP_INFO(t265cpp_node->get_logger(), "RealSenseT265...");
  rclcpp::spin(t265cpp_node);
  rclcpp::shutdown();
  return 0;
}