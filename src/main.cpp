#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rover_persistent_traversability_map/persistent_traversability_map_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rover_persistent_traversability_map::PersistentTraversabilityMapNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
