#ifndef ROVER_PERSISTENT_TRAVERSABILITY_MAP__PERSISTENT_TRAVERSABILITY_MAP_NODE_HPP_
#define ROVER_PERSISTENT_TRAVERSABILITY_MAP__PERSISTENT_TRAVERSABILITY_MAP_NODE_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace rover_persistent_traversability_map
{

class PersistentTraversabilityMapNode : public rclcpp::Node
{
public:
  explicit PersistentTraversabilityMapNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void loadParameters();
  void initializePersistentMap();
  void setupRosInterfaces();

  void inputMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publishMap();

  bool worldToPersistentCell(
    double x,
    double y,
    int & cell_x,
    int & cell_y) const;

  bool getTransformToMapFrame(
    const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform) const;

  void transformPoint2D(
    double x_in,
    double y_in,
    const geometry_msgs::msg::TransformStamped & transform,
    double & x_out,
    double & y_out) const;

  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;

  int persistentIndex(int x, int y) const;
  bool isInsidePersistentMap(int x, int y) const;

private:
  std::string input_map_topic_;
  std::string output_map_topic_;
  std::string map_frame_;

  double resolution_;
  double width_m_;
  double height_m_;
  double origin_x_;
  double origin_y_;
  double publish_rate_;

  int lethal_threshold_;
  int free_threshold_;

  int occupied_value_;
  int free_value_;
  int unknown_value_;

  int width_cells_;
  int height_cells_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr input_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  mutable std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid persistent_map_;
};

}  // namespace rover_persistent_traversability_map

#endif  // ROVER_PERSISTENT_TRAVERSABILITY_MAP__PERSISTENT_TRAVERSABILITY_MAP_NODE_HPP_
