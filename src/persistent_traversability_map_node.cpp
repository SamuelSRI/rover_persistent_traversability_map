#include "rover_persistent_traversability_map/persistent_traversability_map_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

using namespace std::chrono_literals;

namespace rover_persistent_traversability_map
{

PersistentTraversabilityMapNode::PersistentTraversabilityMapNode(
  const rclcpp::NodeOptions & options)
: Node("persistent_traversability_map_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  declareParameters();
  loadParameters();
  initializePersistentMap();
  setupRosInterfaces();

  RCLCPP_INFO(this->get_logger(), "persistent_traversability_map_node started");
  RCLCPP_INFO(this->get_logger(), "input_map_topic: %s", input_map_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_map_topic: %s", output_map_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "persistent map: %.2f m x %.2f m, resolution %.3f m, %d x %d cells",
    width_m_,
    height_m_,
    resolution_,
    width_cells_,
    height_cells_);
}

void PersistentTraversabilityMapNode::declareParameters()
{
  this->declare_parameter<std::string>("input_map_topic", "/traversability_grid");
  this->declare_parameter<std::string>("output_map_topic", "/persistent_traversability_grid");
  this->declare_parameter<std::string>("map_frame", "map");

  this->declare_parameter<double>("resolution", 0.10);
  this->declare_parameter<double>("width_m", 60.0);
  this->declare_parameter<double>("height_m", 60.0);
  this->declare_parameter<double>("origin_x", -30.0);
  this->declare_parameter<double>("origin_y", -30.0);
  this->declare_parameter<double>("publish_rate", 2.0);

  this->declare_parameter<int>("lethal_threshold", 70);
  this->declare_parameter<int>("free_threshold", 20);

  this->declare_parameter<int>("occupied_value", 100);
  this->declare_parameter<int>("free_value", 0);
  this->declare_parameter<int>("unknown_value", -1);
}

void PersistentTraversabilityMapNode::loadParameters()
{
  input_map_topic_ = this->get_parameter("input_map_topic").as_string();
  output_map_topic_ = this->get_parameter("output_map_topic").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();

  resolution_ = this->get_parameter("resolution").as_double();
  width_m_ = this->get_parameter("width_m").as_double();
  height_m_ = this->get_parameter("height_m").as_double();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  lethal_threshold_ = this->get_parameter("lethal_threshold").as_int();
  free_threshold_ = this->get_parameter("free_threshold").as_int();

  occupied_value_ = this->get_parameter("occupied_value").as_int();
  free_value_ = this->get_parameter("free_value").as_int();
  unknown_value_ = this->get_parameter("unknown_value").as_int();

  if (resolution_ <= 0.0) {
    throw std::runtime_error("resolution must be > 0");
  }

  if (width_m_ <= 0.0 || height_m_ <= 0.0) {
    throw std::runtime_error("width_m and height_m must be > 0");
  }

  if (publish_rate_ <= 0.0) {
    throw std::runtime_error("publish_rate must be > 0");
  }

  width_cells_ = static_cast<int>(std::round(width_m_ / resolution_));
  height_cells_ = static_cast<int>(std::round(height_m_ / resolution_));

  if (width_cells_ <= 0 || height_cells_ <= 0) {
    throw std::runtime_error("invalid persistent map size");
  }
}

void PersistentTraversabilityMapNode::initializePersistentMap()
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  persistent_map_.header.frame_id = map_frame_;

  persistent_map_.info.resolution = static_cast<float>(resolution_);
  persistent_map_.info.width = static_cast<uint32_t>(width_cells_);
  persistent_map_.info.height = static_cast<uint32_t>(height_cells_);

  persistent_map_.info.origin.position.x = origin_x_;
  persistent_map_.info.origin.position.y = origin_y_;
  persistent_map_.info.origin.position.z = 0.0;

  persistent_map_.info.origin.orientation.x = 0.0;
  persistent_map_.info.origin.orientation.y = 0.0;
  persistent_map_.info.origin.orientation.z = 0.0;
  persistent_map_.info.origin.orientation.w = 1.0;

  persistent_map_.data.assign(
    static_cast<size_t>(width_cells_ * height_cells_),
    static_cast<int8_t>(unknown_value_));
}

void PersistentTraversabilityMapNode::setupRosInterfaces()
{
  input_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    input_map_topic_,
    rclcpp::QoS(1).reliable(),
    std::bind(
      &PersistentTraversabilityMapNode::inputMapCallback,
      this,
      std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    output_map_topic_,
    rclcpp::QoS(1).transient_local().reliable());

  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);

  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&PersistentTraversabilityMapNode::publishMap, this));
}

void PersistentTraversabilityMapNode::inputMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (msg->data.empty()) {
    return;
  }

  if (msg->info.resolution <= 0.0F || msg->info.width == 0 || msg->info.height == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Received invalid OccupancyGrid");
    return;
  }

  std::string input_frame = msg->header.frame_id;
  if (input_frame.empty()) {
    input_frame = map_frame_;
  }

  geometry_msgs::msg::TransformStamped input_to_map;
  if (!getTransformToMapFrame(input_frame, input_to_map)) {
    return;
  }

  const double input_resolution = static_cast<double>(msg->info.resolution);
  const double input_origin_x = msg->info.origin.position.x;
  const double input_origin_y = msg->info.origin.position.y;
  const double input_origin_yaw = quaternionToYaw(msg->info.origin.orientation);

  const double cos_origin = std::cos(input_origin_yaw);
  const double sin_origin = std::sin(input_origin_yaw);

  std::lock_guard<std::mutex> lock(map_mutex_);

  for (uint32_t y = 0; y < msg->info.height; ++y) {
    for (uint32_t x = 0; x < msg->info.width; ++x) {
      const size_t input_index =
        static_cast<size_t>(y) * static_cast<size_t>(msg->info.width) + static_cast<size_t>(x);

      if (input_index >= msg->data.size()) {
        continue;
      }

      const int input_value = static_cast<int>(msg->data[input_index]);

      if (input_value == unknown_value_ || input_value < 0) {
        continue;
      }

      const double local_x = (static_cast<double>(x) + 0.5) * input_resolution;
      const double local_y = (static_cast<double>(y) + 0.5) * input_resolution;

      const double point_in_input_frame_x =
        input_origin_x + cos_origin * local_x - sin_origin * local_y;
      const double point_in_input_frame_y =
        input_origin_y + sin_origin * local_x + cos_origin * local_y;

      double point_in_map_x = 0.0;
      double point_in_map_y = 0.0;

      transformPoint2D(
        point_in_input_frame_x,
        point_in_input_frame_y,
        input_to_map,
        point_in_map_x,
        point_in_map_y);

      int persistent_x = 0;
      int persistent_y = 0;

      if (!worldToPersistentCell(point_in_map_x, point_in_map_y, persistent_x, persistent_y)) {
        continue;
      }

      const int index = persistentIndex(persistent_x, persistent_y);

      if (input_value >= lethal_threshold_) {
        persistent_map_.data[static_cast<size_t>(index)] =
          static_cast<int8_t>(occupied_value_);
      } else if (input_value <= free_threshold_) {
        persistent_map_.data[static_cast<size_t>(index)] =
          static_cast<int8_t>(free_value_);
      } else {
        // Intermediate cost: keep the strongest information.
        // If the persistent cell is unknown or lower cost, keep this cost.
        const int current_value =
          static_cast<int>(persistent_map_.data[static_cast<size_t>(index)]);

        if (current_value == unknown_value_ || input_value > current_value) {
          persistent_map_.data[static_cast<size_t>(index)] =
            static_cast<int8_t>(std::clamp(input_value, 0, 100));
        }
      }
    }
  }

  persistent_map_.header.stamp = this->now();
  persistent_map_.info.map_load_time = this->now();
}

bool PersistentTraversabilityMapNode::getTransformToMapFrame(
  const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform) const
{
  if (source_frame == map_frame_) {
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = source_frame;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    return true;
  }

  try {
    transform = tf_buffer_.lookupTransform(
      map_frame_,
      source_frame,
      tf2::TimePointZero,
      100ms);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Failed to transform from '%s' to '%s': %s",
      source_frame.c_str(),
      map_frame_.c_str(),
      e.what());
    return false;
  }
}

void PersistentTraversabilityMapNode::transformPoint2D(
  double x_in,
  double y_in,
  const geometry_msgs::msg::TransformStamped & transform,
  double & x_out,
  double & y_out) const
{
  const double yaw = quaternionToYaw(transform.transform.rotation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  x_out = transform.transform.translation.x + cos_yaw * x_in - sin_yaw * y_in;
  y_out = transform.transform.translation.y + sin_yaw * x_in + cos_yaw * y_in;
}

double PersistentTraversabilityMapNode::quaternionToYaw(
  const geometry_msgs::msg::Quaternion & q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

bool PersistentTraversabilityMapNode::worldToPersistentCell(
  double x,
  double y,
  int & cell_x,
  int & cell_y) const
{
  cell_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  cell_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));

  return isInsidePersistentMap(cell_x, cell_y);
}

bool PersistentTraversabilityMapNode::isInsidePersistentMap(int x, int y) const
{
  return x >= 0 && y >= 0 && x < width_cells_ && y < height_cells_;
}

int PersistentTraversabilityMapNode::persistentIndex(int x, int y) const
{
  return y * width_cells_ + x;
}

void PersistentTraversabilityMapNode::publishMap()
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  persistent_map_.header.stamp = this->now();
  persistent_map_.header.frame_id = map_frame_;

  map_pub_->publish(persistent_map_);
}

}  // namespace rover_persistent_traversability_map
