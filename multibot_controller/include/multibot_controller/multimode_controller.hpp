#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

// Custom service để switch controllet
#include "multibot_interfaces/srv/switch_controllet.hpp"
// Service de set diem den + msg vung tac nghen
#include "multibot_interfaces/srv/set_goal.hpp"
#include <multibot_interfaces/srv/set_route.hpp>
#include "multibot_interfaces/msg/congestion_zones.hpp"

#include "multibot_controller/controllet_base.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace multibot_controller
{

/**
 * MultimodeController Node
 *
 * - Quản lý N robots, mỗi robot có 1 controllet đang active
 * - Loop ~50Hz: lấy pose SLAM (map -> base_link) → gọi controllet.compute()
 *   → publish cmd_vel. Wheel odometry chỉ là fallback an toàn khi SLAM chưa
 *   xuất bản TF trong lúc khởi động.
 * - Expose ROS2 service để switch controllet bất kỳ lúc nào (~2ms delay)
 *
 * Kiến trúc adapt từ multipanda_ros2 (Škerlj et al., 2026)
 */
class MultimodeControllerNode : public rclcpp::Node
{
public:
  explicit MultimodeControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  // ── Structs ──────────────────────────────────────────────────────────────

  struct RobotContext {
    std::string ns;

    // Tất cả controllets có sẵn cho robot này
    std::unordered_map<std::string, std::shared_ptr<ControlletBase>> controllets;

    // Controllet đang active
    std::shared_ptr<ControlletBase> active_controllet;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    sensor_msgs::msg::LaserScan latest_scan;
    bool scan_received{false};
    // Pose hiện tại trong frame <robot>/map khi SLAM sẵn sàng.
    nav_msgs::msg::Odometry latest_odom;
    bool odom_received{false};
    bool slam_pose_received{false};
  };

  // ── Methods ───────────────────────────────────────────────────────────────

  void init_robots(const std::vector<std::string> & namespaces);

  // Cập nhật latest_odom bằng transform <robot>/map -> <robot>/base_link.
  // Trả về false khi SLAM/TF chưa sẵn sàng; khi đó control loop dùng wheel odom.
  bool update_pose_from_slam(RobotContext & robot);

  void control_loop();

  void switch_controllet_cb(
    const std::shared_ptr<multibot_interfaces::srv::SwitchControllet::Request> req,
    std::shared_ptr<multibot_interfaces::srv::SwitchControllet::Response> res);

  // Service callback: set diem den cho 1 robot, tu dong chuyen sang controllet "goal"
  void set_goal_cb(
    const std::shared_ptr<multibot_interfaces::srv::SetGoal::Request> req,
    std::shared_ptr<multibot_interfaces::srv::SetGoal::Response> res);

  // ── Members ───────────────────────────────────────────────────────────────

  std::vector<RobotContext> robots_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<multibot_interfaces::srv::SwitchControllet>::SharedPtr switch_srv_;

  // Service set goal + subscriber vung tac nghen + cache zone moi nhat
  rclcpp::Service<multibot_interfaces::srv::SetGoal>::SharedPtr set_goal_srv_;

  rclcpp::Service<multibot_interfaces::srv::SetRoute>::SharedPtr set_route_srv_;

  void set_route_cb(
    const std::shared_ptr<multibot_interfaces::srv::SetRoute::Request> req,
    std::shared_ptr<multibot_interfaces::srv::SetRoute::Response> res);
    
  rclcpp::Subscription<multibot_interfaces::msg::CongestionZones>::SharedPtr congestion_sub_;
  std::vector<multibot_interfaces::msg::CongestionZone> current_zones_;

  double control_freq_{50.0};  // Hz
};

}  // namespace multibot_controller
