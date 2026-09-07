#pragma once
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace multibot_controller
{

/**
 * ControlletBase - Abstract base class
 * Mỗi controllet đại diện cho 1 behavior có thể switch được
 * Adapt từ kiến trúc multipanda_ros2 (Škerlj et al., 2026)
 */
class ControlletBase
{
public:
  ControlletBase(const std::string & robot_ns)
  : robot_ns_(robot_ns), active_(false) {}

  virtual ~ControlletBase() = default;

  // Tính toán cmd_vel cho robot này (tương đương τcmd trong bài báo)
  virtual geometry_msgs::msg::Twist compute(
    const nav_msgs::msg::Odometry & odom,
    double dt) = 0;

  // Gọi khi controllet được activate
  virtual void on_activate() {}

  // Gọi khi controllet bị deactivate
  virtual void on_deactivate() {}

  // Tên behavior này
  virtual std::string name() const = 0;

  const std::string & robot_ns() const { return robot_ns_; }
  bool is_active() const { return active_; }
  void set_active(bool v) { active_ = v; }

protected:
  std::string robot_ns_;
  bool active_;
};

}  // namespace multibot_controller