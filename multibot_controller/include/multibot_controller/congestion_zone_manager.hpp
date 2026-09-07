#pragma once
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
#include <mutex>

#include "multibot_interfaces/msg/congestion_zone.hpp"
#include "multibot_interfaces/msg/congestion_zones.hpp"
#include "multibot_interfaces/srv/declare_congestion_zone.hpp"

namespace multibot_controller
{

class CongestionZoneManagerNode : public rclcpp::Node
{
public:
  explicit CongestionZoneManagerNode(const rclcpp::NodeOptions & options);

private:
  void declare_zone_cb(
    const std::shared_ptr<multibot_interfaces::srv::DeclareCongestionZone::Request> req,
    std::shared_ptr<multibot_interfaces::srv::DeclareCongestionZone::Response> res);

  void publish_zones();

  rclcpp::Service<multibot_interfaces::srv::DeclareCongestionZone>::SharedPtr declare_srv_;
  rclcpp::Publisher<multibot_interfaces::msg::CongestionZones>::SharedPtr zones_pub_;

  std::map<std::string, multibot_interfaces::msg::CongestionZone> active_zones_;
  std::mutex zones_mutex_;
};

}  // namespace multibot_controller
