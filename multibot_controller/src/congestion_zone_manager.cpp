#include "multibot_controller/congestion_zone_manager.hpp"

namespace multibot_controller
{

CongestionZoneManagerNode::CongestionZoneManagerNode(const rclcpp::NodeOptions & options)
: Node("congestion_zone_manager", options)
{
  // transient_local: node/robot join sau van nhan duoc danh sach zone hien tai
  zones_pub_ = create_publisher<multibot_interfaces::msg::CongestionZones>(
    "/congestion_zones", rclcpp::QoS(1).transient_local());

  declare_srv_ = create_service<multibot_interfaces::srv::DeclareCongestionZone>(
    "~/declare_zone",
    std::bind(
      &CongestionZoneManagerNode::declare_zone_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "CongestionZoneManager started. Service: ~/declare_zone");
}

void CongestionZoneManagerNode::declare_zone_cb(
  const std::shared_ptr<multibot_interfaces::srv::DeclareCongestionZone::Request> req,
  std::shared_ptr<multibot_interfaces::srv::DeclareCongestionZone::Response> res)
{
  std::lock_guard<std::mutex> lock(zones_mutex_);

  if (req->clear) {
    auto it = active_zones_.find(req->zone_id);
    if (it == active_zones_.end()) {
      res->success = false;
      res->message = "Zone id '" + req->zone_id + "' khong ton tai, khong the xoa";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }
    active_zones_.erase(it);
    res->success = true;
    res->message = "Da xoa zone '" + req->zone_id + "'";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } else {
    multibot_interfaces::msg::CongestionZone z;
    z.x = req->x;
    z.y = req->y;
    z.radius = req->radius;
    z.active = true;
    active_zones_[req->zone_id] = z;

    res->success = true;
    res->message = "Da khai bao zone '" + req->zone_id + "' tai (" +
      std::to_string(req->x) + ", " + std::to_string(req->y) +
      ") ban kinh " + std::to_string(req->radius);
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  publish_zones();
}

void CongestionZoneManagerNode::publish_zones()
{
  multibot_interfaces::msg::CongestionZones msg;
  for (const auto & [id, zone] : active_zones_) {
    msg.zones.push_back(zone);
  }
  zones_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Publish %zu zone(s) active len /congestion_zones", msg.zones.size());
}

}  // namespace multibot_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(multibot_controller::CongestionZoneManagerNode)
