#include "multibot_controller/multimode_controller.hpp"
#include "multibot_controller/goal_controllet.hpp"

#include <chrono>
#include <tf2/exceptions.h>

namespace multibot_controller
{

MultimodeControllerNode::MultimodeControllerNode(const rclcpp::NodeOptions & options)
: Node("multimode_controller", options),
  tf_buffer_(this->get_clock())
{
  declare_parameter<std::vector<std::string>>("robot_namespaces", {"robot1", "robot2"});
  declare_parameter<double>("control_frequency", 50.0);

  auto namespaces = get_parameter("robot_namespaces").as_string_array();
  control_freq_   = get_parameter("control_frequency").as_double();

  // Không tạo listener trong initializer list: Node phải hoàn tất construction
  // trước để listener có executor/context ROS hợp lệ.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  init_robots(namespaces);

  switch_srv_ = create_service<multibot_interfaces::srv::SwitchControllet>(
    "~/switch_controllet",
    std::bind(
      &MultimodeControllerNode::switch_controllet_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  // Service moi: set diem den cho 1 robot cu the
  set_goal_srv_ = create_service<multibot_interfaces::srv::SetGoal>(
    "~/set_goal",
    std::bind(
      &MultimodeControllerNode::set_goal_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  set_route_srv_ = create_service<multibot_interfaces::srv::SetRoute>(
    "~/set_route",
    std::bind(
      &MultimodeControllerNode::set_route_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  // Subscribe danh sach vung tac nghen (transient_local: robot join sau van nhan duoc)
  congestion_sub_ = create_subscription<multibot_interfaces::msg::CongestionZones>(
    "/congestion_zones", rclcpp::QoS(1).transient_local(),
    [this](const multibot_interfaces::msg::CongestionZones::SharedPtr msg) {
      current_zones_ = msg->zones;
      RCLCPP_INFO(get_logger(), "Congestion zones updated: %zu zone(s) active",
        current_zones_.size());
    });

  auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / control_freq_));
  timer_ = create_wall_timer(
    period_ms, std::bind(&MultimodeControllerNode::control_loop, this));

  RCLCPP_INFO(get_logger(),
    "MultimodeController started: %zu robots @ %.0f Hz",
    robots_.size(), control_freq_);
}

void MultimodeControllerNode::init_robots(const std::vector<std::string> & namespaces)
{
  robots_.reserve(namespaces.size());

  for (const auto & ns : namespaces) {
    RobotContext ctx;
    ctx.ns = ns;

    ctx.controllets["goal"]   = std::make_shared<GoalControllet>(ns);

    ctx.active_controllet = ctx.controllets["goal"];
    ctx.active_controllet->set_active(true);
    ctx.active_controllet->on_activate();

    ctx.cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>(
      "/" + ns + "/cmd_vel", 10);

    robots_.push_back(std::move(ctx));
    const size_t idx = robots_.size() - 1;

    // Wheel odometry dùng làm fallback duy nhất lúc SLAM chưa xuất bản map->odom.
    // Không dùng odom_gt để controller chịu được drift giống robot thật.
    robots_[idx].odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "/" + ns + "/odom", 10,
      [this, idx](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robots_[idx].latest_odom   = *msg;
        robots_[idx].odom_received = true;
      });

    robots_[idx].scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "/" + ns + "/scan_fixed", 10,
      [this, idx](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        robots_[idx].latest_scan = *msg;
        robots_[idx].scan_received = true;
      });

    RCLCPP_INFO(get_logger(),
      "  Robot '%s' initialized with controllets: [patrol, stop, goal]; pose source: SLAM TF", ns.c_str());
  }
}

bool MultimodeControllerNode::update_pose_from_slam(RobotContext & robot)
{
  const auto map_frame = robot.ns + "/map";
  const auto base_frame = robot.ns + "/base_link";

  try {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame, base_frame, tf2::TimePointZero);

    robot.latest_odom.header = transform.header;
    robot.latest_odom.header.frame_id = map_frame;
    robot.latest_odom.child_frame_id = base_frame;
    robot.latest_odom.pose.pose.position.x = transform.transform.translation.x;
    robot.latest_odom.pose.pose.position.y = transform.transform.translation.y;
    robot.latest_odom.pose.pose.position.z = transform.transform.translation.z;
    robot.latest_odom.pose.pose.orientation = transform.transform.rotation;
    robot.slam_pose_received = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    robot.slam_pose_received = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for SLAM transform %s -> %s: %s",
      map_frame.c_str(), base_frame.c_str(), ex.what());
    return false;
  }
}

void MultimodeControllerNode::control_loop()
{
  const double dt = 1.0 / control_freq_;

  // Luôn lấy pose map-frame trước khi tính neighbor/APF để mọi vị trí và
  // congestion zone dùng cùng hệ toạ độ đã được SLAM hiệu chỉnh.
  for (auto & robot : robots_) {
    update_pose_from_slam(robot);
  }

  // Vi tri hien tai cua tat ca robot, dung cho tranh va cham lien-robot
  std::vector<NeighborState> all_positions;
  all_positions.reserve(robots_.size());
  for (const auto & r : robots_) {
    all_positions.push_back({
      r.latest_odom.pose.pose.position.x,
      r.latest_odom.pose.pose.position.y});
  }

  for (auto & robot : robots_) {
    if (!robot.active_controllet) continue;

    // Trước khi nhận được TF SLAM đầu tiên, wheel odom là fallback. Nếu cả hai
    // đều chưa có thì publish cmd_vel = 0 thay vì điều khiển từ pose mặc định.
    if (!robot.slam_pose_received && !robot.odom_received) {
      robot.cmd_vel_pub->publish(geometry_msgs::msg::Twist{});
      continue;
    }

    // Neu controllet dang active la GoalControllet, day world-state moi nhat vao
    if (auto gc = std::dynamic_pointer_cast<GoalControllet>(robot.active_controllet)) {
      gc->set_congestion_zones(current_zones_);
      gc->set_neighbors(all_positions);
      if (robot.scan_received) {
        gc->set_scan(robot.latest_scan);
      }
    }

    auto cmd = robot.active_controllet->compute(robot.latest_odom, dt);
    robot.cmd_vel_pub->publish(cmd);
  }
}

void MultimodeControllerNode::switch_controllet_cb(
  const std::shared_ptr<multibot_interfaces::srv::SwitchControllet::Request> req,
  std::shared_ptr<multibot_interfaces::srv::SwitchControllet::Response> res)
{
  for (auto & robot : robots_) {
    if (robot.ns != req->robot_namespace) continue;

    auto it = robot.controllets.find(req->controllet_name);
    if (it == robot.controllets.end()) {
      res->success = false;
      res->message = "Controllet '" + req->controllet_name + "' not found for robot '" + robot.ns + "'";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    if (it->second == robot.active_controllet) {
      res->success = true;
      res->message = "Already active";
      return;
    }

    robot.active_controllet->on_deactivate();
    robot.active_controllet->set_active(false);

    robot.active_controllet = it->second;
    robot.active_controllet->set_active(true);
    robot.active_controllet->on_activate();

    res->success = true;
    res->message = "Switched '" + robot.ns + "' to '" + req->controllet_name + "'";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    return;
  }

  res->success = false;
  res->message = "Robot namespace '" + req->robot_namespace + "' not found";
  RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

void MultimodeControllerNode::set_goal_cb(
  const std::shared_ptr<multibot_interfaces::srv::SetGoal::Request> req,
  std::shared_ptr<multibot_interfaces::srv::SetGoal::Response> res)
{
  for (auto & robot : robots_) {
    if (robot.ns != req->robot_namespace) continue;

    auto it = robot.controllets.find("goal");
    if (it == robot.controllets.end()) {
      res->success = false;
      res->message = "Robot '" + robot.ns + "' has no 'goal' controllet";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    auto gc = std::dynamic_pointer_cast<GoalControllet>(it->second);
    gc->set_goal(req->x, req->y);

    // Tu dong chuyen sang controllet "goal" neu chua active
    if (robot.active_controllet != it->second) {
      robot.active_controllet->on_deactivate();
      robot.active_controllet->set_active(false);
      robot.active_controllet = it->second;
      robot.active_controllet->set_active(true);
      robot.active_controllet->on_activate();
    }

    res->success = true;
    res->message = "Robot '" + robot.ns + "' goal set to (" +
      std::to_string(req->x) + ", " + std::to_string(req->y) + ")";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    return;
  }

  res->success = false;
  res->message = "Robot namespace '" + req->robot_namespace + "' not found";
  RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

void MultimodeControllerNode::set_route_cb(
  const std::shared_ptr<multibot_interfaces::srv::SetRoute::Request> req,
  std::shared_ptr<multibot_interfaces::srv::SetRoute::Response> res)
{
  for (auto & robot : robots_) {
    if (robot.ns != req->robot_namespace) continue;

    auto it = robot.controllets.find("goal");
    if (it == robot.controllets.end()) {
      res->success = false;
      res->message = "Robot '" + robot.ns + "' has no 'goal' controllet";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    auto gc = std::dynamic_pointer_cast<GoalControllet>(it->second);
    gc->set_route(req->xs, req->ys);

    if (robot.active_controllet != it->second) {
      robot.active_controllet->on_deactivate();
      robot.active_controllet->set_active(false);
      robot.active_controllet = it->second;
      robot.active_controllet->set_active(true);
      robot.active_controllet->on_activate();
    }

    res->success = true;
    res->message = "Robot '" + robot.ns + "' route set with " +
      std::to_string(req->xs.size()) + " waypoint(s)";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    return;
  }

  res->success = false;
  res->message = "Robot namespace '" + req->robot_namespace + "' not found";
  RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

}  // namespace multibot_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(multibot_controller::MultimodeControllerNode)
