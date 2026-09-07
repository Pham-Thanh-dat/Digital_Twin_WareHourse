#pragma once
#include "multibot_controller/controllet_base.hpp"
#include "multibot_interfaces/msg/congestion_zones.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <mutex>
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>
namespace multibot_controller
{
// Vi tri robot khac, dung de day tranh va cham giua cac robot (khong can lidar)
struct NeighborState
{
  double x, y;
};
class GoalControllet : public ControlletBase
{
public:
  explicit GoalControllet(const std::string & ns);
  // Dat diem den moi. Goi lai bat cu luc nao de doi huong.
  void set_goal(double x, double y);
  // Dat 1 lo trinh (route) gom nhieu waypoint. Robot se di tu waypoint hien tai den waypoint tiep theo.
  void set_route(const std::vector<double> & xs, const std::vector<double> & ys);
  // Cap nhat danh sach vung tac nghen dang active (goi truoc moi compute())
  void set_congestion_zones(const std::vector<multibot_interfaces::msg::CongestionZone> & zones);
  // Cap nhat vi tri cac robot khac de tranh va cham (goi truoc moi compute())
  void set_neighbors(const std::vector<NeighborState> & neighbors);
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_scan_{false};
  void set_scan(const sensor_msgs::msg::LaserScan & scan);
  geometry_msgs::msg::Twist compute(const nav_msgs::msg::Odometry & odom, double dt) override;
  void on_activate() override {}
  void on_deactivate() override {}
  std::string name() const override { return "goal"; }
  bool has_goal() const { return has_goal_; }
  bool goal_reached() const { return goal_reached_; }
private:
  // Neu goal nam trong 1 vung tac nghen, day goal ra ngoai ban kinh do
  // theo huong tu tam vung ra goal goc -> tao "duong di moi"
  std::pair<double, double> effective_goal(double gx, double gy) const;

  bool is_point_in_zone(double x, double y) const;
  size_t find_next_valid_waypoint(size_t from_idx) const;

  // Tim zone dau tien (neu co) dang chan diem (x,y). Tra ve nullptr neu khong co.
  const multibot_interfaces::msg::CongestionZone * find_blocking_zone(double x, double y) const;

  // Kiem tra 1 zone cu the (theo toa do tam + ban kinh da luu luc tao bypass)
  // co con ton tai trong danh sach zones_ dang active hay khong. Dung de biet
  // zone gay chan co bi xoa giua chung khong, KHONG dung de kiem tra diem dich
  // (vi diem dich von di la diem NGOAI vung, se luon tra ve false ngay tu dau
  // -> gay huy bypass qua som, la loi da gap phai truoc do).
  bool zone_still_present(double zx, double zy, double zr) const;

  // Tinh 1 diem "vong qua mep" vung tac nghen: nam tren duong tron ban kinh
  // (radius + margin + buffer) quanh tam zone, o phia ma robot dang tu nhien
  // lech ve (dung tich co huong de chon trai/phai), giup robot bam theo mep
  // vung thay vi lao thang qua tam.
  std::pair<double, double> compute_bypass_point(
    double rx, double ry, double tx, double ty,
    const multibot_interfaces::msg::CongestionZone & zone) const;

  double goal_x_{0.0}, goal_y_{0.0};
  bool has_goal_{false};
  bool goal_reached_{false};
  std::vector<std::pair<double,double>> waypoints_;
  size_t current_waypoint_{0};
  double waypoint_reach_radius_ = 0.4;   // m
  double waypoint_spacing_ = 0.05;   // m, khoang cach giua cac waypoint tu dong chia nho
                                       // (giam tu 0.15 -> 0.05 de waypoint day hon, phan ung
                                       // muot hon voi zone/vat can; luu y: khong tu sua duoc
                                       // truong hop duong thang goc cat vao ket cau ke hang,
                                       // chi giup robot cap nhat dich gan hon thuong xuyen hon)

  // --- Bypass state: khi dang vong qua mep 1 vung tac nghen ---
  bool has_bypass_{false};
  double bypass_x_{0.0}, bypass_y_{0.0};
  double bypass_zone_x_{0.0}, bypass_zone_y_{0.0}, bypass_zone_radius_{0.0};  // luu de tra cuu zone con ton tai khong
  size_t bypass_target_waypoint_{0};   // waypoint se quay lai sau khi vong xong
  double bypass_reach_radius_ = 0.35;  // m, coi la da toi diem vong qua
  double bypass_extra_buffer_ = 0.2;   // m, cong them ngoai radius+margin cho chac an toan

  std::vector<multibot_interfaces::msg::CongestionZone> zones_;
  std::vector<NeighborState> neighbors_;
  std::mutex state_mutex_;
  // Potential field tuning
  double last_angular_sign_ = 0.0;
  int same_direction_count_ = 0;
  int escape_cooldown_ = 0;   // >0 nghia la dang trong che do pha limit cycle
  double k_attract_ = 1.0;
  double attract_ramp_radius_ = 0.6;  // m, luc hut giam tuyen tinh trong ban kinh nay
  double k_repel_robot_ = 3.5;
  double robot_repel_radius_ = 0.65;   // m
  double k_repel_zone_ = 4.5;
  double k_repel_obstacle_ = 1.3;
  double obstacle_repel_radius_ = 0.5;   // m
  double zone_margin_ = 0.5;          // m them ngoai radius vung tac nghen
  double goal_tolerance_ = 0.08;      // m
  double max_linear_ = 1.0;           // m/s (da tang toc, cu la 0.2)
  double max_angular_ = 0.8;          // rad/s (da tang toc, cu la 1.0)
  double force_deadband_ = 0.02;   // nguong luc toi thieu de con di chuyen
  double fx_smooth_ = 0.0;
  double fy_smooth_ = 0.0;
  double smoothing_alpha_ = 0.1;   // 0-1, cang nho cang muot nhung phan ung cham hon
  double k_tangent_ratio_ = 0.15;   // ty le luc tiep tuyen so voi luc day, giup luon qua vat can
  double prev_angular_cmd_ = 0.0;
};

}  // namespace multibot_controller
