#include "multibot_controller/goal_controllet.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <iostream>


namespace multibot_controller
{

GoalControllet::GoalControllet(const std::string & ns)
: ControlletBase(ns)
{
}

void GoalControllet::set_goal(double x, double y)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  waypoints_.clear();       // huy route cu neu co, tranh bi ghi de goal moi
  current_waypoint_ = 0;
  has_bypass_ = false;
  goal_x_ = x;
  goal_y_ = y;
  has_goal_ = true;
  goal_reached_ = false;
}

void GoalControllet::set_congestion_zones(
  const std::vector<multibot_interfaces::msg::CongestionZone> & zones)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  zones_.clear();
  for (const auto & z : zones) {
    if (z.active) zones_.push_back(z);
  }
}

void GoalControllet::set_neighbors(const std::vector<NeighborState> & neighbors)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  neighbors_ = neighbors;
}

void GoalControllet::set_route(const std::vector<double> & xs, const std::vector<double> & ys)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  waypoints_.clear();
  has_bypass_ = false;

  const double step = waypoint_spacing_;  // khoang cach giua cac diem con, m

  std::vector<std::pair<double,double>> control_points;
  for (size_t i = 0; i < xs.size() && i < ys.size(); ++i) {
    control_points.emplace_back(xs[i], ys[i]);
  }

  for (size_t i = 0; i + 1 < control_points.size(); ++i) {
    double x0 = control_points[i].first,   y0 = control_points[i].second;
    double x1 = control_points[i+1].first, y1 = control_points[i+1].second;
    double seg_len = std::hypot(x1 - x0, y1 - y0);

    waypoints_.emplace_back(x0, y0);

    int n_sub = static_cast<int>(seg_len / step);
    for (int k = 1; k <= n_sub; ++k) {
      double t = (step * k) / seg_len;
      if (t >= 1.0) break;
      waypoints_.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
  }
  if (!control_points.empty()) {
    waypoints_.push_back(control_points.back());
  }

  current_waypoint_ = 0;
  if (!waypoints_.empty()) {
    goal_x_ = waypoints_[0].first;
    goal_y_ = waypoints_[0].second;
    has_goal_ = true;
    goal_reached_ = false;
  }
}

void GoalControllet::set_scan(const sensor_msgs::msg::LaserScan & scan)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_scan_ = scan;
  has_scan_ = true;
}

std::pair<double, double> GoalControllet::effective_goal(double gx, double gy) const
{
  // Neu goal khong nam trong vung tac nghen nao -> giu nguyen
  for (const auto & z : zones_) {
    double dx = gx - z.x;
    double dy = gy - z.y;
    double dist = std::hypot(dx, dy);
    if (dist < z.radius + zone_margin_) {
      // Goal bi "tac" -> day ra ngoai bien vung, theo huong tu tam vung ra goal goc
      double push_dist = z.radius + zone_margin_;
      if (dist < 1e-6) {
        // Goal trung tam vung -> chon huong mac dinh (tren xuong)
        return {z.x, z.y - push_dist};
      }
      double ux = dx / dist;
      double uy = dy / dist;
      return {z.x + ux * push_dist, z.y + uy * push_dist};
    }
  }
  return {gx, gy};
}

bool GoalControllet::is_point_in_zone(double x, double y) const
{
  for (const auto & z : zones_) {
    double d = std::hypot(x - z.x, y - z.y);
    if (d < z.radius + zone_margin_) return true;
  }
  return false;
}

size_t GoalControllet::find_next_valid_waypoint(size_t from_idx) const
{
  size_t idx = from_idx;
  while (idx + 1 < waypoints_.size() &&
         is_point_in_zone(waypoints_[idx].first, waypoints_[idx].second)) {
    ++idx;
  }
  return idx;
}

const multibot_interfaces::msg::CongestionZone * GoalControllet::find_blocking_zone(
  double x, double y) const
{
  for (const auto & z : zones_) {
    double d = std::hypot(x - z.x, y - z.y);
    if (d < z.radius + zone_margin_) return &z;
  }
  return nullptr;
}

bool GoalControllet::zone_still_present(double zx, double zy, double zr) const
{
  for (const auto & z : zones_) {
    if (std::hypot(z.x - zx, z.y - zy) < 0.05 && std::abs(z.radius - zr) < 0.05) {
      return true;
    }
  }
  return false;
}

std::pair<double, double> GoalControllet::compute_bypass_point(
  double rx, double ry, double tx, double ty,
  const multibot_interfaces::msg::CongestionZone & zone) const
{
  // Huong di tu vi tri hien tai (rx,ry) toi waypoint dich sau vung (tx,ty)
  double dx = tx - rx;
  double dy = ty - ry;
  double dlen = std::hypot(dx, dy);
  if (dlen < 1e-6) { dx = 1.0; dy = 0.0; dlen = 1.0; }
  double dirx = dx / dlen, diry = dy / dlen;

  // Vector vuong goc voi huong di (quay 90 do nguoc chieu kim dong ho)
  double perpx = -diry, perpy = dirx;

  // Vector tu tam zone toi vi tri robot hien tai
  double vx = rx - zone.x, vy = ry - zone.y;

  // Tich co huong (cross product) giua huong di va vector robot-tam-zone:
  // > 0 nghia la robot dang o phia "trai" cua duong thang di qua tam zone
  // -> chon vong qua ben trai (cung phia robot dang lech) de giam quang duong vong,
  //    tranh robot phai cat ngang sang phia doi dien khong can thiet.
  double cross = dirx * vy - diry * vx;
  double side = (cross >= 0.0) ? 1.0 : -1.0;

  double eff_radius = zone.radius + zone_margin_ + bypass_extra_buffer_;
  double bx = zone.x + side * perpx * eff_radius;
  double by = zone.y + side * perpy * eff_radius;
  return {bx, by};
}

geometry_msgs::msg::Twist GoalControllet::compute(
  const nav_msgs::msg::Odometry & odom, double dt)
{
  (void)dt;
  geometry_msgs::msg::Twist cmd;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_goal_) return cmd;

  const double rx = odom.pose.pose.position.x;
  const double ry = odom.pose.pose.position.y;
  double yaw;
  {
    tf2::Quaternion q(
      odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  // --- Xu ly waypoint + bypass vung tac nghen ---
  if (!waypoints_.empty() && current_waypoint_ < waypoints_.size()) {

    if (has_bypass_) {
      // Dang trong qua trinh vong qua mep 1 vung tac nghen.
      // Chi huy bypass som neu CHINH zone gay chan (theo toa do da luu) bi xoa/tat.
      // LUU Y: truoc day code kiem tra nham vao waypoint DICH (von di la diem
      // NGOAI vung) nen luon tra ve "khong con bi chan" ngay lap tuc, khien
      // bypass bi huy qua som va robot lao thang qua tam vung. Da sua lai
      // kiem tra dung zone gay chan ban dau.
      bool zone_gone = !zone_still_present(bypass_zone_x_, bypass_zone_y_, bypass_zone_radius_);

      double dbx = bypass_x_ - rx;
      double dby = bypass_y_ - ry;
      double dist_to_bypass = std::hypot(dbx, dby);

      if (zone_gone || dist_to_bypass < bypass_reach_radius_) {
        // Da toi diem vong qua (hoac zone da het chan) -> quay lai waypoint that
        has_bypass_ = false;
        current_waypoint_ = bypass_target_waypoint_;
        std::cerr << "[congestion] da vong qua xong, quay lai waypoint "
                  << current_waypoint_ << std::endl;
      } else {
        // Con dang vong -> dung diem bypass lam dich tam thoi
        goal_x_ = bypass_x_;
        goal_y_ = bypass_y_;
      }
    }

    if (!has_bypass_) {
      // Kiem tra doan waypoint sap toi co bi vung tac nghen chan khong
      size_t valid_idx = find_next_valid_waypoint(current_waypoint_);
      if (valid_idx != current_waypoint_) {
        const auto * blocking_zone = find_blocking_zone(
          waypoints_[current_waypoint_].first, waypoints_[current_waypoint_].second);

        if (blocking_zone) {
          // Tinh diem vong qua mep, thay vi nhay thang toi waypoint sau vung
          // (nhay thang de robot lao qua giua vung do luc hut keo truc tiep).
          double tx = waypoints_[valid_idx].first;
          double ty = waypoints_[valid_idx].second;
          auto [bx, by] = compute_bypass_point(rx, ry, tx, ty, *blocking_zone);

          has_bypass_ = true;
          bypass_x_ = bx;
          bypass_y_ = by;
          bypass_zone_x_ = blocking_zone->x;
          bypass_zone_y_ = blocking_zone->y;
          bypass_zone_radius_ = blocking_zone->radius;
          bypass_target_waypoint_ = valid_idx;

          goal_x_ = bx;
          goal_y_ = by;

          std::cerr << "[congestion] waypoint " << current_waypoint_
                    << ".." << (valid_idx - 1)
                    << " bi chan boi zone (" << blocking_zone->x << ", " << blocking_zone->y
                    << "), vong qua diem (" << bx << ", " << by << ")" << std::endl;
        } else {
          // Truong hop hiem: khong tim thay zone cu the (vd zone vua bi xoa
          // giua chung) -> fallback ve hanh vi cu, nhay thang toi waypoint hop le.
          current_waypoint_ = valid_idx;
        }
      }
    }

    if (!has_bypass_) {
      double wx = waypoints_[current_waypoint_].first;
      double wy = waypoints_[current_waypoint_].second;
      double dwx = wx - rx;
      double dwy = wy - ry;
      double dist_to_wp = std::sqrt(dwx*dwx + dwy*dwy);

      if (dist_to_wp < waypoint_reach_radius_ && current_waypoint_ + 1 < waypoints_.size()) {
        ++current_waypoint_;
        std::cerr << "[waypoint] reached, now heading to index "
                  << current_waypoint_ << std::endl;
      }
      goal_x_ = waypoints_[current_waypoint_].first;
      goal_y_ = waypoints_[current_waypoint_].second;
    }
  }

  auto [gx, gy] = effective_goal(goal_x_, goal_y_);

  double dx = gx - rx;
  double dy = gy - ry;
  double dist_to_goal = std::hypot(dx, dy);

  // Chi coi la "reached" khi da toi dung goal GOC (khong phai goal bi day/bypass),
  // vi goal bi day/bypass chi la diem trung gian de vong qua vung tac nghen
  double final_goal_x = waypoints_.empty() ? goal_x_ : waypoints_.back().first;
  double final_goal_y = waypoints_.empty() ? goal_y_ : waypoints_.back().second;
  double orig_dist = std::hypot(final_goal_x - rx, final_goal_y - ry);
  if (!has_bypass_ && orig_dist < goal_tolerance_) {
    goal_reached_ = true;
    return cmd;  // dung lai
  }

  // --- Attractive force ve (gx, gy), giam dan khi gan toi de tranh dao dong ---
  // Neu khong giam dan, robot lao qua waypoint voi luc day du, bi day nguoc lai
  // boi repulsive cua zone/robot khac -> dao dong qua lai khong on dinh.
  double fx = 0.0, fy = 0.0;
  if (dist_to_goal > 1e-6) {
    double linear_ratio = std::min(1.0, dist_to_goal / attract_ramp_radius_);
    double attract_scale = linear_ratio * linear_ratio;  // giam phi tuyen, "let" vao dich chinh xac hon
    fx += k_attract_ * attract_scale * dx / dist_to_goal;
    fy += k_attract_ * attract_scale * dy / dist_to_goal;
  }

  // --- Repulsive force tu cac robot khac (tranh va cham) ---
  for (const auto & n : neighbors_) {
    double ndx = rx - n.x;
    double ndy = ry - n.y;
    double ndist = std::hypot(ndx, ndy);
    if (ndist < robot_repel_radius_ && ndist > 1e-6) {
      double strength = k_repel_robot_ * (1.0 / ndist - 1.0 / robot_repel_radius_) / (ndist * ndist);
      fx += strength * ndx / ndist;
      fy += strength * ndy / ndist;
    }
  }

  // --- Repulsive force tu vung tac nghen (day robot khoi vung, khong chi doi goal) ---
  for (const auto & z : zones_) {
    double zdx = rx - z.x;
    double zdy = ry - z.y;
    double zdist = std::hypot(zdx, zdy);
    double eff_radius = z.radius + zone_margin_;
    if (zdist < eff_radius && zdist > 1e-6) {
      double strength = k_repel_zone_ * (1.0 / zdist - 1.0 / eff_radius) / (zdist * zdist);
      fx += strength * zdx / zdist;
      fy += strength * zdy / zdist;
    }
  }

  // --- Repulsive force tu lidar (ne vat can bat ky: tuong, ke, nguoi...) ---
  double min_obstacle_dist = 1e6;
  if (has_scan_) {
    const auto & scan = latest_scan_;
    for (size_t i = 0; i < scan.ranges.size(); i += 4) {
      float r = scan.ranges[i];
      for (size_t k = i + 1; k < std::min(i + 4, scan.ranges.size()); ++k) {
        float rk = scan.ranges[k];  
        if (std::isfinite(rk) && rk < scan.range_max && rk > scan.range_min && rk < r) {
          r = rk;
        }
      }
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
      if (r < min_obstacle_dist) min_obstacle_dist = r;
      if (r > obstacle_repel_radius_ || r < 1e-3) continue;
      static constexpr double kLidarYawOffset = M_PI / 2.0;
      double angle = scan.angle_min + i * scan.angle_increment;
      double ox = r * std::cos(angle + kLidarYawOffset + yaw);
      double oy = r * std::sin(angle + kLidarYawOffset + yaw);

      double strength = k_repel_obstacle_ *
        (1.0 / r - 1.0 / obstacle_repel_radius_) / r;

      double nx = ox / r, ny = oy / r;
      double tx = -ny, ty = nx;

      double to_goal_x = goal_x_ - rx;
      double to_goal_y = goal_y_ - ry;
      double tangent_sign = (tx * to_goal_x + ty * to_goal_y) >= 0.0 ? 1.0 : -1.0;

      // Trong escape mode: TAT han luc tiep tuyen, chi giu luc day thang ra
      // -> pha vo trang thai can bang xoay quanh vat can
      double tangent_gain = (escape_cooldown_ > 0) ? 0.0 : k_tangent_ratio_;

      fx += -strength * nx + tangent_sign * strength * tangent_gain * tx;
      fy += -strength * ny + tangent_sign * strength * tangent_gain * ty;
    }
  }
  /// --- Lam muot luc tong hop de tranh zigzag ---
  fx_smooth_ = smoothing_alpha_ * fx + (1.0 - smoothing_alpha_) * fx_smooth_;
  fy_smooth_ = smoothing_alpha_ * fy + (1.0 - smoothing_alpha_) * fy_smooth_;

  // --- Chuyen force -> Twist (differential drive) ---
  double desired_yaw = std::atan2(fy_smooth_, fx_smooth_);
  double yaw_err = std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));

  double force_mag = std::hypot(fx_smooth_, fy_smooth_);

  if (force_mag < force_deadband_) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }
  double linear = std::clamp(force_mag * 1.0, 0.0, max_linear_);
  // Giam toc do thang neu can quay gap, nhung khong cat het ve 0 de tranh ket cung
  double turn_factor = std::max(0.3, 1.0 - std::abs(yaw_err) / (M_PI / 2.0));
  linear *= turn_factor;

  double angular_cmd = std::clamp(2.0 * yaw_err, -max_angular_, max_angular_);

  // Phat hien limit cycle: neu angular giu nguyen dau qua lau
  double current_sign = (angular_cmd > 0.1) ? 1.0 : (angular_cmd < -0.1 ? -1.0 : 0.0);
  if (current_sign != 0.0 && current_sign == last_angular_sign_) {
    same_direction_count_++;
  } else {
    same_direction_count_ = 0;
  }
  last_angular_sign_ = current_sign;

  double emergency_margin = 0.15;   // m, chi huy escape neu that su sap dam (rat gan)

  double safety_margin = 0.35;

  if (same_direction_count_ > 25 && escape_cooldown_ <= 0) {
    escape_cooldown_ = 60;
    same_direction_count_ = 0;
  }

  if (escape_cooldown_ > 0) {
    escape_cooldown_--;
    if (min_obstacle_dist > safety_margin) {
      // Du xa vat can -> lai thang ve goal (tangent da bi tat o khoi lidar ben tren)
      double to_goal_x = goal_x_ - rx;
      double to_goal_y = goal_y_ - ry;
      double direct_yaw = std::atan2(to_goal_y, to_goal_x);
      double direct_err = std::atan2(std::sin(direct_yaw - yaw), std::cos(direct_yaw - yaw));
      angular_cmd = std::clamp(2.0 * direct_err, -max_angular_, max_angular_);
    }
    // Neu con qua gan (<= safety_margin): KHONG ghi de angular_cmd,
    // giu nguyen huong tinh tu force field (van co luc day phap tuyen tu lidar,
    // chi mat phan tiep tuyen) -> van duoc day ra khoi vat can an toan.

    if (min_obstacle_dist < obstacle_repel_radius_) {
      linear *= 0.4;
    }
    if (min_obstacle_dist <= emergency_margin) {
      // Cuc ky sat -> huy escape hoan toan, tra ve che do binh thuong ngay
      escape_cooldown_ = 0;
    }
  }
  double max_angular_step = 0.12;   // rad/chu ky (o 50Hz), gioi han toc do doi huong
  double delta = angular_cmd - prev_angular_cmd_;
  delta = std::clamp(delta, -max_angular_step, max_angular_step);
  angular_cmd = prev_angular_cmd_ + delta;
  prev_angular_cmd_ = angular_cmd;

  cmd.linear.x = linear;
  cmd.angular.z = angular_cmd;
  return cmd;
}

}  // namespace multibot_controller
