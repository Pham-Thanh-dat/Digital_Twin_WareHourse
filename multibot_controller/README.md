Hệ thống mô phỏng Nhà kho Thông minh (Digital Twin Warehouse) trên nền tảng ROS 2 và Ignition Gazebo. Dự án hỗ trợ mô phỏng đa robot (10 Robots), bản đồ nhà kho thế hệ mới và hệ thống điều khiển băng tải tự động.

---

## 🛠 Features

* **Bản đồ Nhà kho Mới**: Được sinh tự động từ script `generate_warehouse_layout2.py` với cấu trúc kệ hàng và vị trí băng tải tối ưu.
* **Mô phỏng Đa Robot (10 Robots)**: Tích hợp 10 AGV/AMR hoạt động đồng thời trong môi trường Ignition Gazebo.
* **Điều khiển Băng tải Tự động**: Quản lý vận hành hệ thống băng tải qua node `conveyor_controller.py`.

---

## 📁 Repository Structure

```text
multibot_controller/
├── config/                     # File cấu hình tham số (YAML)
├── include/multibot_controller # Header files C++ cho bộ điều khiển đa robot
├── launch/                     # File launch khởi chạy Ignition Gazebo & Nodes
├── models/                     # Mô hình 3D (URDF/Xacro/SDF) của Robot, Kệ hàng, Băng tải
├── scripts/                    # Mã nguồn Python (generate_warehouse_layout2.py, conveyor_controller.py)
├── src/                        # Mã nguồn C++ (Core algorithms & Navigation)
├── worlds/                     # File môi trường nhà kho thế hệ mới (warehouse_layout2.world)
├── CMakeLists.txt              # Cấu hình build CMake cho ROS 2
└── package.xml                 # Khai báo dependency và thông tin package