# Digital Twin Warehouse 🤖🏭

Hệ thống mô phỏng Nhà kho Thông minh (Digital Twin Warehouse) trên nền tảng ROS 2 (Humble) và Ignition Gazebo. Repository này được thiết kế theo cấu trúc **Monorepo**, chứa toàn bộ các package cần thiết để biên dịch và vận hành hệ thống mô phỏng đa robot (10 AGVs/AMRs) và băng tải tự động.

---

## 📦 Packages in Repository

Repository này bao gồm 2 ROS 2 packages độc lập:

1. **`multibot_interfaces`**: Package định nghĩa các Custom Messages (`.msg`) và Services (`.srv`) phục vụ truyền thông giữa các robot và bộ điều khiển.
2. **`multibot_controller`**: Package chính chứa mã nguồn C++/Python điều khiển robot, thuật toán điều khiển băng tải (`conveyor_controller.py`), file mô hình 3D (URDF/Xacro), launch files và môi trường mô phỏng nhà kho (`warehouse_layout2.world`).

---

## 📁 Repository Structure

```text
Digital_Twin_WareHourse/
├── multibot_interfaces/        # Package 1: Custom Messages & Services
│   ├── msg/
│   ├── srv/
│   ├── CMakeLists.txt
│   └── package.xml
│
└── multibot_controller/        # Package 2: Simulation & Controllers
    ├── config/                 # File cấu hình tham số (YAML)
    ├── launch/                 # File launch khởi chạy Ignition Gazebo & Nodes
    ├── models/                 # Mô hình 3D (URDF/Xacro/SDF)
    ├── scripts/                # Script Python (conveyor_controller.py, layout2 generator)
    ├── src/                    # Mã nguồn C++ bộ điều khiển
    ├── worlds/                 # File môi trường nhà kho (warehouse_layout2.world)
    ├── CMakeLists.txt
    └── package.xml
