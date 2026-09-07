cmake_minimum_required(VERSION 3.8)
project(multibot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/SwitchControllet.srv"
"srv/SetGoal.srv"
"msg/CongestionZone.msg"
"msg/CongestionZones.msg"
"srv/SetRoute.srv"
"srv/DeclareCongestionZone.srv"
)

ament_package()
