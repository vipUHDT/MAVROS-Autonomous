cmake_minimum_required(VERSION 3.5)
project(livox_msgs)

# -------------------------------
# Dependencies
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# -------------------------------
# Message files
# -------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# -------------------------------
# Export dependencies
# -------------------------------
ament_export_dependencies(rosidl_default_runtime)

# -------------------------------
# Package export
# -------------------------------
ament_package()

