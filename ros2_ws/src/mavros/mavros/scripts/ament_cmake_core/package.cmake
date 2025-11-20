set(_AMENT_PACKAGE_NAME "mavros")
set(mavros_VERSION "2.12.0")
set(mavros_MAINTAINER "Vladimir Ermakov <vooon341@gmail.com>")
set(mavros_BUILD_DEPENDS "eigen" "mavlink" "geographiclib" "geographiclib-tools" "angles" "diagnostic_updater" "message_filters" "eigen_stl_containers" "libmavconn" "pluginlib" "libconsole-bridge-dev" "tf2_ros" "tf2_eigen" "rclcpp" "rclcpp_components" "rcpputils" "diagnostic_msgs" "geometry_msgs" "mavros_msgs" "nav_msgs" "sensor_msgs" "geographic_msgs" "trajectory_msgs" "std_msgs" "std_srvs")
set(mavros_BUILDTOOL_DEPENDS "ament_cmake" "ament_cmake_python" "eigen3_cmake_module")
set(mavros_BUILD_EXPORT_DEPENDS "eigen" "mavlink" "geographiclib" "geographiclib-tools" "diagnostic_updater" "message_filters" "eigen_stl_containers" "libmavconn" "pluginlib" "libconsole-bridge-dev" "tf2_ros" "tf2_eigen" "rclcpp" "rclcpp_components" "rcpputils" "diagnostic_msgs" "geometry_msgs" "mavros_msgs" "nav_msgs" "sensor_msgs" "geographic_msgs" "trajectory_msgs" "std_msgs" "std_srvs")
set(mavros_BUILDTOOL_EXPORT_DEPENDS "eigen3_cmake_module")
set(mavros_EXEC_DEPENDS "rosidl_default_runtime" "rclpy" "python3-click" "diagnostic_updater" "message_filters" "eigen_stl_containers" "libmavconn" "pluginlib" "libconsole-bridge-dev" "tf2_ros" "tf2_eigen" "rclcpp" "rclcpp_components" "rcpputils" "diagnostic_msgs" "geometry_msgs" "mavros_msgs" "nav_msgs" "sensor_msgs" "geographic_msgs" "trajectory_msgs" "std_msgs" "std_srvs")
set(mavros_TEST_DEPENDS "ament_cmake_gtest" "ament_cmake_gmock" "ament_cmake_pytest" "ament_lint_auto" "ament_lint_common" "gtest" "google-mock")
set(mavros_GROUP_DEPENDS )
set(mavros_MEMBER_OF_GROUPS )
set(mavros_DEPRECATED "")
set(mavros_EXPORT_TAGS)
list(APPEND mavros_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND mavros_EXPORT_TAGS "<mavros plugin=\"mavros_plugins.xml\"/>")
list(APPEND mavros_EXPORT_TAGS "<rosdoc config=\"rosdoc.yaml\"/>")
list(APPEND mavros_EXPORT_TAGS "<rosindex><!-- ROSIndex metadata. Wait until <include> will work -->
      <tags>
        <tag>uav</tag>
        <tag>mav</tag>
        <tag>mavlink</tag>
        <tag>plugin</tag>
        <tag>apm</tag>
        <tag>px4</tag>
      </tags></rosindex>")
