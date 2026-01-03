cmake_minimum_required(VERSION 3.8)
project(windowclean_pro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 requis pour les plugins
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)

# Gazebo
# Essayer de trouver gazebo_ros (requis pour les plugins ROS2)
find_package(gazebo_ros QUIET)
if(NOT gazebo_ros_FOUND)
  message(FATAL_ERROR 
    "gazebo_ros package not found!\n"
    "Please install it with: sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs\n"
    "Or run: ./install_dependencies.sh"
  )
endif()

# Mathématiques et SDF
find_package(ignition-math6 QUIET)
if(NOT ignition-math6_FOUND)
  find_package(gz-math7 QUIET)
  if(NOT gz-math7_FOUND)
    message(FATAL_ERROR "Neither ignition-math6 nor gz-math7 found. Please install Gazebo development packages.")
  endif()
endif()

find_package(sdformat13 QUIET)
if(NOT sdformat13_FOUND)
  find_package(sdformat14 QUIET)
  if(NOT sdformat14_FOUND)
    message(FATAL_ERROR "Neither sdformat13 nor sdformat14 found. Please install Gazebo development packages.")
  endif()
endif()

# URDF
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Chemins d'inclusion
include_directories(
  include
  plugins/include
  ${IGNITION-MATH_INCLUDE_DIRS}
)

# ========================================
# PLUGINS GAZEBO CUSTOM
# ========================================

# Plugin capteur ultrason HC-SR04
add_library(ultrasonic_sensor_plugin SHARED
  plugins/src/ultrasonic_sensor_plugin.cpp
)
ament_target_dependencies(ultrasonic_sensor_plugin
  rclcpp
  sensor_msgs
  geometry_msgs
  gazebo_ros
)
target_link_libraries(ultrasonic_sensor_plugin
  ${IGNITION-MATH_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Plugin système ventouse
add_library(vacuum_suction_plugin SHARED
  plugins/src/vacuum_suction_plugin.cpp
)
ament_target_dependencies(vacuum_suction_plugin
  rclcpp
  std_msgs
  gazebo_ros
)
target_link_libraries(vacuum_suction_plugin
  ${IGNITION-MATH_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Plugin IMU MPU6050
add_library(imu_mpu6050_plugin SHARED
  plugins/src/imv_mpu6050_plugin.cpp
  plugins/include/imv_mpu6050_plugin.h
)
ament_target_dependencies(imu_mpu6050_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
)
target_link_libraries(imu_mpu6050_plugin
  ${IGNITION-MATH_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Plugin contrôleur différentiel
add_library(differential_drive_controller_plugin SHARED
  plugins/src/differential_drive_controller_plugin.cpp
)
ament_target_dependencies(differential_drive_controller_plugin
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  gazebo_ros
)
target_link_libraries(differential_drive_controller_plugin
  ${IGNITION-MATH_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Plugin batterie Li-ion
add_library(battery_plugin SHARED
  plugins/src/battery_plugin.cpp
)
ament_target_dependencies(battery_plugin
  rclcpp
  std_msgs
  gazebo_ros
)
target_link_libraries(battery_plugin
  ${IGNITION-MATH_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Plugin encodeurs incrémentaux
add_library(encoder_plugin SHARED
  plugins/src/encoder_plugin.cpp
)
ament_target_dependencies(encoder_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
)
target_link_libraries(encoder_plugin
  ${IGNITION-MATH_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Installation des plugins
install(TARGETS
  ultrasonic_sensor_plugin
  vacuum_suction_plugin
  imu_mpu6050_plugin
  differential_drive_controller_plugin
  battery_plugin
  encoder_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ========================================
# NOEUDS ROS2 PYTHON
# ========================================

# Installation des scripts Python
install(PROGRAMS
  scripts/ultrasonic_sensor_node.py
  scripts/pump_control_node.py
  scripts/cleaning_arm_node.py
  scripts/safety_monitor_node.py
  scripts/navigation_controller_node.py
  scripts/battery_monitor_node.py
  scripts/validation/test_basic_functionality.py
  scripts/validation/test_navigation_pattern.py
  scripts/validation/test_performance_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
# INSTALLATION FICHIERS
# ========================================

# URDF/Xacro
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

# Meshes
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml"
)

# Config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.sdf" PATTERN "*.world"
)

# Documentation
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

