cmake_minimum_required(VERSION 3.8)
project(fcm_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 1. Библиотека плагинов для Costmap (Радиация)

add_library(fcm_costmap_plugins_core SHARED
  src/radiation_layer.cpp
)

target_include_directories(fcm_costmap_plugins_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fcm_costmap_plugins_core
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  std_msgs
  geometry_msgs
)

# Экспортируем XML файл для pluginlib
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

# 2. Плагины для Деревьев Поведения (BT Nodes)
# Узел проверки связи
add_library(is_connection_lost_bt_node SHARED
  src/is_connection_lost.cpp
)
ament_target_dependencies(is_connection_lost_bt_node
  rclcpp
  behaviortree_cpp
  std_msgs
)

# Узел сохранения позиции
add_library(set_base_pose_bt_node SHARED
  src/set_base_pose.cpp
)
ament_target_dependencies(set_base_pose_bt_node
  rclcpp
  behaviortree_cpp
  geometry_msgs
)

install(TARGETS 
  fcm_costmap_plugins_core
  is_connection_lost_bt_node        
  set_base_pose_bt_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(fcm_costmap_plugins_core)
ament_package()
