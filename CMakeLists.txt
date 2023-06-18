
cmake_minimum_required(VERSION 3.5)
project(rb5-ground-robot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost 1.67 REQUIRED COMPONENTS
  system
  filesystem
  regex
)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(chw_ctl_comm src/chw_ctl_comm.cpp src/utility.cpp)
add_executable(chw_bs_comm src/chw_bs_comm.cpp src/utility.cpp)
add_executable(chw_plan_static src/chw_plan_static.cpp src/utility.cpp)
add_executable(pos_estimator src/pos_estimator.cpp src/utility.cpp)
add_executable(camera_wrapper src/camera_wrapper.cpp src/utility.cpp)
add_executable(obstacles src/obstacles.cpp src/utility.cpp)

target_include_directories(chw_ctl_comm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(chw_bs_comm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(chw_plan_static PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(pos_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(camera_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(obstacles PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(chw_plan_static ${Boost_LIBRARIES})
target_link_libraries(chw_bs_comm ${Boost_LIBRARIES})

ament_target_dependencies(chw_ctl_comm rclcpp std_msgs)
ament_target_dependencies(chw_bs_comm rclcpp std_msgs sensor_msgs cv_bridge)
ament_target_dependencies(chw_plan_static rclcpp std_msgs)
ament_target_dependencies(pos_estimator rclcpp std_msgs nav_msgs geometry_msgs)
ament_target_dependencies(camera_wrapper rclcpp)
ament_target_dependencies(obstacles rclcpp std_msgs sensor_msgs geometry_msgs message_filters)

install(TARGETS chw_ctl_comm
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS chw_bs_comm
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS chw_plan_static
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pos_estimator
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS camera_wrapper
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS obstacles
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

