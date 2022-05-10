
cmake_minimum_required(VERSION 3.5)
project(ytcg_ground_based)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# further dependencies filled manually
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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

# addidng boost library to chw_static_plan
target_link_libraries(chw_plan_static ${Boost_LIBRARIES})
target_link_libraries(chw_bs_comm ${Boost_LIBRARIES})

# targe dependencies filled manually
ament_target_dependencies(chw_ctl_comm rclcpp std_msgs)
ament_target_dependencies(chw_bs_comm rclcpp std_msgs sensor_msgs cv_bridge)
ament_target_dependencies(chw_plan_static rclcpp std_msgs)
ament_target_dependencies(pos_estimator rclcpp std_msgs nav_msgs geometry_msgs)

install(TARGETS chw_ctl_comm
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS chw_bs_comm
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS chw_plan_static
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pos_estimator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


