cmake_minimum_required(VERSION 3.8)
project(ddg_waypoint_follower)

set(lib_name ${PROJECT_NAME}_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ddg_multi_robot_srvs REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  ddg_multi_robot_srvs
  )

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

add_executable(${lib_name} src/ddg_waypoint_follower.cpp)

ament_target_dependencies(${lib_name}
  ${dependencies}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

target_include_directories(${lib_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${lib_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${lib_name}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
