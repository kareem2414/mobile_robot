cmake_minimum_required(VERSION 3.8)
project(robot_tools_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_msgs REQUIRED)

include_directories(include)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp tf2_ros geometry_msgs robot_msgs)
install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_tf_kinematics
  DESTINATION lib/${PROJECT_NAME}

)

ament_target_dependencies(simple_tf_kinematics
  robot_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
