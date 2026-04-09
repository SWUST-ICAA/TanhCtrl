cmake_minimum_required(VERSION 3.8)

project(tanh_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlatTrajectoryReference.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(tanh_ctrl_core
  src/tanh_ctrl.cpp
)

target_link_libraries(tanh_ctrl_core Eigen3::Eigen)
target_include_directories(tanh_ctrl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tanh_ctrl_node_lib
  src/tanh_ctrl_node.cpp
)

target_link_libraries(tanh_ctrl_node_lib
  tanh_ctrl_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(tanh_ctrl_node_lib
  geometry_msgs
  rclcpp
  px4_msgs
  std_msgs
)

add_executable(tanh_ctrl_node
  src/tanh_ctrl_node_main.cpp
)

target_link_libraries(tanh_ctrl_node tanh_ctrl_node_lib)
ament_target_dependencies(tanh_ctrl_node
  rclcpp
)

install(TARGETS tanh_ctrl_core tanh_ctrl_node_lib tanh_ctrl_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/flatness_reference_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(tanh_ctrl_node_topics_test
    test/tanh_ctrl_node_topics_test.cpp
  )
  target_link_libraries(tanh_ctrl_node_topics_test tanh_ctrl_node_lib)
  ament_target_dependencies(tanh_ctrl_node_topics_test
    rclcpp
  )

  ament_add_gtest(tanh_ctrl_core_feedforward_test
    test/tanh_ctrl_core_feedforward_test.cpp
  )
  target_link_libraries(tanh_ctrl_core_feedforward_test tanh_ctrl_core)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
