cmake_minimum_required(VERSION 3.8)

project(tanh_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include(CTest)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlatTrajectoryReference.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(tanh_ctrl_common
  src/common.cpp
)

target_link_libraries(tanh_ctrl_common Eigen3::Eigen)
target_include_directories(tanh_ctrl_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tanh_ctrl_core
  src/tanh_controller.cpp
)

target_link_libraries(tanh_ctrl_core
  Eigen3::Eigen
  tanh_ctrl_common
)
target_include_directories(tanh_ctrl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tanh_ctrl_node_lib
  src/tanh_node.cpp
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
  src/tanh_node_main.cpp
)

target_link_libraries(tanh_ctrl_node tanh_ctrl_node_lib)
ament_target_dependencies(tanh_ctrl_node
  rclcpp
)
target_include_directories(tanh_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS tanh_ctrl_common tanh_ctrl_core tanh_ctrl_node_lib tanh_ctrl_node
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
  add_executable(test_common
    test/test_common.cpp
  )
  target_link_libraries(test_common tanh_ctrl_common)
  target_include_directories(test_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  add_test(
    NAME test_common
    COMMAND test_common
  )

  add_executable(test_tanh_controller
    test/test_tanh_controller.cpp
  )
  target_link_libraries(test_tanh_controller tanh_ctrl_core)
  target_include_directories(test_tanh_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  add_test(
    NAME test_tanh_controller
    COMMAND test_tanh_controller
  )

  ament_add_gtest(test_tanh_node_logic
    test/test_tanh_node_logic.cpp
  )
  target_link_libraries(test_tanh_node_logic tanh_ctrl_node_lib)
  target_include_directories(test_tanh_node_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
