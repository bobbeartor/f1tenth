cmake_minimum_required(VERSION 3.8)

project(joy_initializer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sdl2_vendor REQUIRED)
find_package(sdl2_custom REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_input_node src/joy_input_node.cpp)
ament_target_dependencies(joy_input_node rclcpp sensor_msgs)
target_link_libraries(joy_input_node SDL2::SDL2)

install(TARGETS joy_input_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    joy_input_contract_test
    test/test_joy_input_contract.py
  )
endif()

ament_package()
