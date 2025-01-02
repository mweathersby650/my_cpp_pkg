cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(example_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(battery src/battery.cpp)
ament_target_dependencies(battery rclcpp my_robot_interfaces)

add_executable(led_panel src/led_panel.cpp)
ament_target_dependencies(led_panel rclcpp my_robot_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(hardware_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)

add_executable(turtle_sim_controller src/turtle_sim_controller.cpp)
ament_target_dependencies(turtle_sim_controller rclcpp my_robot_interfaces)
ament_target_dependencies(turtle_sim_controller rclcpp turtlesim)
ament_target_dependencies(turtle_sim_controller rclcpp turtlesim)
ament_target_dependencies(turtle_sim_controller rclcpp geometry_msgs)

add_executable(turtle_sim_spawner src/turtle_sim_spawn.cpp)
ament_target_dependencies(turtle_sim_spawner rclcpp my_robot_interfaces)
ament_target_dependencies(turtle_sim_spawner rclcpp turtlesim)

add_executable(turtle_sim_window src/turtle_sim_window.cpp)
ament_target_dependencies(turtle_sim_window rclcpp my_robot_interfaces)


add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)


add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)


install(TARGETS
   cpp_node
   robot_news_station
   number_publisher
   hardware_status_publisher
   turtle_sim_spawner
   turtle_sim_window
   turtle_sim_controller
   number_counter
   smartphone
   battery
   led_panel
   add_two_ints_server
   add_two_ints_client_no_oop
   add_two_ints_client
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
