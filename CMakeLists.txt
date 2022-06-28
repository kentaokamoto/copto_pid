cmake_minimum_required(VERSION 3.5)
project(copto_pid)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)

add_library(pid_component SHARED src/pid_component.cpp)

target_compile_definitions(
  pid_component
  PRIVATE "COPTO_PID_PID_COMPONENT_BUILDING_DLL"
)

ament_target_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
)

ament_components_registar_nodes(
  pid_component "copto_pid::PIDComponent"
)

add_exectable(copto_pid_node src/copto_pid_node)

target_link_libraries(copto_quat_node quat_component)

install(TARGET copto_pid_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(TARGET pid_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_export_libraries(pid_component)

ament_export_include_directories(include)

ament_package()
