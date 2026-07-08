cmake_minimum_required(VERSION 3.8)
project(visioscan)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define driver source file
FILE(GLOB DRIVER_SRC
  "src/driver/*.cpp"
)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  thread
)

# Build
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include/
)

add_executable(visioscan_node src/bea_node.cpp ${DRIVER_SRC})
target_link_libraries(visioscan_node ${ament_cmake_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(visioscan_node 
  rclcpp
  sensor_msgs
)

# install
install(TARGETS
  visioscan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(Boost)
ament_export_include_directories(include)
ament_package()
