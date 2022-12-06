cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(string_srv REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosbag2_cpp REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
)

add_library(talker_lib
  src/publisher_member_function.cpp)

ament_target_dependencies(talker_lib rclcpp std_msgs geometry_msgs tf2 tf2_ros std_msgs string_srv)

add_executable(talker src/publisher_member_function.cpp src/pub_main.cpp)
target_link_libraries(talker
talker_lib)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/cpp_pubsub
  DESTINATION include
)

# Install shared library
install(
  TARGETS talker_lib
  EXPORT talker_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_add_gtest(test_cpp test/test.cpp test/main.cpp )
target_link_libraries(test_cpp talker_lib)
ament_target_dependencies(test_cpp geometry_msgs
tf2
tf2_ros
std_msgs 
string_srv)
install(
    TARGETS test_cpp
    DESTINATION lib/${PROJECT_NAME}
  )

ament_package()