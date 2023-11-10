cmake_minimum_required(VERSION 3.8)
project(hikrobot_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED COMPONENTS
# cv_bridge
# image_transport
# rclcpp
# sensor_msgs
# std_msgs  
# OpenCV)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# include_directories(${PROJECT_SOURCE_DIR}/include)     
include_directories(${catkin_INCLUDE_DIRS} /opt/MVS/include)
# include_directories(include ${OpenCV_INCLUDE_DIRS})
link_directories(/opt/MVS/lib/aarch64)
link_libraries(MvCameraControl)


add_executable(MvCameraPub src/MvCameraPub.cpp)
ament_target_dependencies(MvCameraPub rclcpp sensor_msgs image_transport OpenCV cv_bridge std_msgs )
target_link_libraries(MvCameraPub
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  MvCameraPub
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
