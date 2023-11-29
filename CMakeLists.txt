cmake_minimum_required(VERSION 3.8)
project(hikrobot_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# include_directories(${PROJECT_SOURCE_DIR}/include)     
include_directories(/opt/MVS/include)
# include_directories(/opt/ros/humble/include/cv_bridge) # you may need this
link_directories(/opt/MVS/lib/aarch64)
link_libraries(MvCameraControl)

#ament_auto_add_executable(MvCameraPub src/MvCameraPub.cpp)
add_executable(MvCameraPub
  src/MvCameraPub.cpp)
target_link_libraries(MvCameraPub MvCameraControl)
ament_target_dependencies(MvCameraPub
 rclcpp
 sensor_msgs
 cv_bridge
 )

install(TARGETS MvCameraPub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
# ament_auto_package()
