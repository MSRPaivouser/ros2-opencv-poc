cmake_minimum_required(VERSION 3.8)
project(poc_percepcao)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher rclcpp sensor_msgs cv_bridge)
target_include_directories(video_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(video_publisher ${OpenCV_LIBS})

add_executable(edge_detector src/edge_detector.cpp)
ament_target_dependencies(edge_detector rclcpp sensor_msgs cv_bridge)
target_include_directories(edge_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(edge_detector ${OpenCV_LIBS})

add_executable(color_detector src/color_detector.cpp)
ament_target_dependencies(color_detector rclcpp sensor_msgs geometry_msgs cv_bridge)
target_include_directories(color_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(color_detector ${OpenCV_LIBS})

add_executable(target_follower src/target_follower.cpp)
ament_target_dependencies(target_follower rclcpp geometry_msgs)

add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator rclcpp sensor_msgs geometry_msgs)

install(TARGETS video_publisher edge_detector color_detector target_follower navigator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
