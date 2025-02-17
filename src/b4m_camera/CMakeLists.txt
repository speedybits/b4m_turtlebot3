cmake_minimum_required(VERSION 3.8)
project(b4m_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(b4m_camera_node src/b4m_camera_node.cpp)
target_include_directories(b4m_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(b4m_camera_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  b4m_camera_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)
target_link_libraries(b4m_camera_node ${OpenCV_LIBS})

install(TARGETS b4m_camera_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch and urdf files
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
