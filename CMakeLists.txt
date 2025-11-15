cmake_minimum_required(VERSION 3.8)
project(ws_wrapper)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

include(FetchContent)
FetchContent_Declare(
  IXWebSocket 
  GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
  GIT_TAG v11.4.5
  SOURCE_DIR ${IXWebSocket_SOURCE_DIR}
)
FetchContent_MakeAvailable(IXWebSocket)

find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(twist2ws src/twist2ws.cc)
target_link_libraries(twist2ws
  ixwebsocket
)
ament_target_dependencies(twist2ws
  rclcpp
  sensor_msgs
)

install(TARGETS
  twist2ws
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
