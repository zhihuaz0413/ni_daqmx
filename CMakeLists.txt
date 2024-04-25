cmake_minimum_required(VERSION 3.5)
project(ni_daqmx)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/include/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib/)
include_directories(${ProtobufIncludePath})
add_executable(ni_daqmx_node src/ni_daqmx_node.cpp src/ni_daqmx.cpp)
ament_target_dependencies(ni_daqmx_node rclcpp std_msgs)
target_link_libraries(ni_daqmx_node ni_proto ${Protobuf_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bin/libstrANSIseq.1.19.a
        nidaqmx
        protobuf::libprotobuf)

install(TARGETS
  ni_daqmx_node
  DESTINATION lib/${PROJECT_NAME})
add_subdirectory(proto)
ament_package()
