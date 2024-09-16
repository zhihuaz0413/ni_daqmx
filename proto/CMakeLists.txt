cmake_minimum_required(VERSION 3.5)
project(ni_proto)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
message("current binary_dir: " ${CMAKE_CURRENT_BINARY_DIR})
message("current source_dir: " ${CMAKE_CURRENT_SOURCE_DIR})
set(ProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
    CACHE INTERNAL "Path to generated protobuf files.")
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(ni_proto STATIC ${ProtoSources} ${ProtoHeaders})

target_link_libraries(ni_proto ${Protobuf_LIBRARIES})
