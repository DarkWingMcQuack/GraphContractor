cmake_minimum_required(VERSION 3.9)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#get options
include(cmake/options.cmake)

if(USE_CLANG)
  SET(CMAKE_C_COMPILER    "clang")
  SET(CMAKE_CXX_COMPILER  "clang++")
  SET(CMAKE_AR            "llvm-ar")
  SET(CMAKE_NM            "llvm-nm")
  SET(CMAKE_OBJDUMP       "llvm-objdump")
  SET(CMAKE_RANLIB        "llvm-ranlib")
endif(USE_CLANG)

project(GraphContractor)

#set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

############################
###get external projects
############################
include(cmake/fmt.cmake)
include(cmake/CLI11.cmake)

#setup flags and ccache
include(cmake/flags.cmake)
include(cmake/ccache.cmake)

# needed for multithreading
find_package(Threads)

add_executable(GraphContractor main.cpp)

target_sources(GraphContractor
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/span.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/Graph.hpp


  PRIVATE
  src/Graph.cpp
  )

# make headers available
target_include_directories(GraphContractor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  fmt
  ${CLI11_INCLUDE_DIR})

#link against libarys
target_link_libraries(GraphContractor LINK_PUBLIC
  fmt
  ${CMAKE_THREAD_LIBS_INIT})

#make sure everything is build in the right order
add_dependencies(GraphContractor fmt-project)
add_dependencies(GraphContractor CLI11-project)
