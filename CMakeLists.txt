cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -latomic")
set (CMAKE_CXX_STANDARD 11)

project(SimpleMaus)

# Useful CMake options for Qt projects
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Search desired Qt packages
#find_package(Qt5Core REQUIRED)

# Create a list with all .cpp source files
set( project_sources
   main.cpp
   Kinematics.cpp
   MouseCom.cpp
   MouseCtrl.cpp
   #kc_mouse.cpp
   #rpi_i2c.cpp
   #i2c.hpp
)

add_executable(${PROJECT_NAME} ${project_sources})
#add_executable (first first.cpp)
set_target_properties (${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED TRUE
  CXX_EXTENSIONS FALSE
)
#target_compile_definitions (first PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries (${PROJECT_NAME}
    atomic
    pthread
)
