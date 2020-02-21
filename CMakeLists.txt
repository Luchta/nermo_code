cmake_minimum_required(VERSION 2.6)

#This is where all the settings to compile for the Edison are stored.
INCLUDE(${CMAKE_SOURCE_DIR}/Edison.cmake)
PROJECT(NRP_Mouse C CXX)

#This is for updating the local SDK to the latest version of mraa and upm
#Uncomment this line to update your SDK every time you run CMake (not every time you build your project)
#INCLUDE(${CMAKE_SOURCE_DIR}/updateSDK.cmake)

#This is the name of the compiled program. Change executableName to anything you want.
set(exe_name NRP_Mouse)

#Here I am defining the source and header files of the project. You can just put .cpp and .h files into this folder and they will get compiled.
file(GLOB_RECURSE cpp_files "*.cpp")
file(GLOB_RECURSE h_files "*.h")

set (Foo_INCLUDE_DIRS "")
foreach (_headerFile ${h_files})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)
list(REMOVE_ITEM Foo_INCLUDE_DIRS "nrp_mouse/mouse_hardware/Examples")

include_directories(${Foo_INCLUDE_DIRS})

#Here you can include extra libraries for your application, like libmraa (http://iotdk.intel.com/docs/master/mraa/) for example
set(extra_libraries -lmraa)

#The actual compilation and linkage
add_executable(${exe_name} ${cpp_files} ${h_files})
target_link_libraries(${exe_name} ${extra_libraries})

#This tells Qt which file to copy over to the Edison.
file(RELATIVE_PATH relativeDir ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}/${exe_name})
add_deployment_file(${relativeDir} ${deploymentFolderAbsolutPath})
