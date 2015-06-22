## CMakeList for /hatt_mlhd/ (**TOP LEVEL**)
##
##                  Penn Haptic Texture Toolkit
## Heather Culbertson, Juan Jose Lopez Delgado, Katherine Kuchenbecker
##
## Original toolkit source may be obtained here:
##   repository.upenn.edu/meam_papers/299
##
## Adapted for use on the Magnetically Levitated Haptic Device by
## Randy Lee (ral63@pitt.edu)
################################################################################

cmake_minimum_required(VERSION 2.6)

# Declare project
project(hatt_mlhd CXX C)

# Enable debug symbols and all messages
# if building on a 32 bit machine (recommended)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -pthread")
set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS OFF)

# if building on a 64 bit machine
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -m32 -pthread")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32 -pthread")

message(STATUS "cxx: ${CMAKE_CXX_COMPILER}  ${CMAKE_CXX_FLAGS}")
message(STATUS "c: ${CMAKE_C_COMPILER}  ${CMAKE_C_FLAGS}")

# Change executable output to /cmake_binary/build/ to preserve relative paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/build/")


# Add subdirectories, where source code and headers are held, & application data
add_subdirectory(src)
add_subdirectory(ml)
add_subdirectory(GL)
add_subdirectory(include)
add_subdirectory(lib)
add_subdirectory(images)
add_subdirectory(build)


