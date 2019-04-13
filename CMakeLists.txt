# @author Akash Patel
# @date April 12, 2019

cmake_minimum_required(VERSION 2.6)

project(krang-qp)

find_package(DART 6.7.0 REQUIRED COMPONENTS utils-urdf)

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

# Include and link to libraries
link_libraries(dart)

# Include the project files
include_directories(include)
include_directories(include/krang-qp)

# Collect the source files
file(GLOB src "src/*.cpp")
list(SORT src)

# Build the library
add_library(${PROJECT_NAME} SHARED ${src})
target_link_libraries(${PROJECT_NAME} nlopt)

## Build scripts

# Install
FILE(GLOB headers "include/krang-qp/*.hpp")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(FILES include/krang-utils.hpp DESTINATION include)
install(FILES ${headers} DESTINATION include/${PROJECT_NAME})

message(STATUS " ")

# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
