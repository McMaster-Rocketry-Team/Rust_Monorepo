cmake_minimum_required(VERSION 3.14)
project(firmware_common_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library target (interface only since it is header-only)
add_library(firmware_common INTERFACE)
target_include_directories(firmware_common INTERFACE include)

# Testing
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
