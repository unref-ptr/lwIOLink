# SPDX-License-Identifier: GPL-3.0+
# SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

cmake_minimum_required(VERSION 3.14)
project(lwIOLink_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch CppUTest from GitHub
include(FetchContent)
set(CPPUTEST_BUILD_TESTS OFF CACHE BOOL "Disable CppUTest self-tests")
set(CPPUTEST_USE_MEM_LEAK_DETECTION OFF CACHE BOOL "Disable memory leak detection")
set(COMPILE_AND_RUN_CPPUTEST_TESTS     OFF CACHE BOOL "" FORCE)
set(COMPILE_AND_RUN_CPPUTESTEXT_TESTS  OFF CACHE BOOL "" FORCE)
set(CPPUTEST_BUILD_EXAMPLES            OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    CppUTest
    GIT_REPOSITORY https://github.com/cpputest/cpputest.git
    GIT_TAG v4.0
)

FetchContent_MakeAvailable(CppUTest)

# Get absolute paths
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
get_filename_component(SRC_DIR "${PROJECT_ROOT_DIR}/src" ABSOLUTE)
get_filename_component(MOCK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mock" ABSOLUTE)
get_filename_component(TESTS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src" ABSOLUTE)

# Create a test-specific version of the library
add_library(lwIOLink_test_lib
    ${SRC_DIR}/lwIOLink.cpp
    ${SRC_DIR}/lwIOLink.hpp
    ${MOCK_DIR}/Arduino.cpp
    ${MOCK_DIR}/master/FrameBuilder.cpp
)

# Define test mode for conditional compilation
target_compile_definitions(lwIOLink_test_lib PRIVATE UNIT_TEST)

# Include directories for test library
target_include_directories(lwIOLink_test_lib PUBLIC
    ${SRC_DIR}
    ${MOCK_DIR}  # For our mock directory
    ${MOCK_DIR}/master  # For master utilities
)

# Create test executable
add_executable(lwIOLink_tests
    ${TESTS_SRC_DIR}/test_main.cpp
    ${TESTS_SRC_DIR}/test_basic.cpp
    ${TESTS_SRC_DIR}/test_communication.cpp
)

# Link against our test library and CppUTest
target_link_libraries(lwIOLink_tests
    lwIOLink_test_lib
    CppUTest
    CppUTestExt
)

# Include directories
target_include_directories(lwIOLink_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SRC_DIR}
    ${MOCK_DIR}
    ${TESTS_SRC_DIR}
)

# Enable testing
enable_testing()

# Add test to CTest
add_test(NAME lwIOLink_unit_tests COMMAND lwIOLink_tests)
