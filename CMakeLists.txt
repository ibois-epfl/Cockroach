#=============================================================================
# Part 1:
cmake_minimum_required(VERSION 3.22.1)

project(Cockroach VERSION 2.0.0 LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

#=============================================================================
# Part 2
find_package(Catch2 REQUIRED)
add_executable(hello_test test/test.cpp)
target_link_libraries(hello_test PRIVATE Catch2::Catch2)
target_include_directories(hello_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
include(CTest)
include(Catch)
catch_discover_tests(hello_test)
enable_testing()