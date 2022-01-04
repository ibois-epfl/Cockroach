cmake_minimum_required(VERSION 3.22.1)
project(cli LANGUAGES CXX)

find_package(Cockroach CONFIG REQUIRED)
message("Cockroach_DIR: ${Cockroach_DIR}")
message("Cockroach_INCLUDE_DIR: ${hello_INCLUDE_DIR}")
include_directories(${Cockroach_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} Cockroach::Cockroach)

add_custom_target (run
    COMMAND cli
    DEPENDS cli
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR})