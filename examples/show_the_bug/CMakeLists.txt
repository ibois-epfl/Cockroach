# CMake min version
cmake_minimum_required(VERSION 3.12)

# Project info
project(show_the_bug LANGUAGES CXX)


# Find Open3D
find_package(Open3D CONFIG REQUIRED)
message("Open3D_DIR: ${Open3D_DIR}")
message("Open3D_INCLUDE_DIR: ${Open3D_INCLUDE_DIR}")

# # Find Cockroach
# find_package(Cockroach CONFIG REQUIRED)
# message("Cockroach_DIR: ${Cockroach_DIR}")
# message("Cockroach_INCLUDE_DIR: ${Cockroach_INCLUDE_DIR}")
# include_directories(${Cockroach_INCLUDE_DIR})



# Create project
add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} Cockroach::Cockroach)
target_link_libraries(${PROJECT_NAME} Open3D::Open3D)

add_custom_target (run
    COMMAND show_the_bug
    DEPENDS show_the_bug
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR})