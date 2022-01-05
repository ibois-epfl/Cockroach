# CMake min version
cmake_minimum_required(VERSION 3.12)

# # Project info
# project(show_the_bug LANGUAGES CXX)

# # # Find Cockroach
# # find_package(Cockroach CONFIG REQUIRED)
# # message("Cockroach_DIR: ${Cockroach_DIR}")
# # message("Cockroach_INCLUDE_DIR: ${Cockroach_INCLUDE_DIR}")
# # include_directories(${Cockroach_INCLUDE_DIR})


# option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
# if(STATIC_WINDOWS_RUNTIME)
#     set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
# else()
#     set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
# endif()

# # Find Open3D
# find_package(Open3D CONFIG REQUIRED)
# message("Open3D_DIR: ${Open3D_DIR}")
# message("Open3D_INCLUDE_DIR: ${Open3D_INCLUDE_DIR}")
# include_directories(PUBLIC ${Open3D_INCLUDE_DIR})


# add_executable(${PROJECT_NAME})
# target_sources(${PROJECT_NAME} PRIVATE src/main.cpp)


# # target_link_libraries(${PROJECT_NAME} PRIVATE Cockroach::Cockroach)
# target_link_libraries(${PROJECT_NAME} PRIVATE Open3D::Open3D)

# # add_custom_target (run
# #     COMMAND show_the_bug
# #     DEPENDS show_the_bug
# #     WORKING_DIRECTORY ${CMAKE_PROJECT_DIR})

#==============================================================================

# # On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
# cmake_minimum_required(VERSION 3.18)

project(show_the_bug LANGUAGES C CXX)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE Open3D::Open3D)

# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
# if(WIN32)
#     get_target_property(open3d_type Open3D::Open3D TYPE)
#     if(open3d_type STREQUAL "SHARED_LIBRARY")
#         message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
#         add_custom_command(TARGET Draw POST_BUILD
#                            COMMAND ${CMAKE_COMMAND} -E copy
#                                    ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
#                                    ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
#     endif()
# endif()