#--------------------------------------------------------------------------------------------------
#                                    Dev notes
#--------------------------------------------------------------------------------------------------

# 2023-01-22: Notes on the current Windows-focused CMake build:
# 1- Cilantro is compiled statically (.lib) because for Windows isn't working the CMAkeLIst.txt file
# 2- Open3d is compiled dynamically (.dll) but can also be compiled statically

#--------------------------------------------------------------------------------------------------
#                                    Info project
#--------------------------------------------------------------------------------------------------
# Version and project name
cmake_minimum_required(VERSION 3.12)
project(show_bug LANGUAGES C CXX)

#--------------------------------------------------------------------------------------------------
#                                    Packages
#--------------------------------------------------------------------------------------------------
# Find pre-compiled Open3D
find_package(Open3D CONFIG REQUIRED)
if(Open3D_FOUND)
    message("Open3D_DIR: ${Open3D_DIR}")
    message("Open3D_INCLUDE_DIR: ${Open3D_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Open3d is required to build this project.")
endif()

# Find pre-compiled Cilantro
find_package(Cilantro CONFIG REQUIRED)
if(Cilantro_FOUND)
    message("Cilantro_DIR: ${Cilantro_DIR}")
    message("Cilantro_INCLUDE_DIR: ${Cilantro_INCLUDE_DIR}")
    include_directories(${Cilantro_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Cilantro is required to build this project.")
endif()

# Find header-only Cockroach (no-linking, just include-directory)
find_package(Cockroach CONFIG REQUIRED)
if(Cockroach_FOUND)
    message("Cockroach_DIR: ${Cockroach_DIR}")
    message("Cockroach_INCLUDE_DIR: ${Cockroach_INCLUDE_DIR}")
    include_directories(${Cockroach_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Cockroach is required to build this project.")
endif()

#--------------------------------------------------------------------------------------------------
#                                    Executables/linking
#--------------------------------------------------------------------------------------------------
# Build project .sln
add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE src/main.cpp)

# Target libraries
target_link_libraries(${PROJECT_NAME} PUBLIC ${Cilantro_LIBRARIES}) # static-library
target_link_libraries(${PROJECT_NAME} PRIVATE Open3D::Open3D) # dynamic library

#--------------------------------------------------------------------------------------------------
#                                    Post-build
#--------------------------------------------------------------------------------------------------
# Open3d .dll and cloud transfert to .exe directory only for windows
if(WIN32)
    # On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
    get_target_property(Open3D_type Open3D::Open3D TYPE)
    if(Open3D_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        get_filename_component(Open3D_parent ${Open3D_DIR} DIRECTORY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy
                            "${Open3D_parent}/bin/Open3D.dll"
                            ${PROJECT_BINARY_DIR}/$<CONFIG>)
    endif()
    # Copy also the testing bug point cloud from source directory
    if(EXISTS ${PROJECT_SOURCE_DIR}/cloud/bug.ply)
        message(STATUS "Copying bug.ply testing point cloud to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy
                            "${PROJECT_SOURCE_DIR}/cloud/bug.ply"
                            ${PROJECT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()
