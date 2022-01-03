# # The minimum version for CMake
# cmake_minimum_required(VERSION "3.22.1")
# # Name the project and version
# project(TEST_CMAKE VERSION 1.0)
# # The project-specific files to added to the build.
# add_executable(${PROJECT_NAME} main.cpp)


# # Option for download all the dependencies of the submodules
# # Download all the submodules dependecies
# find_package(Git QUIET)
# if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# # Update submodules as needed
#     option(GIT_SUBMODULE "Check submodules during build" ON)
#     if(GIT_SUBMODULE)
#         message(STATUS "Submodule update")
#         execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
#                         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                         RESULT_VARIABLE GIT_SUBMOD_RESULT)
#         if(NOT GIT_SUBMOD_RESULT EQUAL "0")
#             message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
#         endif()
#     endif()
# endif()

# # Check all the submodules dependecies
# if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/glfw/CMakeLists.txt")
#     message(FATAL_ERROR "The glfw submodules was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
# endif()

# # Add sub-module directories
# add_subdirectory(include/Open3d)
# add_subdirectory(include/Cilantro)

# # Target sub-module's include directories
# target_include_directories(
#     ${PROJECT_NAME}
#     PUBLIC include/Open3d/cpp
#     PUBLIC include/Cilantro)

# # Target sub-module's link
# target_link_directories(
#     ${PROJECT_NAME}
#     PUBLIC include/Open3d/cpp
# )