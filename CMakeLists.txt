#--------------------------------------------------------------------------------------------------
#                                    Build
#--------------------------------------------------------------------------------------------------
# Cmake minimal version
cmake_minimum_required(VERSION 3.12)

# Project info
message("=========== CMake setup for Cockroach ===========")
project(Cockroach VERSION 1.0.1 LANGUAGES CXX)
set(PROJECT_DESCRIPTION "A header only c++ umbrella library for point cloud procecssing")
set(PROJECT_HOMEPAGE_URL "https://github.com/ibois-epfl/Cockroach")
set(PROJECT_EMAIL "andrea.settimi@epfl.ch")

# Machine's specifics
include(GNUInstallDirs)

# Specify the library as header-only as Interface
add_library(${PROJECT_NAME} INTERFACE)

# Specify compiler's requirements
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

# Specify the header files' directory
target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

#--------------------------------------------------------------------------------------------------
#                                    Third-parties
#--------------------------------------------------------------------------------------------------

# # Option for download all the dependencies of the submodules
# # Download all the submodules and check them
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

# # Check if open3d sub-modules were downloaded
# if(NOT EXISTS "${PROJECT_SOURCE_DIR}/include/third_parties/open3d/CMakeLists.txt")
#     message(FATAL_ERROR "The open3d submodules was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
# endif()

# # Add Open3d sub-directory
# add_subdirectory(include/third_parties/open3d)

# target_include_directories(${PROJECT_NAME}
#     PUBLIC include/third_parties/open3d
# )

# target_link_directories(${PROJECT_NAME}
#     PRIVATE open3d
# )

# Open3D
find_package(Open3D REQUIRED)
if(Open3D_FOUND AND Open3D_VERSION_MAJOR LESS_EQUAL "14")
    message(STATUS "Open3d version ${Open3D_VERSION} found in the system, no need to reinstall it")
else()
    message(FATAL_ERROR "Open3d not found")
    # download open3d submodule, configure, build and install
endif()

# Cilantro


#--------------------------------------------------------------------------------------------------
#                                    Test
#--------------------------------------------------------------------------------------------------

# Missing testing feature (CTest?)

#--------------------------------------------------------------------------------------------------
#                                    Install
#--------------------------------------------------------------------------------------------------

# The project is build locally but it needs to be installed in the system to be referenced by other executables
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Version controlling
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_NAME}ConfigVersion.cmake"
                                VERSION ${PROJECT_VERSION}
                                COMPATIBILITY SameMajorVersion
)

if(NOT INCLUDE_INSTALL_DIR)
  set(INCLUDE_INSTALL_DIR include/Cockroach)
endif()

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  PATH_VARS INCLUDE_INSTALL_DIR
)

# The installation configuration is over and the install can happen
install(EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

# Produce the cmake configuration files to be reused by other CMake projects
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

# All the header files are actually copied
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
