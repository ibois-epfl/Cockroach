#--------------------------------------------------------------------------------------------------
#                                    Dev notes
#--------------------------------------------------------------------------------------------------
# 2023-01-22: modules need now to be generate, build and installed seperately. In next version we
# should have this process automatized if the library is missing from the machine
# 2023-01-22: we also require some testing for C++ code

#--------------------------------------------------------------------------------------------------
#                                    Info project
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

#--------------------------------------------------------------------------------------------------
#                                    Third-parties
#--------------------------------------------------------------------------------------------------
# Eigen
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND AND Eigen3_VERSION_MAJOR GREATER_EQUAL "3" AND Eigen3_VERSION_MINOR GREATER "2")
    message(STATUS "Eigen3 version ${Eigen3_VERSION} found in the system, no need to reinstall it")
else()
    message(FATAL_ERROR "Eigen3 not found, please install it seperately for now as indicated in documentation (https://ibois-epfl.github.io/Cockroach-documentation/docs/installation/vanilla-installation/)")
endif()

# Open3D
find_package(Open3D REQUIRED)
if(Open3D_FOUND AND Open3D_VERSION_MAJOR LESS_EQUAL "14")
    message(STATUS "Open3d version ${Open3D_VERSION} found in the system, no need to reinstall it")
else()
    message(FATAL_ERROR "open3d not found, please install it seperately for now as indicated in documentation (https://ibois-epfl.github.io/Cockroach-documentation/docs/installation/vanilla-installation/)")
endif()

# Cilantro
find_package(Cilantro REQUIRED)
if(Cilantro_FOUND)
    message(STATUS "Cilantro found in the system, no need to reinstall it")
else()
    message(FATAL_ERROR "Cilantro not found, please install it seperately for now as indicated in documentation (https://ibois-epfl.github.io/Cockroach-documentation/docs/installation/vanilla-installation/)")
endif()

#--------------------------------------------------------------------------------------------------
#                                    Configure
#--------------------------------------------------------------------------------------------------

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
#                                    Test
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
#                                    Install
#--------------------------------------------------------------------------------------------------

# The project is build locally but it needs to be installed in the system to be referenced by exe
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

# Configuration of module/package generation
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
