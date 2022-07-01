########################################################################
# REFERENCES AND BASH COMMANDS
########################################################################

#https://github.com/hshikata/udivone/blob/daba4ea493189c9e0118328d5f863f6a9f3cbf2e/superbuild/CMakeLists.txt
#https://github.com/simogasp/eesepDependencies/blob/994c701d29b753eefbe5a39cad38e5f880a31786/CMakeLists.txt
#https://fuchsia.googlesource.com/cobalt/+/refs/tags/v0.1.3/CMakeLists.txt

# bash commands
# cmake -DGET_LIBS=ON -DBUILD_MY_PROJECTS=OFF  -DBUILD_SHARED_LIBS=ON -G "Visual Studio 17 2022" -A x64 .. && cmake --build . --config Release
# cmake -DGET_LIBS=OFF -DBUILD_MY_PROJECTS=ON  -DBUILD_SHARED_LIBS=ON -G "Visual Studio 17 2022" -A x64 .. && cmake --build . --config Release
# cmake -DGET_LIBS=ON -DBUILD_MY_PROJECTS=ON  -DBUILD_SHARED_LIBS=ON -G "Visual Studio 17 2022" -A x64 .. && cmake --build . --config Release
# Release\my_exe


########################################################################
#PROJECT INITIALIZATION
########################################################################
project(superbuild LANGUAGES CXX)
cmake_minimum_required(VERSION 3.21)

if (MSVC)
 message("Petras is talking to you: MSVC Compiler, this line is not needed if MinGW compiler is used")
 #Without this message no .dll file is created, place ir after the project name
 set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

 #This one copies .dlls to project build directory
 set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
 set(ENABLE_EXPORTS ON)
 message(CMAKE_RUNTIME_OUTPUT_DIRECTORY)

endif (MSVC)

set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

########################################################################
# START SUPERBUILD
########################################################################

SET(GET_LIBS "" CACHE STRING "Set option to download dependencies")

cmake_policy(SET CMP0097 NEW)
include(ExternalProject)

set(LIB_DEBUG_SUFFIX "")
if (MSVC)
  set(LIB_DEBUG_SUFFIX "d")
endif ()


########################################################################
# CGAL
########################################################################

SET(GET_LIBS "" CACHE STRING "Set option to download dependencies")
if (GET_LIBS)

ExternalProject_Add(cgal
  URL https://github.com/CGAL/cgal/releases/download/v5.4.1/CGAL-5.4.1-library.zip
  #GIT_REPOSITORY https://github.com/CGAL/cgal.git
  #GIT_TAG v5.4.1
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    #-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  SOURCE_DIR   "${CMAKE_BINARY_DIR}/install/cgal"
  BUILD_ALWAYS "" #do not buld
  INSTALL_COMMAND "" #installer for now is empty
)


ExternalProject_Add(cgal_dependencies
  URL https://github.com/CGAL/cgal/releases/download/v5.4.1/CGAL-5.4.1-win64-auxiliary-libraries-gmp-mpfr.zip
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    #-DCMAKE_INSTALL_PREFIX:PATH="${CMAKE_BINARY_DIR}/install"
  SOURCE_DIR   "${CMAKE_BINARY_DIR}/install/cgal/auxiliary"
    #INSTALL_DIR   "${CMAKE_INSTALL_PREFIX}/install"
  CONFIGURE_COMMAND "" #do not configure
  BUILD_COMMAND "" #do not buld
  INSTALL_COMMAND "" #installer for now is empty
)


########################################################################
# Boost
########################################################################


  if (WIN32)
    set(BOOST_BOOTSTRAP_CMD ${CMAKE_CURRENT_BINARY_DIR}/boost-prefix/src/boost/bootstrap.bat)
    set(BOOST_BUILD_CMD ${CMAKE_CURRENT_BINARY_DIR}/boost-prefix/src/boost/b2.exe)
  elseif (UNIX)
    set(BOOST_BOOTSTRAP_CMD ${CMAKE_CURRENT_BINARY_DIR}/boost-prefix/src/boost/bootstrap.sh)
    set(BOOST_BUILD_CMD ${CMAKE_CURRENT_BINARY_DIR}/boost-prefix/src/boost/b2)
  endif ()

  ExternalProject_Add(boost
    GIT_REPOSITORY https://github.com/boostorg/boost.git
    GIT_TAG boost-1.78.0
    CONFIGURE_COMMAND ${BOOST_BOOTSTRAP_CMD}
    BUILD_COMMAND
      ${BOOST_BUILD_CMD}
      --prefix=${CMAKE_BINARY_DIR}/install/boost
      -sZLIB_LIBRARY_PATH=${CMAKE_BINARY_DIR}/install/boost/lib
      --without-python
      address-model=64
      variant=debug,release
      link=shared
      runtime-link=shared
      threading=multi
      install
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND ""
    #SOURCE_DIR "${CMAKE_BINARY_DIR}/install/boost" #install directory is in build/install/eigen
  )



########################################################################
# EIGEN
########################################################################

  message(AUTHOR_WARNING "GET_LIBS_" ${GET_LIBS})
  ExternalProject_Add(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install/eigen #has not effect
    SOURCE_DIR   "${CMAKE_BINARY_DIR}/install/eigen" #install directory is in build/install/eigen
    BUILD_COMMAND "" #do not build
    INSTALL_COMMAND "" #do not install
  )

########################################################################
# CILANTRO
########################################################################


ExternalProject_Add(cilantro
GIT_REPOSITORY https://github.com/kzampog/cilantro.git
CMAKE_ARGS
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
  -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install/cilantro #has not effect
SOURCE_DIR   "${CMAKE_BINARY_DIR}/install/cilantro" #install directory is in build/install/eigen
#INSTALL_DIR "${CMAKE_BINARY_DIR}/install/cilantro/build" 
BUILD_ALWAYS ON #do not buld
BUILD_IN_SOURCE  TRUE
INSTALL_COMMAND "" #do not install
)

########################################################################
# OPEN3D
########################################################################

#http://www.open3d.org/docs/latest/getting_started.html#c
ExternalProject_Add(open3d

  URL https://github.com/isl-org/Open3D/releases/download/v0.15.1/open3d-devel-windows-amd64-0.15.1.zip
  
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    #-DCMAKE_INSTALL_PREFIX:PATH="${CMAKE_BINARY_DIR}/install"
  SOURCE_DIR   "${CMAKE_BINARY_DIR}/install/open3d"
    #INSTALL_DIR   "${CMAKE_INSTALL_PREFIX}/install"
  CONFIGURE_COMMAND "" #do not configure
  BUILD_COMMAND "" #do not buld
  INSTALL_COMMAND "" #installer for now is empty
)


########################################################################
# COCKROACH
########################################################################

ExternalProject_Add(cockroach
  URL https://github.com/ibois-epfl/Cockroach/releases/download/1.0.0/cockroach_headers.zip
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    #-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  SOURCE_DIR   "${CMAKE_BINARY_DIR}/install/cockroach"
  CONFIGURE_COMMAND "" #do not configure
  BUILD_COMMAND "" #do not buld
  INSTALL_COMMAND "" #installer for now is empty
)

endif ()







########################################################################
#EXECUTABLE THANK LINK HEADER ONLY DIRECTORY
########################################################################

SET(BUILD_MY_PROJECTS "" CACHE STRING "Build Project")
if (BUILD_MY_PROJECTS)
    message(AUTHOR_WARNING "${CMAKE_BINARY_DIR}/install/cockroach/")



    #C:/IBOIS57/_Code/Software/CPP/CMAKE/super_build/example_cockroach/build/install/cockroach/Cockroach.hpp
    #C:/IBOIS57/_Code/Software/CPP/CMAKE/super_build/example_cockroach/build/install/cockroach/

  add_executable(my_exe main_cockroach.cpp)
  #include_directories("${CMAKE_BINARY_DIR}/install/eigen/") #add directory of the EIGEN header-only library 
  #target_include_directories(my_exe PRIVATE "${CMAKE_BINARY_DIR}/install/boost/include/boost-1_78/") #add directory of the EIGEN header-only library 
 
 
  list(APPEND includePath 
  "${CMAKE_BINARY_DIR}/install/boost/include/boost-1_78/"
  "${CMAKE_BINARY_DIR}/install/eigen/"
  "${CMAKE_BINARY_DIR}/install/cgal/include"
  "${CMAKE_BINARY_DIR}/install/cgal/auxiliary/gmp/include"
  "${CMAKE_BINARY_DIR}/install/cilantro/include/"
  "${CMAKE_BINARY_DIR}/install/open3d/CMake/"
  "${CMAKE_BINARY_DIR}/install/open3d/include/"
  "${CMAKE_BINARY_DIR}/install/open3d/include/open3d/pipelines/registration"
  "${CMAKE_BINARY_DIR}/install/cockroach/"
  #"${CMAKE_BINARY_DIR}/install/cockroach/Cockroach"

  )

  target_include_directories(my_exe PRIVATE "$<BUILD_INTERFACE:${includePath}>") #add directory of the EIGEN header-only library 


  #Link libraries, Open3D and Cilantro
  target_link_directories(my_exe PUBLIC "${CMAKE_BINARY_DIR}/install/cilantro/Release")
  #target_link_libraries(my_exe PUBLIC Open3D::Open3D cilantro )#link compiled open3d library

  include( "${CMAKE_BINARY_DIR}/install/open3d/CMake/Open3DConfig.cmake")
  target_link_libraries(my_exe PUBLIC cilantro Open3D::Open3D)#link compiled open3d library



  #Copy Open3D.dll to Release folder

  add_custom_command(TARGET my_exe POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${CMAKE_BINARY_DIR}/install/open3d/bin/Open3D.dll"      # <--this is in-file
        "${CMAKE_BINARY_DIR}/Release")                 # <--this is out-file path

  #install(FILES "${CMAKE_BINARY_DIR}/install/open3d/bin/Open3D.dll" DESTINATION "${CMAKE_BINARY_DIR}/Release")
  

endif ()


#[[

###############################################################################
#EXECUTABLE THANK LINK HEADER ONLY DIRECTORY
###############################################################################
SET(BUILD_MY_PROJECTS "" CACHE STRING "Build Project")
if (BUILD_MY_PROJECTS)
  message(AUTHOR_WARNING "BUILD_MY_PROJECTS_" ${BUILD_MY_PROJECTS})
  message(AUTHOR_WARNING "${CMAKE_BINARY_DIR}/install/cilantro/Release")
  add_executable(my_exe main_cilantro.cpp)


  list(APPEND includePath 
 # "${CMAKE_BINARY_DIR}/install/"
 # "${CMAKE_BINARY_DIR}/install/cilantro/"
 # "${CMAKE_BINARY_DIR}/install/cilantro/Release"
  "${CMAKE_BINARY_DIR}/install/cilantro/include/"
 # "${CMAKE_BINARY_DIR}/install/cilantro/include/cilantro/"
 # "${CMAKE_BINARY_DIR}/install/cilantro/include/cilantro/core"
  "${CMAKE_BINARY_DIR}/install/eigen/"
  )
  # C:/IBOIS57/_Code/Software/CPP/CMAKE/super_build/example_cilantro/build/install/cilantro/include/cilantro/core
  #message(AUTHOR_WARNING "${CMAKE_BINARY_DIR}/install/cilantro/include/cilantro/core")
  #target_include_directories(my_exe PRIVATE "${CMAKE_BINARY_DIR}/install/boost/include/boost-1_78/") #add directory of the EIGEN header-only library 
 
  target_include_directories(my_exe PRIVATE "$<BUILD_INTERFACE:${includePath}>") #add directory of the EIGEN header-only library 
  #cilantro library linking
  target_link_directories(my_exe PUBLIC  "${CMAKE_BINARY_DIR}/install/cilantro/Release")
  target_link_libraries(my_exe PUBLIC cilantro)
endif ()
]]


