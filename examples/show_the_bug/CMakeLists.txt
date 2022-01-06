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
find_package(Open3D CONFIG REQUIRED)
message("Open3D_DIR: ${Open3D_DIR}")
message("Open3D_INCLUDE_DIR: ${Open3D_INCLUDE_DIR}")

# # Find Cockroach
find_package(Cockroach CONFIG REQUIRED)
message("Cockroach_DIR: ${Cockroach_DIR}")
message("Cockroach_INCLUDE_DIR: ${Cockroach_INCLUDE_DIR}")
include_directories(${Cockroach_INCLUDE_DIR})


add_executable(${PROJECT_NAME})



target_sources(${PROJECT_NAME} PRIVATE src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE Open3D::Open3D)
target_link_libraries(${PROJECT_NAME} PRIVATE Cockroach::Cockroach)



# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
if(WIN32)
    get_target_property(Open3D_type Open3D::Open3D TYPE)
    if(Open3D_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        get_filename_component(Open3D_parent ${Open3D_DIR} DIRECTORY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy
                            "${Open3D_parent}/bin/Open3D.dll"
                            ${PROJECT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()