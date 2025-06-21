cmake_minimum_required(VERSION 3.10)
project(OpenVRDriverExample LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add OpenVR submodule
add_subdirectory(openvr)

# Include directories
include_directories(
    openvr/headers
    include
)

# Source files
file(GLOB DRIVER_SOURCES "src/*.cpp")

# Create the driver library
add_library(driver_example SHARED ${DRIVER_SOURCES})

# Link against OpenVR
target_link_libraries(driver_example PRIVATE openvr_api)

# Define platform-specific properties
if(WIN32)
    set_target_properties(driver_example PROPERTIES SUFFIX ".dll")
    target_compile_definitions(driver_example PRIVATE "WIN32" "_WINDOWS" "CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS")
elseif(LINUX)
    set_target_properties(driver_example PROPERTIES PREFIX "lib" SUFFIX ".so")
    target_compile_options(driver_example PRIVATE -fPIC)
elseif(APPLE)
    set_target_properties(driver_example PROPERTIES SUFFIX ".dylib")
    target_compile_options(driver_example PRIVATE -fPIC)
endif()

# Installation
# This part might need adjustment based on SteamVR's expected driver structure
# and the target operating system.
# For now, we'll just define an install target for the library.
install(TARGETS driver_example
    LIBRARY DESTINATION lib/driver_example
    RUNTIME DESTINATION bin/driver_example
)

# Add a custom build step to copy resources (if any)
# file(COPY resources/ DESTINATION ${CMAKE_BINARY_DIR}/resources)
# install(DIRECTORY resources/ DESTINATION share/driver_example/resources)

# Add a custom build step to copy the driver manifest
# file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR})
# install(FILES driver.vrdrivermanifest DESTINATION share/driver_example)

message(STATUS "OpenVR SDK Path: ${OPENVR_INCLUDE_DIR}")
message(STATUS "Driver sources: ${DRIVER_SOURCES}")
