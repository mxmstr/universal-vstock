cmake_minimum_required(VERSION 3.10)
project(VirtualGunstockUI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenVR
find_package(OpenVR REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenVR_INCLUDE_DIRS}
)

# Add source files for the executable
add_executable(VirtualGunstockUI src/main.cpp src/IPCUtils.cpp)

# Link OpenVR
target_link_libraries(VirtualGunstockUI PRIVATE ${OpenVR_LIBRARIES})

# Ensure the executable is placed in a sensible output directory (e.g., build/bin)
set_target_properties(VirtualGunstockUI PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
