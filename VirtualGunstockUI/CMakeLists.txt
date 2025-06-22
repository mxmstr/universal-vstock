cmake_minimum_required(VERSION 3.10)
project(VirtualGunstockUIProject) # Changed project name slightly to avoid conflict if root is also "VirtualGunstockUI"

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Library for IPCUtils and shared headers ---
# This library will be used by the UI executable and the driver
add_library(VirtualGunstockUI_lib STATIC
    src/IPCUtils.cpp
    # PoseUpdateData.h is header only, but its path needs to be known
)

# Public include directory for VirtualGunstockUI_lib, so other targets linking to it can find:
# - IPCUtils.h
# - PoseUpdateData.h
target_include_directories(VirtualGunstockUI_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# The driver project will link to VirtualGunstockUI_lib to get IPCUtils and PoseUpdateData

# --- UI Executable ---
add_executable(VirtualGunstockUI src/main.cpp)

# UI executable needs to include its own headers and those from the VirtualGunstockUI_lib
target_include_directories(VirtualGunstockUI PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src # For main.cpp if it includes local headers not in 'include'
    # The include path for IPCUtils.h and PoseUpdateData.h will come via VirtualGunstockUI_lib
)

# Link the UI executable against its utility library and OpenVR
# OpenVR::openvr is assumed to be an INTERFACE library target provided by the root CMakeLists.txt
target_link_libraries(VirtualGunstockUI PRIVATE
    VirtualGunstockUI_lib
    OpenVR::openvr
)

# Ensure the executable is placed in a sensible output directory (e.g., build/bin)
set_target_properties(VirtualGunstockUI PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# For the driver to link against VirtualGunstockUI_lib, its definition must be visible.
# This is implicitly handled by add_subdirectory in the parent scope.
# If VirtualGunstockUI_lib were an INTERFACE library, we might need to do more here,
# but as a STATIC library, its properties (like include directories) are propagated.
