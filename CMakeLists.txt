cmake_minimum_required(VERSION 3.10)
project(OpenVRVirtualControllerSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include GNUInstallDirs for standard installation paths (CMAKE_INSTALL_BINDIR, etc.)
include(GNUInstallDirs)

# --- OpenVR Submodule ---
# Add OpenVR submodule. It is expected to provide the target OpenVR::openvr_api.
# If it doesn't, or uses a different name, this will need adjustment.
add_subdirectory(openvr)

# Verify that the OpenVR target is available.
# The openvr submodule's CMakeLists.txt should define OpenVR::openvr_api.
# If not, we might need to create an alias or an interface library as a fallback,
# but for a standard OpenVR SDK submodule, OpenVR::openvr_api is the common target.
if(NOT TARGET OpenVR::openvr_api)
    if(TARGET openvr_api) # A common older name
        add_library(OpenVR::openvr_api ALIAS openvr_api)
        message(STATUS "Aliased existing target 'openvr_api' to 'OpenVR::openvr_api'.")
    else()
        message(FATAL_ERROR "OpenVR target 'OpenVR::openvr_api' (or 'openvr_api') not found after adding subdirectory 'openvr'. Please check the OpenVR submodule's CMake configuration.")
    endif()
else()
    message(STATUS "Using target 'OpenVR::openvr_api' from OpenVR submodule.")
endif()

# --- Add Subprojects ---

# IPC Utility Library (Static)
add_subdirectory(src/IPC)

# OpenVR Driver (Shared Library)
add_subdirectory(src/driver)

# OpenVR Client (Executable)
add_subdirectory(src/client)


# --- Installation ---
# Installation for the client (OpenVRClient) is handled in src/client/CMakeLists.txt
# Installation for the driver (VirtualGunstockDriver) is handled in src/driver/CMakeLists.txt
# No top-level installation rules needed here unless for project-wide files.

message(STATUS "Project ${PROJECT_NAME} configured.")
message(STATUS "  IPC Utility: target 'IPCUtils' in src/IPC/")
message(STATUS "  Driver:      target 'VirtualGunstockDriver' in src/driver/")
message(STATUS "  Client:      target 'OpenVRClient' in src/client/")
message(STATUS "Make sure the OpenVR submodule ('openvr') correctly provides the 'OpenVR::openvr_api' target.")
