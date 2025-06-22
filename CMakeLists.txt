cmake_minimum_required(VERSION 3.10)
project(OpenVRVirtualControllerSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include GNUInstallDirs for standard installation paths (CMAKE_INSTALL_BINDIR, etc.)
include(GNUInstallDirs)




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
