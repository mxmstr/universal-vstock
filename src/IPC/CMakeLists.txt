cmake_minimum_required(VERSION 3.10)
project(IPCUtils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the library
add_library(IPCUtils STATIC
    GunstockConfigData.h
    IPCUtils.cpp
    IPCUtils.h
)

include_directories("${CMAKE_SOURCE_DIR}/openvr/headers")

# Define public headers
target_include_directories(IPCUtils PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Ensure PoseUpdateData.h is accessible to targets linking IPCUtils
# This is implicitly handled by target_include_directories if PoseUpdateData.h is in the same directory as IPCUtils.h
# If it were in a subdirectory, e.g., include/, then that would be specified.

# Optional: Add install rule if this library might be installed independently
# install(TARGETS IPCUtils
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ipc
# )
