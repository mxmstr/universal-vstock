cmake_minimum_required(VERSION 3.10)
project(OpenVRClient LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files for the client executable
set(CLIENT_SOURCES
    "main.cpp"
)

# Create the executable
add_executable(OpenVRClient ${CLIENT_SOURCES})

# Target include directories
# IPCUtils and OpenVR::openvr_api should propagate their include directories
# If main.cpp needs direct access to headers in src/client (if any besides main.cpp)
# target_include_directories(OpenVRClient PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

# Link against OpenVR and IPCUtils
target_link_libraries(OpenVRClient PRIVATE
    OpenVR::openvr_api # Provided by the OpenVR submodule
    IPCUtils           # Our static IPC library
)

# Platform specific settings (if any)
if(WIN32)
    target_compile_definitions(OpenVRClient PRIVATE "WIN32" "_WINDOWS")
    # Example: Copy executable to a common bin directory in the build folder
    add_custom_command(TARGET OpenVRClient POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:OpenVRClient>
        "${CMAKE_BINARY_DIR}/bin/" # Copy to <build_dir>/bin/
        COMMENT "Copying client executable to build output bin directory."
    )
elseif(LINUX)
    # Example: Copy executable to a common bin directory in the build folder
    add_custom_command(TARGET OpenVRClient POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:OpenVRClient>
        "${CMAKE_BINARY_DIR}/bin/" # Copy to <build_dir>/bin/
        COMMENT "Copying client executable to build output bin directory."
    )
endif()

# Installation for the client executable
install(TARGETS OpenVRClient
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" # Installs to <prefix>/bin
)
