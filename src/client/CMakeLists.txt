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

# Deps
set(OPENVR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers")

set(SIZEOF_VOIDP ${CMAKE_SIZEOF_VOID_P})
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROCESSOR_ARCH "64")
else()
    set(PROCESSOR_ARCH "32")
endif()

if(WIN32)
    set(PLATFORM_NAME "win")
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_NAME "linux")
elseif(APPLE)
    set(PLATFORM_NAME "osx")
endif()

find_library(OPENVR_LIB openvr_api HINTS "${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/${PLATFORM_NAME}${PROCESSOR_ARCH}/" NO_DEFAULT_PATH )

target_link_libraries(OpenVRClient PUBLIC "${OPENVR_LIB}")

include_directories("${CMAKE_SOURCE_DIR}/openvr/headers")

# Link against OpenVR and IPCUtils
target_link_libraries(OpenVRClient PRIVATE
    #openvr_api # Provided by the OpenVR submodule
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
