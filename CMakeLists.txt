cmake_minimum_required(VERSION 3.10)
project(OpenVRDriverExample LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add OpenVR submodule
add_subdirectory(openvr)

# Include directories
include_directories(
    openvr/headers
    openvr/samples/drivers/utils/driverlog
    openvr/samples/drivers/utils/vrmath
    include
)

# Source files
file(GLOB DRIVER_SOURCES "src/*")
file(GLOB DRIVER_HEADERS "include/*")

# Create the driver library
add_library(driver_universal_vstock SHARED 
    ${DRIVER_SOURCES}
    ${DRIVER_HEADERS}
)

# Link against OpenVR
target_link_libraries(driver_universal_vstock PRIVATE ${OpenVR_LIBRARIES})

# Define platform-specific properties
if(WIN32)
    set_target_properties(driver_universal_vstock PROPERTIES SUFFIX ".dll")
    target_compile_definitions(driver_universal_vstock PRIVATE "WIN32" "_WINDOWS" "CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS")
elseif(LINUX)
    set_target_properties(driver_universal_vstock PROPERTIES PREFIX "lib" SUFFIX ".so")
    target_compile_options(driver_universal_vstock PRIVATE -fPIC)
elseif(APPLE)
    set_target_properties(driver_universal_vstock PROPERTIES SUFFIX ".dylib")
    target_compile_options(driver_universal_vstock PRIVATE -fPIC)
endif()

# Installation
# This part might need adjustment based on SteamVR's expected driver structure
# and the target operating system.
# For now, we'll just define an install target for the library.
install(TARGETS driver_universal_vstock
    LIBRARY DESTINATION lib/driver_universal_vstock
    RUNTIME DESTINATION bin/driver_universal_vstock
)

# Add a custom build step to copy resources (if any)
# file(COPY resources/ DESTINATION ${CMAKE_BINARY_DIR}/resources)
# install(DIRECTORY resources/ DESTINATION share/driver_universal_vstock/resources)

# Add a custom build step to copy the driver manifest
# file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR})
# install(FILES driver.vrdrivermanifest DESTINATION share/driver_universal_vstock)

message(STATUS "OpenVR SDK Path: ${OPENVR_INCLUDE_DIR}")
message(STATUS "Driver sources: ${DRIVER_SOURCES}")
