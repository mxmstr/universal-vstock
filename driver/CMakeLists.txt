cmake_minimum_required(VERSION 3.10)
project(VirtualGunstockDriver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files for the driver
set(DRIVER_SOURCES
    "VirtualDevice.cpp"
    "VirtualGunstockDriverProvider.cpp"
    "driver_main.cpp"
    "VirtualDevice.h"
    "VirtualGunstockDriverProvider.h"
)

# Create the driver library
add_library(VirtualGunstockDriver SHARED ${DRIVER_SOURCES})

# Target include directories
# The actual targets OpenVR::openvr and VirtualGunstockUI_lib (or a shared lib)
# will be set up in the root CMakeLists.txt and VirtualGunstockUI/CMakeLists.txt
target_include_directories(VirtualGunstockDriver PUBLIC
    $<TARGET_PROPERTY:OpenVR::openvr,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:VirtualGunstockUI_lib,INTERFACE_INCLUDE_DIRECTORIES> # For IPCUtils and PoseUpdateData
    ${CMAKE_CURRENT_SOURCE_DIR} # For driver's own headers
)

# Link against OpenVR and the UI/Shared library (for IPC)
target_link_libraries(VirtualGunstockDriver PRIVATE
    OpenVR::openvr
    VirtualGunstockUI_lib # For IPCUtils symbols
)

# Define platform-specific properties
if(WIN32)
    set_target_properties(VirtualGunstockDriver PROPERTIES SUFFIX ".dll")
    target_compile_definitions(VirtualGunstockDriver PRIVATE "WIN32" "_WINDOWS" "CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS")
    # Copy the DLL to a known location (e.g., build output directory/driver)
    add_custom_command(TARGET VirtualGunstockDriver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VirtualGunstockDriver>
        ${CMAKE_BINARY_DIR}/driver_universal_vstock/bin/win64/driver_VirtualGunstockDriver.dll # Example path
    )
    # Copy manifest file
    configure_file(driver.vrdrivermanifest ${CMAKE_BINARY_DIR}/driver_universal_vstock/driver.vrdrivermanifest COPYONLY)

elseif(LINUX)
    set_target_properties(VirtualGunstockDriver PROPERTIES PREFIX "lib" SUFFIX ".so")
    target_compile_options(VirtualGunstockDriver PRIVATE -fPIC)
    # Copy the .so to a known location
     add_custom_command(TARGET VirtualGunstockDriver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VirtualGunstockDriver>
        ${CMAKE_BINARY_DIR}/driver_universal_vstock/bin/linux64/libVirtualGunstockDriver.so # Example path
    )
    # Copy manifest file
    configure_file(driver.vrdrivermanifest ${CMAKE_BINARY_DIR}/driver_universal_vstock/driver.vrdrivermanifest COPYONLY)
endif()

# Installation (example, adjust as needed)
install(TARGETS VirtualGunstockDriver
    LIBRARY DESTINATION lib/driver_universal_vstock/bin # Or platform specific paths
    RUNTIME DESTINATION lib/driver_universal_vstock/bin
)

install(FILES driver.vrdrivermanifest
    DESTINATION lib/driver_universal_vstock
)
