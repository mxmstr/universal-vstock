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
target_include_directories(VirtualGunstockDriver PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}" # For driver's own headers (VirtualDevice.h, etc.)
    # OpenVR::openvr_api and IPCUtils will propagate their include directories
)

# Link against OpenVR and IPCUtils
target_link_libraries(VirtualGunstockDriver PRIVATE
    OpenVR::openvr_api # Provided by the OpenVR submodule, aliased in root CMakeLists.txt if needed
    IPCUtils           # Our static IPC library
)

# Define platform-specific properties
if(WIN32)
    set_target_properties(VirtualGunstockDriver PROPERTIES SUFFIX ".dll")
    # Ensure symbols are exported. For MinGW, CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS might be needed.
    # For MSVC, you typically use __declspec(dllexport) or a .def file.
    # Assuming CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS is okay for now or handled by OpenVR's structure.
    target_compile_definitions(VirtualGunstockDriver PRIVATE "WIN32" "_WINDOWS" "CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS")

    # Installation: DLL and manifest
    install(TARGETS VirtualGunstockDriver
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/driver_VirtualGunstockDriver/bin/win64"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/driver_VirtualGunstockDriver/bin/win64"
    )
    install(FILES driver.vrdrivermanifest
        DESTINATION "${CMAKE_INSTALL_DATADIR}/driver_VirtualGunstockDriver"
    )

    # Post-build copy for easier testing during development (optional)
    # This copies to a path relative to the build directory, not for installation
    add_custom_command(TARGET VirtualGunstockDriver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VirtualGunstockDriver>
        "${CMAKE_BINARY_DIR}/bin/driver_VirtualGunstockDriver/bin/win64/driver_VirtualGunstockDriver.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/driver.vrdrivermanifest"
        "${CMAKE_BINARY_DIR}/bin/driver_VirtualGunstockDriver/driver.vrdrivermanifest"
        COMMENT "Copying driver files to build output directory for testing."
    )

elseif(LINUX)
    set_target_properties(VirtualGunstockDriver PROPERTIES PREFIX "lib" SUFFIX ".so")
    target_compile_options(VirtualGunstockDriver PRIVATE -fPIC)

    # Installation: .so and manifest
    install(TARGETS VirtualGunstockDriver
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/driver_VirtualGunstockDriver/bin/linux64"
    )
    install(FILES driver.vrdrivermanifest
        DESTINATION "${CMAKE_INSTALL_DATADIR}/driver_VirtualGunstockDriver"
    )

    # Post-build copy for easier testing (optional)
    add_custom_command(TARGET VirtualGunstockDriver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VirtualGunstockDriver>
        "${CMAKE_BINARY_DIR}/lib/driver_VirtualGunstockDriver/bin/linux64/libVirtualGunstockDriver.so"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/driver.vrdrivermanifest"
        "${CMAKE_BINARY_DIR}/lib/driver_VirtualGunstockDriver/driver.vrdrivermanifest"
        COMMENT "Copying driver files to build output directory for testing."
    )
endif()

# Ensure dependent targets provide necessary include directories
# This is typically handled by target_link_libraries with PUBLIC/INTERFACE dependencies
# For example, IPCUtils should have:
# target_include_directories(IPCUtils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# And OpenVR::openvr_api is expected to do the same.
