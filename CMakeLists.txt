cmake_minimum_required(VERSION 3.10)
project(VirtualGunstockSystem LANGUAGES CXX) # Renamed project for clarity

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- OpenVR Submodule ---
# Add OpenVR submodule. It's assumed this creates a target (e.g., openvr_api or similar).
# We will create an interface library OpenVR::openvr to abstract the actual target name.
add_subdirectory(openvr)

# Create an interface library for OpenVR.
# This provides a consistent way for subprojects to link against OpenVR.
# Replace 'openvr_api' with the actual target name created by add_subdirectory(openvr) if different.
# If openvr/CMakeLists.txt itself defines OpenVR::OpenVR or openvr::openvr, this might not be needed
# or would alias that. For now, we assume a target 'openvr_api' is created.
if(TARGET openvr_api)
    add_library(OpenVR::openvr INTERFACE IMPORTED GLOBAL)
    target_link_libraries(OpenVR::openvr INTERFACE openvr_api)
    # If openvr_api doesn't propagate include directories, uncomment and adjust:
    # target_include_directories(OpenVR::openvr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers)
else()
    # Fallback: If no clear target from add_subdirectory(openvr), create a simple interface
    # This assumes OpenVR headers are in openvr/headers and a library might be found or built.
    # This part would need more robust handling if 'openvr_api' target doesn't exist.
    add_library(OpenVR::openvr INTERFACE)
    target_include_directories(OpenVR::openvr INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers"
        "${CMAKE_CURRENT_SOURCE_DIR}/openvr/samples/drivers/utils/driverlog" # Example, adjust if needed
        "${CMAKE_CURRENT_SOURCE_DIR}/openvr/samples/drivers/utils/vrmath"    # Example, adjust if needed
    )
    # If OpenVR provides a pre-compiled library or 'add_subdirectory(openvr)' builds one with a known name:
    # target_link_libraries(OpenVR::openvr INTERFACE NameOfOpenVRLibraryTarget)
    message(WARNING "OpenVR target 'openvr_api' not found after add_subdirectory(openvr). OpenVR::openvr interface created with default include paths. Linking might require manual setup if OpenVR isn't built as a library by the submodule's CMake.")
endif()


# --- Add Subprojects ---
# The VirtualGunstockUI subproject provides VirtualGunstockUI_lib (for IPC) and the UI executable
add_subdirectory(VirtualGunstockUI)

# The driver subproject provides the VirtualGunstockDriver library
add_subdirectory(driver)


# --- Installation ---
# Installation for VirtualGunstockUI (executable)
# The target VirtualGunstockUI must be defined in VirtualGunstockUI/CMakeLists.txt
# The driver installation is handled within driver/CMakeLists.txt
if(TARGET VirtualGunstockUI)
    install(TARGETS VirtualGunstockUI RUNTIME DESTINATION bin)
else()
    message(WARNING "Target VirtualGunstockUI not found for installation. Check VirtualGunstockUI/CMakeLists.txt.")
endif()

message(STATUS "Project ${PROJECT_NAME} configured. Build with your chosen generator.")
message(STATUS "OpenVR submodule expected to provide its targets (e.g., 'openvr_api' or similar).")
message(STATUS "Driver: target 'VirtualGunstockDriver' in driver/")
message(STATUS "UI: target 'VirtualGunstockUI' and 'VirtualGunstockUI_lib' in VirtualGunstockUI/")
