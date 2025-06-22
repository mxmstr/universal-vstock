cmake_minimum_required(VERSION 3.10)
project(VirtualGunstockSystem LANGUAGES CXX) # Renamed project for clarity

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- OpenVR Submodule ---
# Add OpenVR submodule. It's assumed this creates a target (e.g., openvr_api or similar).
# We will create an interface library OpenVR::openvr to abstract the actual target name.
add_subdirectory(openvr)

# Attempt to create a consistent OpenVR target name for subprojects.
# The openvr submodule's CMakeLists.txt is expected to create a target.
# Common names are 'openvr_api', 'OpenVR::OpenVR', or 'openvr::openvr'.

if(TARGET OpenVR::OpenVR)
    # If the submodule already provides OpenVR::OpenVR, just use it by ensuring it's globally visible or aliasing.
    # For an imported target from a subdirectory to be visible, it should be GLOBAL or an ALIAS should be made.
    # If OpenVR::OpenVR is IMPORTED, it should be globally imported.
    # If it's a regular target, we can alias it.
    if (GET_TARGET_PROPERTY OpenVR_is_imported OpenVR::OpenVR IMPORTED)
        # Already an imported target, assume it's correctly set up by openvr's CMake.
        # If not GLOBAL, this might still be an issue for other subdirectories.
        # For safety, one could add an alias if it's not already global.
        message(STATUS "Using existing IMPORTED target OpenVR::OpenVR from submodule.")
    else()
        add_library(OpenVR::openvr ALIAS OpenVR::OpenVR)
        message(STATUS "Aliasing existing target OpenVR::OpenVR to OpenVR::openvr.")
    endif()
elseif(TARGET openvr::openvr) # Some projects use lowercase
    if (GET_TARGET_PROPERTY OpenVR_is_imported openvr::openvr IMPORTED)
        message(STATUS "Using existing IMPORTED target openvr::openvr from submodule.")
    else()
        add_library(OpenVR::openvr ALIAS openvr::openvr)
        message(STATUS "Aliasing existing target openvr::openvr to OpenVR::openvr.")
    endif()
elseif(TARGET openvr_api)
    # If openvr_api is the target provided by the submodule.
    add_library(OpenVR::openvr ALIAS openvr_api)
    message(STATUS "Aliasing existing target openvr_api to OpenVR::openvr.")
else()
    # Fallback: If no known target is found from add_subdirectory(openvr),
    # create a new INTERFACE library for OpenVR.
    # This assumes OpenVR headers are in openvr/headers.
    add_library(OpenVR::openvr INTERFACE)
    target_include_directories(OpenVR::openvr INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers"
        # Add other necessary include paths from the OpenVR submodule if needed
        # e.g., "${CMAKE_CURRENT_SOURCE_DIR}/openvr/samples/drivers/utils/driverlog"
        # e.g., "${CMAKE_CURRENT_SOURCE_DIR}/openvr/samples/drivers/utils/vrmath"
    )
    # If OpenVR submodule provides a pre-compiled library that's not made into a target,
    # you might need to link to it directly:
    # find_library(OPENVR_LIBRARY_FILE NAMES openvr_api PATHS "${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/win64" REQUIRED) # Example
    # target_link_libraries(OpenVR::openvr INTERFACE ${OPENVR_LIBRARY_FILE})
    message(WARNING "No standard OpenVR target (OpenVR::OpenVR, openvr::openvr, openvr_api) found after add_subdirectory(openvr). Created a new INTERFACE library OpenVR::openvr. Manual configuration for linking might be needed if OpenVR is not header-only or if its library isn't automatically found/built.")
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
