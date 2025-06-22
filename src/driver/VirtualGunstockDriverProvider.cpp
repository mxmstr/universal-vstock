#include "VirtualGunstockDriverProvider.h"
#include "VirtualDevice.h" // Added
#include "openvr_driver.h" // For VR_INIT_SERVER_DRIVER_CONTEXT, k_InterfaceVersions, DriverPose_t, VRServerDriverHost
#include <cstdint> // For uint32_t
#include "IPCUtils.h" // Include for IPC functions
#include <string> // For std::string comparison

VirtualGunstockDriverProvider g_driverProvider;

vr::EVRInitError VirtualGunstockDriverProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitializeIPC(); // Initialize IPC

    const char* right_vstock_serial = "vstock_override_right_01";
    const char* left_vstock_serial = "vstock_override_left_01";

    auto rightDevice = std::make_unique<VirtualDevice>(right_vstock_serial, vr::ETrackedDeviceClass::TrackedDeviceClass_Controller);
    if (vr::VRServerDriverHost()) { // Good practice to check if VRServerDriverHost is available
        vr::VRServerDriverHost()->TrackedDeviceAdded(rightDevice->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, rightDevice.get());
    }
    m_vecVirtualDevices.push_back(std::move(rightDevice));

    auto leftDevice = std::make_unique<VirtualDevice>(left_vstock_serial, vr::ETrackedDeviceClass::TrackedDeviceClass_Controller);
    if (vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->TrackedDeviceAdded(leftDevice->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, leftDevice.get());
    }
    m_vecVirtualDevices.push_back(std::move(leftDevice));

    return vr::VRInitError_None;
}

void VirtualGunstockDriverProvider::Cleanup()
{
    m_vecVirtualDevices.clear(); // Clear devices first
    CleanupIPC(); // Cleanup IPC
}

const char * const *VirtualGunstockDriverProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void VirtualGunstockDriverProvider::RunFrame()
{
    // Read the data from shared memory (with proper synchronization like a mutex!)
    PoseUpdateData data = ReadFromSharedMemory();

    if (data.shouldUpdate)
    {
        for (auto& device : m_vecVirtualDevices) {
            // Assuming PoseUpdateData.serialNumberToUpdate is compatible with std::string comparison
            if (device && device->GetSerialNumber() == data.serialNumberToUpdate) {
                device->UpdatePose(*reinterpret_cast<vr::DriverPose_t*>(&data.pose));
                // data.shouldUpdate = false; // Typically reset after processing, if this is the sole handler for the update.
                break; // Assuming one update per frame for a specific serial
            }
        }
        // If data.shouldUpdate was potentially modified and needs to be written back:
        // WriteToSharedMemory(data);
    }
}

bool VirtualGunstockDriverProvider::ShouldBlockStandbyMode()
{
    return false;
}

void VirtualGunstockDriverProvider::EnterStandby()
{
    // Logic for entering standby, if any
}

void VirtualGunstockDriverProvider::LeaveStandby()
{
    // Logic for leaving standby, if any
}
