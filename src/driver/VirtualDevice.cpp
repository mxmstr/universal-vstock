// VirtualDevice.cpp
#include "VirtualDevice.h"


VirtualDevice::VirtualDevice(std::string serialNumber, vr::ETrackedDeviceClass deviceClass) {
    m_sSerialNumber = serialNumber;
    m_eDeviceClass = deviceClass; // Store the device class
    m_pose = {0}; // Initialize pose to a default state
    m_pose.qRotation = {1, 0, 0, 0}; // Valid identity quaternion
    m_pose.vecPosition[0] = 0.0;
    m_pose.vecPosition[1] = 0.0;
    m_pose.vecPosition[2] = 0.0;
    m_pose.poseIsValid = false; // Start as invalid until we get data
    m_pose.result = vr::TrackingResult_Uninitialized;
    m_pose.deviceIsConnected = true; // Assuming the virtual device is always connected
    m_pose.willDriftInYaw = false;
    m_pose.shouldApplyHeadModel = false;
    m_pose.poseTimeOffset = 0.f;
    // m_pose.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0}; // Identity
    // m_pose.qDriverFromHead_RawRotation = {1.0, 0.0, 0.0, 0.0}; // Identity
    // m_pose.vecDriverFromHead_Translation[0] = 0.f;
    // m_pose.vecDriverFromHead_Translation[1] = 0.f;
    // m_pose.vecDriverFromHead_Translation[2] = 0.f;
    // m_pose.vecSecondaryCameraTranslation[0] = 0.f;
    // m_pose.vecSecondaryCameraTranslation[1] = 0.f;
    // m_pose.vecSecondaryCameraTranslation[2] = 0.f;
    // m_pose.qSecondaryCameraRotation = {1.0, 0.0, 0.0, 0.0}; // Identity
    // m_pose.ucSecondaryCameraFOV = 0;
    // m_pose.ucSecondaryCameraNearFarRange[0] = 0;
    // m_pose.ucSecondaryCameraNearFarRange[1] = 0;


}

vr::EVRInitError VirtualDevice::Activate(vr::TrackedDeviceIndex_t unObjectId) {
    m_unObjectId = unObjectId; // SteamVR gives us an ID. Store it.
    
    // Optionally, you can inform SteamVR about the properties of this device here.
    // For example, its model number, render model, etc.
    // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "MyVirtualController_v1");
    // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5"); // Example render model

    return vr::VRInitError_None;
}

void VirtualDevice::Deactivate() {
    m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
}

void VirtualDevice::EnterStandby() {
    // Typically do nothing for a virtual device
}

void* VirtualDevice::GetComponent(const char* pchComponentNameAndVersion) {
    // No components on this virtual device, or specific components like IVRControllerComponent can be returned here
    return nullptr;
}

void VirtualDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    // No debug requests are handled by this sample
    if (unResponseBufferSize > 0) {
        pchResponseBuffer[0] = ' '; // Ensure a null-terminated string
    }
}

// This is the MOST IMPORTANT method for this approach!
vr::DriverPose_t VirtualDevice::GetPose() {
    // This method is called by SteamVR to get the current pose of the device.
    // The pose should be updated by your driver's RunFrame() loop.
    return m_pose;
}

std::string VirtualDevice::GetSerialNumber() {
    return m_sSerialNumber;
}

// You will call this from your main driver provider's RunFrame loop
void VirtualDevice::UpdatePose(const vr::DriverPose_t& newPose) {
    m_pose = newPose;
    m_pose.poseIsValid = true; // Mark the pose as valid
    m_pose.result = vr::TrackingResult_Running_OK; // Indicate that tracking is running correctly
    m_pose.deviceIsConnected = true; // Ensure device is marked as connected

    // If your device provides velocity and angular velocity, make sure they are also updated.
    // For example:
    // m_pose.vecVelocity[0] = newPose.vecVelocity[0];
    // m_pose.vecVelocity[1] = newPose.vecVelocity[1];
    // m_pose.vecVelocity[2] = newPose.vecVelocity[2];
    // m_pose.vecAngularVelocity[0] = newPose.vecAngularVelocity[0];
    // m_pose.vecAngularVelocity[1] = newPose.vecAngularVelocity[1];
    // m_pose.vecAngularVelocity[2] = newPose.vecAngularVelocity[2];

    // After updating the pose, if the device is active, inform SteamVR.
    if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_pose, sizeof(vr::DriverPose_t));
    }
}
