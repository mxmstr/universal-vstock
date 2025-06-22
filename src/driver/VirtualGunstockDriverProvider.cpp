#include "VirtualGunstockDriverProvider.h"
#include "VirtualDevice.h"
#include "openvr_driver.h"
#include <cstdint>
#include "../IPC/IPCUtils.h" // Ensure this path is correct for IPCUtils.h
// GunstockConfigData.h is included via VirtualGunstockDriverProvider.h
#include <string>
#include <cstring> // For memset

// Forward declarations for helper functions (to be defined later in this file)
static vr::HmdMatrix34_t MultiplyMatrices(const vr::HmdMatrix34_t& matA, const vr::HmdMatrix34_t& matB);
static vr::DriverPose_t ConvertMatrixToDriverPose(const vr::HmdMatrix34_t& matrix);
static void MatrixSetIdentity(vr::HmdMatrix34_t *pMatrix);

VirtualGunstockDriverProvider g_driverProvider;

vr::EVRInitError VirtualGunstockDriverProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitializeIPC(); // For reading config, not poses

    // Create your virtual devices
    m_vecVirtualDevices.push_back(std::make_unique<VirtualDevice>("vstock_override_right_01", vr::ETrackedDeviceClass::Controller));
    if (vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_vecVirtualDevices[0]->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::Controller, m_vecVirtualDevices[0].get());
    } else {
        // Log error or handle missing VRServerDriverHost
        return vr::VRInitError_Init_InvalidInterface;
    }

    m_vecVirtualDevices.push_back(std::make_unique<VirtualDevice>("vstock_override_left_01", vr::ETrackedDeviceClass::Controller));
    if (vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_vecVirtualDevices[1]->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::Controller, m_vecVirtualDevices[1].get());
    } else {
        // Log error or handle missing VRServerDriverHost
        return vr::VRInitError_Init_InvalidInterface;
    }

    // Get handles to the RAW poses of the real controllers
    if (vr::VRDriverInput()) {
        vr::VRDriverInput()->CreatePoseComponent("/user/hand/right/input/pose/raw", &m_hRightHandPose);
        vr::VRDriverInput()->CreatePoseComponent("/user/hand/left/input/pose/raw", &m_hLeftHandPose);

        // Get a handle to the button we'll use for activation (e.g., left grip)
        vr::VRDriverInput()->CreateBooleanComponent("/user/hand/left/input/grip/click", &m_hOffHandGrip);
    } else {
        // Log error or handle missing VRDriverInput
        return vr::VRInitError_Init_InvalidInterface;
    }


    // Initialize default config
    m_config.config_updated = false;
    MatrixSetIdentity(&m_config.offset_matrix);
    m_config.offset_matrix.m[2][3] = 0.1f; // 10cm forward example

    // Example: Initialize other config values if they were added
    // m_config.dominant_hand = 1; // Right hand
    // m_config.activation_button_id = vr::k_EButton_Grip; // Example

    return vr::VRInitError_None;
}

void VirtualGunstockDriverProvider::Cleanup()
{
    CleanupIPC(); // Cleanup IPC first
    m_vecVirtualDevices.clear(); // Then clear devices
}

const char * const *VirtualGunstockDriverProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void VirtualGunstockDriverProvider::RunFrame()
{
    // 1. (Optional) Check IPC for any new configuration from the UI app
    // GunstockConfigData newConfig = ReadFromSharedMemory();
    // if (newConfig.config_updated) {
    //     m_config = newConfig;
    //     m_config.config_updated = false; // Reset flag after applying
    //     // Potentially re-initialize or update something based on new config
    // }

    // 2. Update the input components to get the latest data
    if (vr::VRDriverInput()) {
        vr::VRDriverInput()->UpdatePoseComponent(m_hRightHandPose, 0.0f, 0.0f); // Added timeOffset and fSecondsFromNow arguments
        vr::VRDriverInput()->UpdatePoseComponent(m_hLeftHandPose, 0.0f, 0.0f);  // Added timeOffset and fSecondsFromNow arguments
        vr::VRDriverInput()->UpdateBooleanComponent(m_hOffHandGrip, 0.0f, 0.0f); // Added timeOffset and fSecondsFromNow arguments for boolean component
    } else {
        return; // Cannot proceed without VRDriverInput
    }


    // 3. Read the data from the components
    vr::InputPoseActionData_t rightPoseData = {0};
    vr::InputPoseActionData_t leftPoseData = {0};
    vr::InputDigitalActionData_t gripData = {0};

    if (vr::VRDriverInput()) {
        vr::VRDriverInput()->GetPoseActionDataForDevice(m_hRightHandPose, vr::k_ulInvalidActionSetHandle, vr::k_ulInvalidInputValueHandle, &rightPoseData, sizeof(rightPoseData), vr::k_ulInvalidPropertyContainer);
        vr::VRDriverInput()->GetPoseActionDataForDevice(m_hLeftHandPose, vr::k_ulInvalidActionSetHandle, vr::k_ulInvalidInputValueHandle, &leftPoseData, sizeof(leftPoseData), vr::k_ulInvalidPropertyContainer);
        // The GetDigitalActionData function signature in openvr_driver.h is:
        // EVRInputError (OPENVR_FNTABLE_CALLTYPE *GetDigitalActionData)( VRInputComponentHandle_t ulComponent, InputDigitalActionData_t *pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice );
        vr::VRDriverInput()->GetDigitalActionData(m_hOffHandGrip, &gripData, sizeof(gripData), vr::k_ulInvalidInputValueHandle );
    } else {
        return; // Cannot proceed
    }

    // 4. Perform the gunstock logic
    bool isGunstockActive = gripData.bActive && gripData.bState;

    // Ensure devices exist
    if (m_vecVirtualDevices.size() < 2) return; // Should not happen if Init succeeded

    if (isGunstockActive && rightPoseData.pose.bPoseIsValid && rightPoseData.bActive) // Check rightPoseData.bActive as well
    {
        vr::HmdMatrix34_t dominant_hand_pose = rightPoseData.pose.mDeviceToAbsoluteTracking;
        vr::HmdMatrix34_t new_off_hand_pose_matrix;

        new_off_hand_pose_matrix = MultiplyMatrices(dominant_hand_pose, m_config.offset_matrix);

        vr::DriverPose_t newPose = ConvertMatrixToDriverPose(new_off_hand_pose_matrix);

        // Copy velocity from the dominant hand
        newPose.vecVelocity[0] = rightPoseData.pose.vVelocity.v[0];
        newPose.vecVelocity[1] = rightPoseData.pose.vVelocity.v[1];
        newPose.vecVelocity[2] = rightPoseData.pose.vVelocity.v[2];
        newPose.vecAngularVelocity[0] = rightPoseData.pose.vAngularVelocity.v[0];
        newPose.vecAngularVelocity[1] = rightPoseData.pose.vAngularVelocity.v[1];
        newPose.vecAngularVelocity[2] = rightPoseData.pose.vAngularVelocity.v[2];

        newPose.poseIsValid = true;
        newPose.result = vr::TrackingResult_Running_OK;
        newPose.deviceIsConnected = true;

        m_vecVirtualDevices[1]->UpdatePose(newPose); // Update the LEFT virtual device (off-hand)
    }
    else if (leftPoseData.pose.bPoseIsValid && leftPoseData.bActive) // If not active, pass through left hand
    {
        // If gunstock is NOT active, pass through the real left hand pose
         m_vecVirtualDevices[1]->UpdatePose(leftPoseData.pose);
    } else {
        // Keep current pose or set to invalid if real left hand is not valid
        vr::DriverPose_t currentPose = m_vecVirtualDevices[1]->GetPose();
        currentPose.poseIsValid = false;
        currentPose.result = vr::TrackingResult_Uninitialized;
        m_vecVirtualDevices[1]->UpdatePose(currentPose);
    }

    // Always update the right hand's virtual device to pass through the real pose
    if(rightPoseData.pose.bPoseIsValid && rightPoseData.bActive) { // Check rightPoseData.bActive
        m_vecVirtualDevices[0]->UpdatePose(rightPoseData.pose);
    } else {
        // Keep current pose or set to invalid if real right hand is not valid
        vr::DriverPose_t currentPose = m_vecVirtualDevices[0]->GetPose();
        currentPose.poseIsValid = false;
        currentPose.result = vr::TrackingResult_Uninitialized;
        m_vecVirtualDevices[0]->UpdatePose(currentPose);
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

// Helper function to set a 3x4 matrix to identity
static void MatrixSetIdentity(vr::HmdMatrix34_t *pMatrix)
{
    memset(pMatrix, 0, sizeof(vr::HmdMatrix34_t));
    pMatrix->m[0][0] = 1.0f;
    pMatrix->m[1][1] = 1.0f;
    pMatrix->m[2][2] = 1.0f;
}

// Helper function for matrix multiplication (matC = matA * matB)
static vr::HmdMatrix34_t MultiplyMatrices(const vr::HmdMatrix34_t& matA, const vr::HmdMatrix34_t& matB) {
    vr::HmdMatrix34_t result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                result.m[i][j] += matA.m[i][k] * matB.m[k][j];
            }
            if (j == 3) { // Translation column
                result.m[i][j] += matA.m[i][3];
            }
        }
    }
    return result;
}

// Helper function to convert HmdMatrix34_t to DriverPose_t
// This is a simplified conversion. A full conversion would also handle QWorldFromDriverRotation etc. if needed.
static vr::DriverPose_t ConvertMatrixToDriverPose(const vr::HmdMatrix34_t& matrix) {
    vr::DriverPose_t pose = {0};

    // Position
    pose.vecPosition[0] = matrix.m[0][3];
    pose.vecPosition[1] = matrix.m[1][3];
    pose.vecPosition[2] = matrix.m[2][3];

    // Rotation (matrix to quaternion)
    // From https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    float trace = matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2];
    if (trace > 0) {
        float S = sqrt(trace + 1.0f) * 2.0f; // S=4*qw
        pose.qRotation.w = 0.25f * S;
        pose.qRotation.x = (matrix.m[2][1] - matrix.m[1][2]) / S;
        pose.qRotation.y = (matrix.m[0][2] - matrix.m[2][0]) / S;
        pose.qRotation.z = (matrix.m[1][0] - matrix.m[0][1]) / S;
    } else if ((matrix.m[0][0] > matrix.m[1][1]) && (matrix.m[0][0] > matrix.m[2][2])) {
        float S = sqrt(1.0f + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2]) * 2.0f; // S=4*qx
        pose.qRotation.w = (matrix.m[2][1] - matrix.m[1][2]) / S;
        pose.qRotation.x = 0.25f * S;
        pose.qRotation.y = (matrix.m[0][1] + matrix.m[1][0]) / S;
        pose.qRotation.z = (matrix.m[0][2] + matrix.m[2][0]) / S;
    } else if (matrix.m[1][1] > matrix.m[2][2]) {
        float S = sqrt(1.0f + matrix.m[1][1] - matrix.m[0][0] - matrix.m[2][2]) * 2.0f; // S=4*qy
        pose.qRotation.w = (matrix.m[0][2] - matrix.m[2][0]) / S;
        pose.qRotation.x = (matrix.m[0][1] + matrix.m[1][0]) / S;
        pose.qRotation.y = 0.25f * S;
        pose.qRotation.z = (matrix.m[1][2] + matrix.m[2][1]) / S;
    } else {
        float S = sqrt(1.0f + matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1]) * 2.0f; // S=4*qz
        pose.qRotation.w = (matrix.m[1][0] - matrix.m[0][1]) / S;
        pose.qRotation.x = (matrix.m[0][2] + matrix.m[2][0]) / S;
        pose.qRotation.y = (matrix.m[1][2] + matrix.m[2][1]) / S;
        pose.qRotation.z = 0.25f * S;
    }

    // Set defaults for other pose members
    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected = true;
    pose.willDriftInYaw = false;
    pose.shouldApplyHeadModel = false;
    pose.poseTimeOffset = 0.f;
    // Initialize QWorldFromDriverRotation to identity
    pose.qWorldFromDriverRotation.w = 1.f;
    pose.qWorldFromDriverRotation.x = 0.f;
    pose.qWorldFromDriverRotation.y = 0.f;
    pose.qWorldFromDriverRotation.z = 0.f;
    // Initialize qDriverFromHead_RawRotation to identity
    pose.qDriverFromHead_RawRotation.w = 1.f;
    pose.qDriverFromHead_RawRotation.x = 0.f;
    pose.qDriverFromHead_RawRotation.y = 0.f;
    pose.qDriverFromHead_RawRotation.z = 0.f;


    // Velocities and angular velocities should be copied from the source controller if available
    // This function only converts the matrix part.
    memset(pose.vecVelocity, 0, sizeof(pose.vecVelocity));
    memset(pose.vecAngularVelocity, 0, sizeof(pose.vecAngularVelocity));

    return pose;
}
