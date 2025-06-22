#include "VirtualGunstockDriverProvider.h"
#include "VirtualDevice.h"
//#include "openvr_driver.h"
#include <cstdint>
#include "../IPC/IPCUtils.h" // Ensure this path is correct for IPCUtils.h
// GunstockConfigData.h is included via VirtualGunstockDriverProvider.h
#include <string>
#include <cstring> // For memset
#include <cmath> // For sqrt in ConvertMatrixToDriverPose

// Forward declarations for helper functions (to be defined later in this file)
static vr::HmdMatrix34_t MultiplyMatrices(const vr::HmdMatrix34_t& matA, const vr::HmdMatrix34_t& matB);
static vr::DriverPose_t ConvertHmdMatrix34ToDriverPose(const vr::HmdMatrix34_t& matrix); // Renamed for clarity
static vr::DriverPose_t ConvertTrackedDevicePoseToDriverPose(const vr::TrackedDevicePose_t& tracked_pose);
static void MatrixSetIdentity(vr::HmdMatrix34_t *pMatrix);

VirtualGunstockDriverProvider g_driverProvider;

vr::EVRInitError VirtualGunstockDriverProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitializeIPC(); // For reading config, not poses

    m_unRightControllerIndex = vr::k_unTrackedDeviceIndexInvalid;
    m_unLeftControllerIndex = vr::k_unTrackedDeviceIndexInvalid;

    // Create your virtual devices
    m_vecVirtualDevices.push_back(std::make_unique<VirtualDevice>("vstock_override_right_01", vr::ETrackedDeviceClass::TrackedDeviceClass_Controller));
    if (vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_vecVirtualDevices[0]->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, m_vecVirtualDevices[0].get());
    } else {
        if(vr::VRDriverLog()) vr::VRDriverLog()->Log("VirtualGunstockDriverProvider: VRServerDriverHost is null during right device add.");
        return vr::VRInitError_Init_InvalidInterface;
    }

    m_vecVirtualDevices.push_back(std::make_unique<VirtualDevice>("vstock_override_left_01", vr::ETrackedDeviceClass::TrackedDeviceClass_Controller));
    if (vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_vecVirtualDevices[1]->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, m_vecVirtualDevices[1].get());
    } else {
        if(vr::VRDriverLog()) vr::VRDriverLog()->Log("VirtualGunstockDriverProvider: VRServerDriverHost is null during left device add.");
        return vr::VRInitError_Init_InvalidInterface;
    }

    // IVRDriverInput does not support CreatePoseComponent or CreateBooleanComponent with paths in this OpenVR version.
    // Physical controller identification and input reading will be attempted in RunFrame().
    // m_hRightHandPose, m_hLeftHandPose, m_hOffHandGrip are removed / not used with this approach.
    if(vr::VRDriverLog()) vr::VRDriverLog()->Log("VirtualGunstockDriverProvider: IVRDriverInput methods for raw pose/button paths are not available in this OpenVR version. Will attempt manual controller discovery.");

    // Initialize default config
    m_config.config_updated = false;
    MatrixSetIdentity(reinterpret_cast<vr::HmdMatrix34_t*>(&m_config.offset_matrix));
    m_config.offset_matrix.m[2][3] = 0.1f; // 10cm forward example

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
    // }

    // Ensure virtual devices are present
    if (m_vecVirtualDevices.size() < 2) return;

    // Find physical controllers if not already found
    if (m_unLeftControllerIndex == vr::k_unTrackedDeviceIndexInvalid || m_unRightControllerIndex == vr::k_unTrackedDeviceIndexInvalid)
    {
        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
        {
            if (!vr::VRProperties()) continue;
            vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);
            if (container == vr::k_ulInvalidPropertyContainer) continue;

            vr::ETrackedPropertyError error;
            int32_t controllerRole = vr::VRProperties()->GetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, &error);
            if (error == vr::TrackedProp_Success)
            {
                if (controllerRole == vr::TrackedControllerRole_LeftHand)
                {
                    m_unLeftControllerIndex = i;
                }
                else if (controllerRole == vr::TrackedControllerRole_RightHand)
                {
                    m_unRightControllerIndex = i;
                }
            }
        }
    }

    // If controllers are not found, cannot proceed with logic
    if (m_unLeftControllerIndex == vr::k_unTrackedDeviceIndexInvalid || m_unRightControllerIndex == vr::k_unTrackedDeviceIndexInvalid)
    {
        // Set virtual devices to invalid pose if physical controllers aren't found
        vr::DriverPose_t invalidPose = {0};
        invalidPose.poseIsValid = false;
        invalidPose.result = vr::TrackingResult_Uninitialized;
        invalidPose.deviceIsConnected = false; // Mark as disconnected if we can't find physical counterparts
        m_vecVirtualDevices[0]->UpdatePose(invalidPose);
        m_vecVirtualDevices[1]->UpdatePose(invalidPose);
        return;
    }

    // Get raw poses from all tracked devices
    vr::TrackedDevicePose_t arrTrackedDevicePoses[vr::k_unMaxTrackedDeviceCount];
    if (vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, arrTrackedDevicePoses, vr::k_unMaxTrackedDeviceCount);
    } else {
        return; // Cannot proceed
    }

    const vr::TrackedDevicePose_t& physicalRightControllerPose = arrTrackedDevicePoses[m_unRightControllerIndex];
    const vr::TrackedDevicePose_t& physicalLeftControllerPose  = arrTrackedDevicePoses[m_unLeftControllerIndex];

    // TODO: Implement actual button reading if an alternative method is found.
    // For now, gunstock is considered always active if the dominant (right) hand is tracked.
    // This is a placeholder because IVRDriverInput_003 doesn't support reading specific button inputs
    // from other controllers via paths like /user/hand/left/input/grip/click.
    bool isGunstockActive = physicalRightControllerPose.bPoseIsValid && physicalRightControllerPose.bDeviceIsConnected;


    // Perform the gunstock logic
    if (isGunstockActive)
    {
        vr::HmdMatrix34_t dominant_hand_hmd_matrix = physicalRightControllerPose.mDeviceToAbsoluteTracking;
        vr::HmdMatrix34_t new_off_hand_hmd_matrix = MultiplyMatrices(dominant_hand_hmd_matrix, *reinterpret_cast<vr::HmdMatrix34_t*>(&m_config.offset_matrix));

        vr::DriverPose_t newOffHandDriverPose = ConvertHmdMatrix34ToDriverPose(new_off_hand_hmd_matrix);

        // Copy velocity from the dominant hand (physicalRightControllerPose is TrackedDevicePose_t)
        newOffHandDriverPose.vecVelocity[0] = physicalRightControllerPose.vVelocity.v[0];
        newOffHandDriverPose.vecVelocity[1] = physicalRightControllerPose.vVelocity.v[1];
        newOffHandDriverPose.vecVelocity[2] = physicalRightControllerPose.vVelocity.v[2];
        newOffHandDriverPose.vecAngularVelocity[0] = physicalRightControllerPose.vAngularVelocity.v[0];
        newOffHandDriverPose.vecAngularVelocity[1] = physicalRightControllerPose.vAngularVelocity.v[1];
        newOffHandDriverPose.vecAngularVelocity[2] = physicalRightControllerPose.vAngularVelocity.v[2];

        newOffHandDriverPose.poseIsValid = true; // ConvertHmdMatrix34ToDriverPose sets this, but ensure
        newOffHandDriverPose.result = vr::TrackingResult_Running_OK;
        newOffHandDriverPose.deviceIsConnected = true;

        m_vecVirtualDevices[1]->UpdatePose(newOffHandDriverPose); // Update the LEFT virtual device (off-hand)
    }
    else if (physicalLeftControllerPose.bPoseIsValid && physicalLeftControllerPose.bDeviceIsConnected)
    {
        // If gunstock is NOT active, pass through the real left hand pose
        m_vecVirtualDevices[1]->UpdatePose(ConvertTrackedDevicePoseToDriverPose(physicalLeftControllerPose));
    } else {
        // Keep current pose or set to invalid if real left hand is not valid/connected
        vr::DriverPose_t currentPose = m_vecVirtualDevices[1]->GetPose();
        currentPose.poseIsValid = false;
        currentPose.result = vr::TrackingResult_Uninitialized;
        currentPose.deviceIsConnected = physicalLeftControllerPose.bDeviceIsConnected;
        m_vecVirtualDevices[1]->UpdatePose(currentPose);
    }

    // Always update the right hand's virtual device to pass through the real pose
    if(physicalRightControllerPose.bPoseIsValid && physicalRightControllerPose.bDeviceIsConnected) {
        m_vecVirtualDevices[0]->UpdatePose(ConvertTrackedDevicePoseToDriverPose(physicalRightControllerPose));
    } else {
        // Keep current pose or set to invalid if real right hand is not valid/connected
        vr::DriverPose_t currentPose = m_vecVirtualDevices[0]->GetPose();
        currentPose.poseIsValid = false;
        currentPose.result = vr::TrackingResult_Uninitialized;
        currentPose.deviceIsConnected = physicalRightControllerPose.bDeviceIsConnected;
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
    pose.vecPosition[0] = matrix.m[0][3]; // float to double, implicit conversion is fine
    pose.vecPosition[1] = matrix.m[1][3];
    pose.vecPosition[2] = matrix.m[2][3];

    // Rotation (matrix to quaternion)
    // From https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    // matrix components are float, pose.qRotation components are double.
    double trace = matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2];
    if (trace > 0.0) { // Use double literal
        double S = sqrt(trace + 1.0) * 2.0; // S=4*qw, use double literals and sqrt from cmath (should take double)
        pose.qRotation.w = 0.25 * S;
        pose.qRotation.x = (matrix.m[2][1] - matrix.m[1][2]) / S;
        pose.qRotation.y = (matrix.m[0][2] - matrix.m[2][0]) / S;
        pose.qRotation.z = (matrix.m[1][0] - matrix.m[0][1]) / S;
    } else if ((matrix.m[0][0] > matrix.m[1][1]) && (matrix.m[0][0] > matrix.m[2][2])) {
        double S = sqrt(1.0 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2]) * 2.0; // S=4*qx
        pose.qRotation.w = (matrix.m[2][1] - matrix.m[1][2]) / S;
        pose.qRotation.x = 0.25 * S;
        pose.qRotation.y = (matrix.m[0][1] + matrix.m[1][0]) / S;
        pose.qRotation.z = (matrix.m[0][2] + matrix.m[2][0]) / S;
    } else if (matrix.m[1][1] > matrix.m[2][2]) {
        double S = sqrt(1.0 + matrix.m[1][1] - matrix.m[0][0] - matrix.m[2][2]) * 2.0; // S=4*qy
        pose.qRotation.w = (matrix.m[0][2] - matrix.m[2][0]) / S;
        pose.qRotation.x = (matrix.m[0][1] + matrix.m[1][0]) / S;
        pose.qRotation.y = 0.25 * S;
        pose.qRotation.z = (matrix.m[1][2] + matrix.m[2][1]) / S;
    } else {
        double S = sqrt(1.0 + matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1]) * 2.0; // S=4*qz
        pose.qRotation.w = (matrix.m[1][0] - matrix.m[0][1]) / S;
        pose.qRotation.x = (matrix.m[0][2] + matrix.m[2][0]) / S;
        pose.qRotation.y = (matrix.m[1][2] + matrix.m[2][1]) / S;
        pose.qRotation.z = 0.25 * S;
    }

    // Set defaults for other pose members
    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected = true;
    pose.willDriftInYaw = false;
    pose.shouldApplyHeadModel = false;
    pose.poseTimeOffset = 0.0; // Use double literal
    // Initialize QWorldFromDriverRotation to identity
    pose.qWorldFromDriverRotation.w = 1.0; // Use double literal
    pose.qWorldFromDriverRotation.x = 0.0;
    pose.qWorldFromDriverRotation.y = 0.0;
    pose.qWorldFromDriverRotation.z = 0.0;
    // Initialize qDriverFromHeadRotation to identity
    pose.qDriverFromHeadRotation.w = 1.0;
    pose.qDriverFromHeadRotation.x = 0.0;
    pose.qDriverFromHeadRotation.y = 0.0;
    pose.qDriverFromHeadRotation.z = 0.0;


    // Velocities and angular velocities should be copied from the source controller if available
    // This function only converts the matrix part.
    memset(pose.vecVelocity, 0, sizeof(pose.vecVelocity));
    memset(pose.vecAngularVelocity, 0, sizeof(pose.vecAngularVelocity));

    return pose;
}
