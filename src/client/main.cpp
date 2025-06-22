#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

//#include <openvr_driver.h>
#include <openvr.h>
#include "IPCUtils.h" // Will be created in a later step
//#include "PoseUpdateData.h" // Will be created in a later step


// Placeholder for GetUserConfiguredOffset
vr::HmdMatrix34_t GetUserConfiguredOffset() {
    vr::HmdMatrix34_t offset_matrix;
    // Example: Off-hand is 10cm in front of dominant hand, rotated slightly.
    // This is an identity matrix for now (no offset).
    // Row-major order:
    // m[row][column]
    offset_matrix.m[0][0] = 1.0f; offset_matrix.m[0][1] = 0.0f; offset_matrix.m[0][2] = 0.0f; offset_matrix.m[0][3] = 0.0f; // X translation
    offset_matrix.m[1][0] = 0.0f; offset_matrix.m[1][1] = 1.0f; offset_matrix.m[1][2] = 0.0f; offset_matrix.m[1][3] = 0.0f; // Y translation
    offset_matrix.m[2][0] = 0.0f; offset_matrix.m[2][1] = 0.0f; offset_matrix.m[2][2] = 1.0f; offset_matrix.m[2][3] = 0.1f; // Z translation (10 cm forward)
    return offset_matrix;
}

// Implementation for ConvertMatrixToDriverPose
// Adapted from OpenVR examples or common utilities
void HmdMatrix_SetIdentity(vr::HmdMatrix34_t* pMatrix)
{
    pMatrix->m[0][0] = 1.f;
    pMatrix->m[0][1] = 0.f;
    pMatrix->m[0][2] = 0.f;
    pMatrix->m[0][3] = 0.f;
    pMatrix->m[1][0] = 0.f;
    pMatrix->m[1][1] = 1.f;
    pMatrix->m[1][2] = 0.f;
    pMatrix->m[1][3] = 0.f;
    pMatrix->m[2][0] = 0.f;
    pMatrix->m[2][1] = 0.f;
    pMatrix->m[2][2] = 1.f;
    pMatrix->m[2][3] = 0.f;
}

vr::HmdQuaternion2_t GetRotation(const vr::HmdMatrix34_t& matrix) {
    vr::HmdQuaternion2_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}



vr::DriverPose2_t ConvertMatrixToDriverPose(const vr::HmdMatrix34_t& matrix) {
    vr::DriverPose2_t pose = { 0 };

    pose.vecPosition[0] = matrix.m[0][3];
    pose.vecPosition[1] = matrix.m[1][3];
    pose.vecPosition[2] = matrix.m[2][3];

    pose.qRotation = GetRotation(matrix);

    // Set some defaults, can be overridden by the caller
    pose.poseIsValid = true;
    pose.result = vr::ETrackingResult2::ETrackingResult_TrackingResult_Running_OK;
    pose.deviceIsConnected = true; // Assuming if we have a pose, it's connected

    // Velocities and accelerations are not derived from the matrix alone.
    // They should be copied from the source pose (e.g., dominant hand) if needed.
    // For now, initialize to zero.
    pose.vecVelocity[0] = 0; pose.vecVelocity[1] = 0; pose.vecVelocity[2] = 0;
    pose.vecAngularVelocity[0] = 0; pose.vecAngularVelocity[1] = 0; pose.vecAngularVelocity[2] = 0;
    pose.vecAcceleration[0] = 0; pose.vecAcceleration[1] = 0; pose.vecAcceleration[2] = 0;
    pose.vecAngularAcceleration[0] = 0; pose.vecAngularAcceleration[1] = 0; pose.vecAngularAcceleration[2] = 0;

    pose.qWorldFromDriverRotation.w = 1;
    pose.qWorldFromDriverRotation.x = 0;
    pose.qWorldFromDriverRotation.y = 0;
    pose.qWorldFromDriverRotation.z = 0;

    pose.qDriverFromHeadRotation.w = 1;
    pose.qDriverFromHeadRotation.x = 0;
    pose.qDriverFromHeadRotation.y = 0;
    pose.qDriverFromHeadRotation.z = 0;


    return pose;
}

// Helper function to get the serial number of a tracked device
std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if (unRequiredBufferLen == 0)
		return "";

	char *pchBuffer = new char[unRequiredBufferLen];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;
	return sResult;
}


int main() {
    vr::EVRInitError error = vr::VRInitError_None;
    vr::IVRSystem* vr_system = vr::VR_Init(&error, vr::VRApplication_Utility);

    if (error != vr::VRInitError_None) {
        std::cerr << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(error) << std::endl;
        return 1;
    }
    std::cout << "OpenVR initialized successfully." << std::endl;

    InitializeIPC(); // Initialize shared memory

    vr::TrackedDeviceIndex_t left_controller_index = vr::k_unTrackedDeviceIndexInvalid;
    vr::TrackedDeviceIndex_t right_controller_index = vr::k_unTrackedDeviceIndexInvalid;

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
        if (vr_system->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_Controller) {
            vr::ETrackedControllerRole role = vr_system->GetControllerRoleForTrackedDeviceIndex(i);
            if (role == vr::TrackedControllerRole_LeftHand) {
                left_controller_index = i;
                std::cout << "Left controller found at index: " << i << std::endl;
            } else if (role == vr::TrackedControllerRole_RightHand) {
                right_controller_index = i;
                std::cout << "Right controller found at index: " << i << std::endl;
            }
        }
    }

    if (left_controller_index == vr::k_unTrackedDeviceIndexInvalid || right_controller_index == vr::k_unTrackedDeviceIndexInvalid) {
        std::cerr << "Controllers not found. Please ensure controllers are connected and tracked." << std::endl;
        CleanupIPC();
        vr::VR_Shutdown();
        return 1;
    }

    // For simplicity, let's assume the right hand is dominant. This could be configurable.
    vr::TrackedDeviceIndex_t dominant_hand_index = right_controller_index;
    vr::TrackedDeviceIndex_t off_hand_index = left_controller_index;

    std::cout << "Starting main loop..." << std::endl;

    bool should_exit = false;
    while (!should_exit) {
        vr::TrackedDevicePose_t tracked_poses[vr::k_unMaxTrackedDeviceCount];
        vr_system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, tracked_poses, vr::k_unMaxTrackedDeviceCount);

        if (!tracked_poses[dominant_hand_index].bPoseIsValid || !tracked_poses[off_hand_index].bPoseIsValid) {
            // std::cerr << "Dominant or off-hand pose is not valid. Skipping frame." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        vr::HmdMatrix34_t dominant_hand_pose = tracked_poses[dominant_hand_index].mDeviceToAbsoluteTracking;
        // vr::HmdMatrix34_t off_hand_pose = tracked_poses[off_hand_index].mDeviceToAbsoluteTracking; // Not directly used for calculation if overriding

        vr::VRControllerState_t controller_state;
        if (vr_system->GetControllerState(off_hand_index, &controller_state, sizeof(controller_state))) {
            bool is_gunstock_active = (controller_state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) != 0;

            if (is_gunstock_active) {
                // std::cout << "Gunstock active!" << std::endl;
                vr::HmdMatrix34_t offset_matrix = GetUserConfiguredOffset();
                vr::HmdMatrix34_t new_off_hand_pose_matrix;

                // Matrix multiplication: New Pose = Dominant Hand Pose * Offset
                // This assumes offset_matrix is the transform from dominant hand to off-hand
                new_off_hand_pose_matrix.m[0][0] = dominant_hand_pose.m[0][0] * offset_matrix.m[0][0] + dominant_hand_pose.m[0][1] * offset_matrix.m[1][0] + dominant_hand_pose.m[0][2] * offset_matrix.m[2][0];
                new_off_hand_pose_matrix.m[0][1] = dominant_hand_pose.m[0][0] * offset_matrix.m[0][1] + dominant_hand_pose.m[0][1] * offset_matrix.m[1][1] + dominant_hand_pose.m[0][2] * offset_matrix.m[2][1];
                new_off_hand_pose_matrix.m[0][2] = dominant_hand_pose.m[0][0] * offset_matrix.m[0][2] + dominant_hand_pose.m[0][1] * offset_matrix.m[1][2] + dominant_hand_pose.m[0][2] * offset_matrix.m[2][2];
                new_off_hand_pose_matrix.m[0][3] = dominant_hand_pose.m[0][0] * offset_matrix.m[0][3] + dominant_hand_pose.m[0][1] * offset_matrix.m[1][3] + dominant_hand_pose.m[0][2] * offset_matrix.m[2][3] + dominant_hand_pose.m[0][3];

                new_off_hand_pose_matrix.m[1][0] = dominant_hand_pose.m[1][0] * offset_matrix.m[0][0] + dominant_hand_pose.m[1][1] * offset_matrix.m[1][0] + dominant_hand_pose.m[1][2] * offset_matrix.m[2][0];
                new_off_hand_pose_matrix.m[1][1] = dominant_hand_pose.m[1][0] * offset_matrix.m[0][1] + dominant_hand_pose.m[1][1] * offset_matrix.m[1][1] + dominant_hand_pose.m[1][2] * offset_matrix.m[2][1];
                new_off_hand_pose_matrix.m[1][2] = dominant_hand_pose.m[1][0] * offset_matrix.m[0][2] + dominant_hand_pose.m[1][1] * offset_matrix.m[1][2] + dominant_hand_pose.m[1][2] * offset_matrix.m[2][2];
                new_off_hand_pose_matrix.m[1][3] = dominant_hand_pose.m[1][0] * offset_matrix.m[0][3] + dominant_hand_pose.m[1][1] * offset_matrix.m[1][3] + dominant_hand_pose.m[1][2] * offset_matrix.m[2][3] + dominant_hand_pose.m[1][3];

                new_off_hand_pose_matrix.m[2][0] = dominant_hand_pose.m[2][0] * offset_matrix.m[0][0] + dominant_hand_pose.m[2][1] * offset_matrix.m[1][0] + dominant_hand_pose.m[2][2] * offset_matrix.m[2][0];
                new_off_hand_pose_matrix.m[2][1] = dominant_hand_pose.m[2][0] * offset_matrix.m[0][1] + dominant_hand_pose.m[2][1] * offset_matrix.m[1][1] + dominant_hand_pose.m[2][2] * offset_matrix.m[2][1];
                new_off_hand_pose_matrix.m[2][2] = dominant_hand_pose.m[2][0] * offset_matrix.m[0][2] + dominant_hand_pose.m[2][1] * offset_matrix.m[1][2] + dominant_hand_pose.m[2][2] * offset_matrix.m[2][2];
                new_off_hand_pose_matrix.m[2][3] = dominant_hand_pose.m[2][0] * offset_matrix.m[0][3] + dominant_hand_pose.m[2][1] * offset_matrix.m[1][3] + dominant_hand_pose.m[2][2] * offset_matrix.m[2][3] + dominant_hand_pose.m[2][3];


                PoseUpdateData data_to_send;
                data_to_send.shouldUpdate = true;

                std::string off_hand_serial = GetTrackedDeviceString(vr_system, off_hand_index, vr::Prop_SerialNumber_String);
                if (off_hand_serial.length() < sizeof(data_to_send.serialNumberToUpdate)) {
                    strcpy_s(data_to_send.serialNumberToUpdate, off_hand_serial.c_str());
                } else {
                    std::cerr << "Error: Serial number too long for buffer." << std::endl;
                    // Handle error, maybe skip update or log
                }

                data_to_send.pose = ConvertMatrixToDriverPose(new_off_hand_pose_matrix);
                data_to_send.pose.poseIsValid = true;
                data_to_send.pose.result = vr::ETrackingResult2::ETrackingResult_TrackingResult_Running_OK;

                // Copy velocity and angular velocity from the dominant hand's pose
                // This makes the off-hand appear to move rigidly with the dominant hand
                data_to_send.pose.vecVelocity[0] = tracked_poses[dominant_hand_index].vVelocity.v[0];
                data_to_send.pose.vecVelocity[1] = tracked_poses[dominant_hand_index].vVelocity.v[1];
                data_to_send.pose.vecVelocity[2] = tracked_poses[dominant_hand_index].vVelocity.v[2];
                data_to_send.pose.vecAngularVelocity[0] = tracked_poses[dominant_hand_index].vAngularVelocity.v[0];
                data_to_send.pose.vecAngularVelocity[1] = tracked_poses[dominant_hand_index].vAngularVelocity.v[1];
                data_to_send.pose.vecAngularVelocity[2] = tracked_poses[dominant_hand_index].vAngularVelocity.v[2];


                WriteToSharedMemory(data_to_send);
            } else {
                // Optionally, send a "shouldUpdate = false" message if the gunstock was previously active
                // This tells the driver to revert to normal tracking for the off-hand.
                // For now, the driver might handle this by timing out if no updates are received.
                // Or, we can explicitly tell it to stop overriding.
                PoseUpdateData data_to_send;
                data_to_send.shouldUpdate = false;
                std::string off_hand_serial = GetTrackedDeviceString(vr_system, off_hand_index, vr::Prop_SerialNumber_String);
                 if (off_hand_serial.length() < sizeof(data_to_send.serialNumberToUpdate)) {
                    strcpy_s(data_to_send.serialNumberToUpdate, off_hand_serial.c_str());
                } else {
                    // log error
                }
                // No need to set pose if shouldUpdate is false, but serial number is important
                WriteToSharedMemory(data_to_send);
            }
        }

        // Handle VR Events (e.g., exit)
        vr::VREvent_t event;
        while (vr_system->PollNextEvent(&event, sizeof(event))) {
            if (event.eventType == vr::VREvent_Quit) {
                should_exit = true;
                std::cout << "VR Quit event received." << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Adjust as needed
    }

    std::cout << "Exiting main loop." << std::endl;
    CleanupIPC();
    vr::VR_Shutdown();
    return 0;
}

