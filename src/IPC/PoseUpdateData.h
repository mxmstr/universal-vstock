#pragma once

//#include <openvr.h>

// Temporary workaround for openvr.h compilation issues
namespace vr 
{

    enum ETrackingResult2
    {
        ETrackingResult_TrackingResult_Uninitialized = 1,
        ETrackingResult_TrackingResult_Calibrating_InProgress = 100,
        ETrackingResult_TrackingResult_Calibrating_OutOfRange = 101,
        ETrackingResult_TrackingResult_Running_OK = 200,
        ETrackingResult_TrackingResult_Running_OutOfRange = 201,
        ETrackingResult_TrackingResult_Fallback_RotationOnly = 300,
    };

    struct HmdQuaternion2_t
    {
        double w, x, y, z;
    };

    struct DriverPose2_t
    {
        /* Time offset of this pose, in seconds from the actual time of the pose,
         * relative to the time of the PoseUpdated() call made by the driver.
         */
        double poseTimeOffset;

        /* Generally, the pose maintained by a driver
         * is in an inertial coordinate system different
         * from the world system of x+ right, y+ up, z+ back.
         * Also, the driver is not usually tracking the "head" position,
         * but instead an internal IMU or another reference point in the HMD.
         * The following two transforms transform positions and orientations
         * to app world space from driver world space,
         * and to HMD head space from driver local body space.
         *
         * We maintain the driver pose state in its internal coordinate system,
         * so we can do the pose prediction math without having to
         * use angular acceleration.  A driver's angular acceleration is generally not measured,
         * and is instead calculated from successive samples of angular velocity.
         * This leads to a noisy angular acceleration values, which are also
         * lagged due to the filtering required to reduce noise to an acceptable level.
         */
        vr::HmdQuaternion2_t qWorldFromDriverRotation;
        double vecWorldFromDriverTranslation[3];

        vr::HmdQuaternion2_t qDriverFromHeadRotation;
        double vecDriverFromHeadTranslation[3];

        /* State of driver pose, in meters and radians. */
        /* Position of the driver tracking reference in driver world space
        * +[0] (x) is right
        * +[1] (y) is up
        * -[2] (z) is forward
        */
        double vecPosition[3];

        /* Velocity of the pose in meters/second */
        double vecVelocity[3];

        /* Acceleration of the pose in meters/second */
        double vecAcceleration[3];

        /* Orientation of the tracker, represented as a quaternion */
        vr::HmdQuaternion2_t qRotation;

        /* Angular velocity of the pose in axis-angle
        * representation. The direction is the angle of
        * rotation and the magnitude is the angle around
        * that axis in radians/second. */
        double vecAngularVelocity[3];

        /* Angular acceleration of the pose in axis-angle
        * representation. The direction is the angle of
        * rotation and the magnitude is the angle around
        * that axis in radians/second^2. */
        double vecAngularAcceleration[3];

        ETrackingResult2 result;

        bool poseIsValid;
        bool willDriftInYaw;
        bool shouldApplyHeadModel;
        bool deviceIsConnected;
    };
}

struct PoseUpdateData {
    bool shouldUpdate;
    // uint32_t deviceId; // To identify which tracked device this pose belongs to - REMOVED
    char serialNumberToUpdate[256]; // ADDED - To identify device by serial number
    vr::DriverPose2_t pose;
};
