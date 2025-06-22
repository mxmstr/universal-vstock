#pragma once

#include <openvr_driver.h> // For vr::DriverPose_t
// #include <cstdint> // For uint32_t - No longer needed unless other fields use it

struct PoseUpdateData {
    bool shouldUpdate;
    // uint32_t deviceId; // To identify which tracked device this pose belongs to - REMOVED
    char serialNumberToUpdate[256]; // ADDED - To identify device by serial number
    vr::DriverPose_t pose;
};
