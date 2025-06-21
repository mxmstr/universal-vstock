#pragma once

#include <openvr_driver.h> // For vr::DriverPose_t
#include <cstdint>         // For uint32_t

struct PoseUpdateData {
    bool shouldUpdate;
    uint32_t deviceId; // To identify which tracked device this pose belongs to
    vr::DriverPose_t pose;
};
