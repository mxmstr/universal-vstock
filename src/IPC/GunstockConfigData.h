// GunstockConfigData.h
#pragma once
#include <openvr.h> // Use the real one

struct GunstockConfigData {
    bool config_updated;
    vr::HmdMatrix34_t offset_matrix;
    // Add other settings here:
    // int dominant_hand; // 0 for left, 1 for right (example)
    // uint64_t activation_button_id; // OpenVR button ID (example)
    // bool another_setting; // (example)
};
