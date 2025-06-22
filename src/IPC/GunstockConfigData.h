// GunstockConfigData.h
#pragma once
//#include <openvr.h> // Use the real one

namespace vr {
	struct HmdMatrix34_2_t {
		float m[3][4]; // 3x4 matrix for position and rotation
	};
}

struct GunstockConfigData {
    bool config_updated;
    vr::HmdMatrix34_2_t offset_matrix;
    // Add other settings here:
    // int dominant_hand; // 0 for left, 1 for right (example)
    // uint64_t activation_button_id; // OpenVR button ID (example)
    // bool another_setting; // (example)
};
