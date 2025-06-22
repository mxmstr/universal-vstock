#include <iostream>
#include <string>
#include <vector>
//#include <openvr.h> // For HmdMatrix34_t
#include "../IPC/IPCUtils.h" // Adjusted path
#include "../IPC/GunstockConfigData.h" // Adjusted path

// Placeholder for getting user-configured offset from a UI
// For now, it returns a hardcoded offset.
vr::HmdMatrix34_2_t GetUserConfiguredOffsetFromUI() {
    vr::HmdMatrix34_2_t offset_matrix;
    // Initialize to identity
    offset_matrix.m[0][0] = 1.0f; offset_matrix.m[0][1] = 0.0f; offset_matrix.m[0][2] = 0.0f; offset_matrix.m[0][3] = 0.0f;
    offset_matrix.m[1][0] = 0.0f; offset_matrix.m[1][1] = 1.0f; offset_matrix.m[1][2] = 0.0f; offset_matrix.m[1][3] = 0.0f;
    offset_matrix.m[2][0] = 0.0f; offset_matrix.m[2][1] = 0.0f; offset_matrix.m[2][2] = 1.0f; offset_matrix.m[2][3] = 0.0f;

    // Example: Off-hand is 10cm in front of dominant hand (Z-axis)
    // and 5cm down (Y-axis, assuming OpenVR coordinate system where +Y is up)
    offset_matrix.m[2][3] = 0.1f;  // 10cm forward
    // offset_matrix.m[1][3] = -0.05f; // 5cm down (example)
    // offset_matrix.m[0][3] = 0.02f;  // 2cm to the right (example)

    // Add rotation if needed, e.g. rotate 45 degrees around Y axis
    // float angle = 45.0f * 3.14159265f / 180.0f; // 45 degrees in radians
    // offset_matrix.m[0][0] = cos(angle); offset_matrix.m[0][2] = sin(angle);
    // offset_matrix.m[2][0] = -sin(angle); offset_matrix.m[2][2] = cos(angle);

    std::cout << "Using hardcoded offset: 10cm forward." << std::endl;
    // In a real app, this would come from text boxes, sliders, etc.
    // And you might want to allow configuring dominant hand, activation button, etc.
    return offset_matrix;
}

int main() {
    std::cout << "Virtual Gunstock Configuration Utility" << std::endl;
    std::cout << "Initializing IPC..." << std::endl;
    InitializeIPC();

    // This simulates a user clicking "Apply Settings" in a UI.
    // In a real application, this would be triggered by a UI event.
    std::cout << "Preparing to send configuration to the driver..." << std::endl;

    GunstockConfigData config_to_send;
    config_to_send.config_updated = true;
    config_to_send.offset_matrix = GetUserConfiguredOffsetFromUI();

    // Populate other settings if they were added to GunstockConfigData
    // config_to_send.dominant_hand = 1; // 0 for left, 1 for right
    // config_to_send.activation_button_id = vr::k_EButton_Grip; // Example button

    std::cout << "Sending configuration..." << std::endl;
    WriteToSharedMemory(config_to_send);
    std::cout << "Configuration sent to driver." << std::endl;

    // In a real UI app, the app would remain open, allowing users to change settings.
    // For this simplified example, we send once and then wait for user to close.
    std::cout << "Client will now exit. The driver will use the sent configuration." << std::endl;
    std::cout << "Press Enter to close this utility..." << std::endl;
    std::cin.get(); // Wait for user to press Enter

    std::cout << "Cleaning up IPC..." << std::endl;
    CleanupIPC();
    std::cout << "Exiting." << std::endl;
    return 0;
}
