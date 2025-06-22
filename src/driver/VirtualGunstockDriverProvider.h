#pragma once

#include "openvr_driver.h" // Standard OpenVR driver header
#include <vector>
#include <memory>
#include "VirtualDevice.h"
#include "../IPC/GunstockConfigData.h" // Include the new config data struct

class VirtualGunstockDriverProvider : public vr::IServerTrackedDeviceProvider
{
public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;
    virtual void Cleanup() override;
    virtual const char * const *GetInterfaceVersions() override;
    virtual void RunFrame() override;
    virtual bool ShouldBlockStandbyMode() override;
    virtual void EnterStandby() override;
    virtual void LeaveStandby() override;

private:
    std::vector<std::unique_ptr<VirtualDevice>> m_vecVirtualDevices;

    // Handles for IVRDriverInput
    vr::VRInputComponentHandle_t m_hRightHandPose;
    vr::VRInputComponentHandle_t m_hLeftHandPose;
    vr::VRInputComponentHandle_t m_hOffHandGrip; // To check for activation

    // Stored Configuration
    GunstockConfigData m_config;
};
