#pragma once

#include "openvr_driver.h"
#include "VirtualDevice.h" // Added
#include <vector>          // Added
#include <memory>          // Added

class VirtualGunstockDriverProvider : public vr::IServerTrackedDeviceProvider
{
public: // It's common to put public members after private, but the issue implies it's here.
    std::vector<std::unique_ptr<VirtualDevice>> m_vecVirtualDevices; // Added

public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;
    virtual void Cleanup() override;
    virtual const char * const *GetInterfaceVersions() override;
    virtual void RunFrame() override;
    virtual bool ShouldBlockStandbyMode() override;
    virtual void EnterStandby() override;
    virtual void LeaveStandby() override;
};
