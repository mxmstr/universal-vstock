#include "openvr_driver.h"

class CExampleServerDriverProvider : public vr::IServerTrackedDeviceProvider
{
public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override
    {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        return vr::VRInitError_None;
    }
    virtual void Cleanup() override {}
    virtual const char * const *GetInterfaceVersions() override
    {
        return vr::k_InterfaceVersions;
    }
    virtual void RunFrame() override {}
    virtual bool ShouldBlockStandbyMode() override { return false; }
    virtual void EnterStandby() override {}
    virtual void LeaveStandby() override {}
};

class CExampleWatchdogDriverProvider : public vr::IVRWatchdogProvider
{
public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override
    {
        VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
        return vr::VRInitError_None;
    }
    virtual void Cleanup() override {}
};

CExampleServerDriverProvider g_serverDriverProvider;
CExampleWatchdogDriverProvider g_watchdogDriverProvider;

extern "C" __declspec(dllexport)
void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverProvider;
    }
    if (0 == strcmp(vr::IVRWatchdogProvider_Version, pInterfaceName))
    {
        return &g_watchdogDriverProvider;
    }

    if (pReturnCode)
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

    return NULL;
}
