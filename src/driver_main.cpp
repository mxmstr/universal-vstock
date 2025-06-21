#include "openvr_driver.h"
#include "VirtualGunstockDriverProvider.h"

// Definition moved to VirtualGunstockDriverProvider.h
extern VirtualGunstockDriverProvider g_driverProvider;

extern "C" __declspec(dllexport)
void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_driverProvider; // Updated to g_driverProvider
    }
    // Removed watchdog provider section

    if (pReturnCode)
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

    return NULL;
}
