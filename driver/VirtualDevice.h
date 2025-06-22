// VirtualDevice.h
#pragma once
#include <openvr_driver.h>
#include <string>

class VirtualDevice : public vr::ITrackedDeviceServerDriver {
private:
    vr::TrackedDeviceIndex_t m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    std::string m_sSerialNumber;
    vr::ETrackedDeviceClass m_eDeviceClass;

    vr::DriverPose_t m_pose; // This will hold the latest pose

public:
    VirtualDevice(std::string serialNumber, vr::ETrackedDeviceClass deviceClass);
    virtual ~VirtualDevice() = default;

    // --- ITrackedDeviceServerDriver implementation ---
    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void EnterStandby() override;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;

    // --- Helper methods ---
    std::string GetSerialNumber();
    void UpdatePose(const vr::DriverPose_t& newPose);
};
