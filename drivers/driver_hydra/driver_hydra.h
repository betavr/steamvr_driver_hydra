#pragma once

#include <openvr_driver.h>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <mutex>
#include <atomic>
#include <sstream>
#include <SDKDDKVer.h>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <sixense.h>
#include <sixense_math.hpp>
#include <sixense_utils/derivatives.hpp>
#include <Eigen/Geometry>

class CHydraControllerDriver;

class CServerDriver_Hydra : public vr::IServerTrackedDeviceProvider
{
public:
    CServerDriver_Hydra();
    virtual ~CServerDriver_Hydra();
    virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext);
    virtual void Cleanup();
    virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame();
    virtual bool ShouldBlockStandbyMode() { return false; }
    virtual void EnterStandby() {}
    virtual void LeaveStandby() {}
    void LaunchHydraMonitor();

private:
    std::atomic<bool> m_bStopRequested;
    bool m_bLaunchedHydraMonitor;
    void LaunchHydraMonitor(const char * pchDriverInstallDir);
    std::string m_sDriverInstallDir;
    std::thread *m_Thread;
    std::recursive_mutex m_Mutex;
    typedef std::lock_guard<std::recursive_mutex> scope_lock;
    std::vector< CHydraControllerDriver * > m_vecControllers;
    static void ThreadEntry(CServerDriver_Hydra *pDriver);
    void ThreadFunc();
    void ScanForNewControllers(bool bNotifyServer);
    void CheckForChordedSystemButtons();
    virtual uint32_t GetTrackedDeviceCount();
    virtual CHydraControllerDriver * FindTrackedDeviceDriver(const char * pchId);
};
