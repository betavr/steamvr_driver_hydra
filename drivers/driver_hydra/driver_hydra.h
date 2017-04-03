#pragma once

//========= Copyright Valve Corporation ============//

#include <openvr_driver.h>
#include <sixense.h>
#include <sixense_utils/derivatives.hpp>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

class CHydraHmdLatest;

class CServerDriver_Hydra : public vr::IServerTrackedDeviceProvider
{
public:
	CServerDriver_Hydra();
	virtual ~CServerDriver_Hydra();

	uint32_t GetTrackedDeviceCount();
	vr::ITrackedDeviceServerDriver * GetTrackedDeviceDriver(uint32_t unWhich);
	vr::ITrackedDeviceServerDriver * FindTrackedDeviceDriver(const char * pchId);

	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;
	virtual void Cleanup() override;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() override;

	virtual bool ShouldBlockStandbyMode() override;
	virtual void EnterStandby() override;
	virtual void LeaveStandby() override;

	std::string m_strDriverInstallDir;
	void LaunchHydraMonitor();

private:
	static void ThreadEntry( CServerDriver_Hydra *pDriver );
	void ThreadFunc();
	void ScanForNewControllers( bool bNotifyServer );
	void CheckForChordedSystemButtons();

	void LaunchHydraMonitor( const char * pchDriverInstallDir );

	vr::IVRServerDriverHost* m_pDriverHost;

	bool m_bLaunchedHydraMonitor;

	std::atomic<bool> m_bStopRequested;
	std::thread *m_Thread;   // change m_Thread to a pointer
	std::recursive_mutex m_Mutex;
	typedef std::lock_guard<std::recursive_mutex> scope_lock;
	std::vector< CHydraHmdLatest * > m_vecControllers;
};

/*
class CClientDriver_Hydra : public vr::IClientTrackedDeviceProvider
{
public:
	CClientDriver_Hydra();
	virtual ~CClientDriver_Hydra();

	// Inherited via IClientTrackedDeviceProvider
	virtual vr::EVRInitError Init(vr::EClientDriverMode eDriverMode, vr::IDriverLog * pDriverLog, vr::IClientDriverHost * pDriverHost, const char * pchUserDriverConfigDir, const char * pchDriverInstallDir ) override;
	virtual void Cleanup() override;
	virtual bool BIsHmdPresent( const char * pchUserConfigDir ) override;
	virtual vr::EVRInitError SetDisplayId( const char * pchDisplayId ) override;
	virtual vr::HiddenAreaMesh_t GetHiddenAreaMesh( vr::EVREye eEye ) override;
	virtual uint32_t GetMCImage( uint32_t *pImgWidth, uint32_t *pImgHeight, uint32_t *pChannels, void *pDataBuffer, uint32_t unBufferLen ) override;

private:
	vr::IClientDriverHost* m_pDriverHost;

};
*/

class CHydraHmdLatest : public vr::ITrackedDeviceServerDriver, public vr::IVRControllerComponent
{
public:
	CHydraHmdLatest( vr::IVRServerDriverHost * pDriverHost, int base, int n );
	virtual ~CHydraHmdLatest();

	// Implementation of vr::ITrackedDeviceServerDriver
	virtual vr::EVRInitError Activate( uint32_t unObjectId ) override;
	virtual void Deactivate() override;
	virtual void EnterStandby() override;
	void *GetComponent( const char *pchComponentNameAndVersion ) override;
	virtual void DebugRequest( const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize ) override;
	virtual vr::DriverPose_t GetPose() override;

	// Implementation of vr::IVRControllerComponent
	virtual vr::VRControllerState_t GetControllerState() override;
	virtual bool TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds ) override;

	static const vr::EVRButtonId k_EButton_Button1 = ( vr::EVRButtonId ) 7;
	static const vr::EVRButtonId k_EButton_Button2 = ( vr::EVRButtonId ) 8;
	static const vr::EVRButtonId k_EButton_Button3 = ( vr::EVRButtonId ) 9;
	static const vr::EVRButtonId k_EButton_Button4 = vr::k_EButton_ApplicationMenu;
	static const vr::EVRButtonId k_EButton_Bumper  = vr::k_EButton_Grip; // Just for demo compatibility

	bool IsActivated() const;
	bool HasControllerId( int nBase, int nId );
	bool Update( sixenseControllerData & cd );
	bool IsHoldingSystemButton() const;
	void ConsumeSystemButtonPress();
	const char *GetSerialNumber();

	static void RealignCoordinates( CHydraHmdLatest * pHydraA, CHydraHmdLatest * pHydraB );
	void FinishRealignCoordinates( sixenseMath::Matrix3 & matHmdRotation, sixenseMath::Vector3 &vecHmdPosition );

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	static const float k_fScaleSixenseToMeters;
	static const std::chrono::milliseconds k_SystemButtonChordingDelay;
	static const std::chrono::milliseconds k_SystemButtonPulsingDuration;

	typedef void ( vr::IVRServerDriverHost::*ButtonUpdate )( uint32_t unWhichDevice, vr::EVRButtonId eButtonId, double eventTimeOffset );

	void SendButtonUpdates( ButtonUpdate ButtonEvent, uint64_t ulMask );
	void UpdateControllerState( sixenseControllerData & cd );
	void UpdateTrackingState( sixenseControllerData & cd );
	void DelaySystemButtonForChording( sixenseControllerData & cd );
	bool WaitingForHemisphereTracking( sixenseControllerData & cd );

	// Handle for calling back into vrserver with events and updates
	vr::IVRServerDriverHost *m_pDriverHost;

	// Which Hydra controller
	int m_nBase;
	int m_nId;
	std::string m_strSerialNumber;

	// Used to deduplicate state data from the sixense driver
	uint8_t m_ucPoseSequenceNumber;

	// To main structures for passing state to vrserver
	vr::DriverPose_t m_Pose;
	vr::VRControllerState_t m_ControllerState;

	// Ancillary tracking state
	sixenseMath::Vector3 m_WorldFromDriverTranslation;
	sixenseMath::Quat m_WorldFromDriverRotation;
	sixenseUtils::Derivatives m_Deriv;
	enum { k_eHemisphereTrackingDisabled, k_eHemisphereTrackingButtonDown, k_eHemisphereTrackingEnabled } m_eHemisphereTrackingState;
	bool m_bCalibrated;

	// Other controller with from the last realignment
	CHydraHmdLatest *m_pAlignmentPartner;

	// Timeout for system button chording
	std::chrono::steady_clock::time_point m_SystemButtonDelay;
	enum { k_eIdle, k_eWaiting, k_eSent, k_ePulsed, k_eBlocked } m_eSystemButtonState;

	// Cached for answering version queries from vrserver
	unsigned short m_firmware_revision;
	unsigned short m_hardware_revision;

	// Assigned by vrserver upon Activate().  The same ID visible to clients
	vr::TrackedDeviceIndex_t m_unSteamVRTrackedDeviceId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	// The rendermodel used by the device. Check the contents of "c:\Program Files (x86)\Steam\steamapps\common\SteamVR\resources\rendermodels" for available models.
	std::string m_strRenderModel;

	// IMU emulation things
	std::chrono::steady_clock::time_point m_ControllerLastUpdateTime;
	Eigen::Quaternionf m_ControllerLastRotation;
	sixenseMath::Vector3 m_LastVelocity;
	sixenseMath::Vector3 m_LastAcceleration;
	Eigen::Vector3f m_LastAngularVelocity;
	bool m_bHasUpdateHistory;
	bool m_bEnableAngularVelocity;
	bool m_bEnableHoldThumbpad;

	// steamvr.vrsettings config values
	bool m_bEnableIMUEmulation;
	bool m_bEnableDeveloperMode;
	float m_fJoystickDeadzone;

};
