//============ Copyright (c) Valve Corporation, All rights reserved. ============

#define WIN32_LEAN_AND_MEAN

#include "driver_hydra.h"
#include "driverlog.h"

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h> // for timeBeginPeriod()
#pragma comment(lib, "winmm.lib")
#endif

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

CServerDriver_Hydra g_serverDriverHydra;

//-----------------------------------------------------------------------------
// Purpose: HmdDriverFactory
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverHydra;
    }

    // watchdog causes problems with steamvr (endless startup loop), so it's disabled for now
    /*
    if (0 == strcmp(IVRWatchdogProvider_Version, pInterfaceName))
    {
    return &g_watchdogDriverHydra;
    }
    */

    if (pReturnCode)
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}


inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
    HmdQuaternion_t quat;
    quat.w = w;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    return quat;
}

// keys for use with the settings API
static const char * const k_pch_Hydra_Section = "hydra";
static const char * const k_pch_Hydra_RenderModel_String = "rendermodel";
static const char * const k_pch_Hydra_EnableIMU_Bool = "enableimu";
static const char * const k_pch_Hydra_EnableDeveloperMode_Bool = "enabledevelopermode";
static const char * const k_pch_Hydra_JoystickDeadzone_Float = "joystickdeadzone";

static void GenerateSerialNumber(char *p, int psize, int base, int controller)
{
    _snprintf(p, psize, "hydra%d_controller%d", base, controller);
}

//-----------------------------------------------------------------------------
// Purpose: Watchdog
//-----------------------------------------------------------------------------
/*
class CWatchdogDriver_Hydra : public IVRWatchdogProvider
{
public:
    CWatchdogDriver_Hydra()
    {
        m_pWatchdogThread = nullptr;
    }

    virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext);
    virtual void Cleanup();

private:
    std::thread *m_pWatchdogThread;
};

CWatchdogDriver_Hydra g_watchdogDriverHydra;

bool g_bExiting = false;

void WatchdogThreadFunction()
{
    while (!g_bExiting)
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        vr::VRWatchdogHost()->WatchdogWakeUp();
    }
}

EVRInitError CWatchdogDriver_Hydra::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    // Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to 
    // be pressed. A real driver should wait for a system button event or something else from the 
    // the hardware that signals that the VR system should start up.
    g_bExiting = false;
    m_pWatchdogThread = new std::thread(WatchdogThreadFunction);
    if (!m_pWatchdogThread)
    {
        DriverLog("Unable to create watchdog thread\n");
        return VRInitError_Driver_Failed;
    }

    return VRInitError_None;
}

void CWatchdogDriver_Hydra::Cleanup()
{
    g_bExiting = true;
    if (m_pWatchdogThread)
    {
        m_pWatchdogThread->join();
        delete m_pWatchdogThread;
        m_pWatchdogThread = nullptr;
    }

    CleanupDriverLog();
}
*/

//-----------------------------------------------------------------------------
// Purpose: Controller Driver
//-----------------------------------------------------------------------------
class CHydraControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
    bool IsActivated() const
    {
        return m_unObjectId != vr::k_unTrackedDeviceIndexInvalid;
    }

    bool HasControllerId(int nBase, int nId)
    {
        return nBase == m_nBase && nId == m_nId;
    }

    /** Process sixenseControllerData.  Return true if it's new to help caller manage sleep durations */
    bool Update(sixenseControllerData & cd)
    {
        if (m_ucPoseSequenceNumber == cd.sequence_number || !IsActivated())
            return false;
        m_ucPoseSequenceNumber = cd.sequence_number;

        UpdateTrackingState(cd);

        // Block all buttons until initial press confirms hemisphere
        if (WaitingForHemisphereTracking(cd))
            return true;

        DelaySystemButtonForChording(cd);

        UpdateControllerState(cd);

        return true;
    }


    bool IsHoldingSystemButton() const
    {
        return m_eSystemButtonState == k_eWaiting;
    }

    void ConsumeSystemButtonPress()
    {
        if (m_eSystemButtonState == k_eWaiting)
        {
            m_eSystemButtonState = k_eBlocked;
        }
    }


    // User initiated manual alignment of the coordinate system of driver_hydra with the HMD:
    //
    // The user has put two controllers on either side of her head, near the shoulders.  We
    // assume that the HMD is roughly in between them (so the exact distance apart doesn't
    // matter as long as the pose is symmetrical) and we align the HMD's coordinate system
    // using the line between the controllers (again, exact position is not important, only
    // symmetry).
    static void RealignCoordinates(CHydraControllerDriver * pHydraA, CHydraControllerDriver * pHydraB)
    {
        if (pHydraA->m_unObjectId == vr::k_unTrackedDeviceIndexInvalid)
            return;

        pHydraA->m_pAlignmentPartner = pHydraB;
        pHydraB->m_pAlignmentPartner = pHydraA;

        // Ask hydra_monitor to tell us HMD pose
        static vr::VREvent_Data_t nodata = { 0 };
        vr::VRServerDriverHost()->VendorSpecificEvent(pHydraA->m_unObjectId,
            (vr::EVREventType) (vr::VREvent_VendorSpecific_Reserved_Start + 0), nodata,
            -std::chrono::duration_cast<std::chrono::seconds>(k_SystemButtonChordingDelay).count());
    }

    // hydra_monitor called us back with the HMD information
    // (Note we should probably cache pose at the moment of the chording, but we just use current here)
    void FinishRealignCoordinates(sixenseMath::Matrix3 & matHmdRotation, sixenseMath::Vector3 & vecHmdPosition)
    {
        using namespace sixenseMath;

        CHydraControllerDriver * pHydraA = this;
        CHydraControllerDriver * pHydraB = m_pAlignmentPartner;

        if (!pHydraA || !pHydraB)
            return;

        // Assign left/right arbitrarily for a second
        Vector3 posLeft(pHydraA->m_Pose.vecPosition[0], pHydraA->m_Pose.vecPosition[1], pHydraA->m_Pose.vecPosition[2]);
        Vector3 posRight(pHydraB->m_Pose.vecPosition[0], pHydraB->m_Pose.vecPosition[1], pHydraB->m_Pose.vecPosition[2]);

        Vector3 posCenter = (posLeft + posRight) * 0.5f;
        Vector3 posDiff = posRight - posLeft;

        // Choose arbitrary controller for hint about which one is on the right:
        // Assume controllers are roughly upright, so +X vector points across body.
        Quat q1(pHydraA->m_Pose.qRotation.x, pHydraA->m_Pose.qRotation.y, pHydraA->m_Pose.qRotation.z, pHydraA->m_Pose.qRotation.w);
        Vector3 rightProbe = q1 * Vector3(1, 0, 0);
        if (rightProbe * posDiff < 0) // * is dot product
        {
            std::swap(posLeft, posRight);
            posDiff = posDiff * -1.0f;
        }

        // Find a vector pointing forward relative to the hands, so we can rotate
        // that to match forward for the head.  Use -Y by right hand rule.
        Vector3 hydraFront = posDiff ^ Vector3(0, -1, 0); // ^ is cross product
        Vector3 hmdFront = matHmdRotation * Vector3(0, 0, -1); // -Z implicitly forward

        // Project both "front" vectors onto the XZ plane (we only care about yaw,
        // because we assume the HMD space is Y up, and hydra space is also Y up,
        // assuming the base is level).
        hydraFront[1] = 0.0f;
        hmdFront[1] = 0.0f;

        // Rotation is what makes the hydraFront point toward hmdFront
        Quat rotation = Quat::rotation(hydraFront, hmdFront);

        // Adjust for the natural pose of HMD vs controllers
        Vector3 vecAlignPosition = vecHmdPosition + Vector3(0, -0.100f, -0.100f);
        Vector3 translation = vecAlignPosition - rotation * posCenter;

        // Note that it is very common for all objects from a given driver to share
        // the same world transforms, because they are in the same driver space and
        // the same world space.
        pHydraA->m_WorldFromDriverTranslation = translation;
        pHydraA->m_WorldFromDriverRotation = rotation;
        pHydraA->m_bCalibrated = true;
        pHydraB->m_WorldFromDriverTranslation = translation;
        pHydraB->m_WorldFromDriverRotation = rotation;
        pHydraB->m_bCalibrated = true;
    }

    void DebugRequest(const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        std::istringstream ss(pchRequest);
        std::string strCmd;

        ss >> strCmd;
        if (strCmd == "hydra:realign_coordinates")
        {
            // hydra_monitor is calling us back with HMD tracking information so we can
            // finish realigning our coordinate system to the HMD's
            float m[3][3], v[3];
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    // Note the transpose, because sixenseMath::Matrix3 and vr::HmdMatrix34_t disagree on row/col major
                    ss >> m[j][i];
                }
                ss >> v[i];
            }
            sixenseMath::Matrix3 matRot(m);
            sixenseMath::Vector3 matPos(v);

            FinishRealignCoordinates(matRot, matPos);
        }

        /*
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
        */
    }

    CHydraControllerDriver(int base, int n):
        m_nBase(base),
        m_nId(n),
        m_ucPoseSequenceNumber(0),
        m_eHemisphereTrackingState(k_eHemisphereTrackingDisabled),
        m_bCalibrated(false),
        m_pAlignmentPartner(NULL),
        m_eSystemButtonState(k_eIdle),
        m_unObjectId(vr::k_unTrackedDeviceIndexInvalid),
        m_ulPropertyContainer(vr::k_ulInvalidPropertyContainer)
    {
        char buf[1024];
        GenerateSerialNumber(buf, sizeof(buf), base, n);
        m_sSerialNumber = buf;
        m_sModelNumber = "ViveMV"; //"Hydra";
        m_sManufacturerName = "HTC"; //"Razer";

        memset(&m_ControllerState, 0, sizeof(m_ControllerState));
        memset(&m_Pose, 0, sizeof(m_Pose));
        m_Pose.result = vr::TrackingResult_Calibrating_InProgress;

        sixenseControllerData cd;
        sixenseGetNewestData(m_nId, &cd);
        m_firmware_revision = cd.firmware_revision;
        m_hardware_revision = cd.hardware_revision;

        DriverLog("Using settings values\n");

        // assign rendermodel
        vr::VRSettings()->GetString(k_pch_Hydra_Section, k_pch_Hydra_RenderModel_String, buf, sizeof(buf));
        m_sRenderModel = buf;

        // enable IMU emulation
        m_bEnableIMUEmulation = vr::VRSettings()->GetBool(k_pch_Hydra_Section, k_pch_Hydra_EnableIMU_Bool);
        m_bEnableAngularVelocity = true;

        // set joystick deadzone
        m_fJoystickDeadzone = vr::VRSettings()->GetFloat(k_pch_Hydra_Section, k_pch_Hydra_JoystickDeadzone_Float);

        // enable developer features
        m_bEnableDeveloperMode = vr::VRSettings()->GetBool(k_pch_Hydra_Section, k_pch_Hydra_EnableDeveloperMode_Bool);

        // "Hold Thumbpad" mode (not user configurable)
        m_bEnableHoldThumbpad = true;
    }

    virtual ~CHydraControllerDriver()
    {
    }

    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {
        DriverLog("Activated device: %s (object id %d)\n", GetSerialNumber().c_str(), unObjectId);
        m_unObjectId = unObjectId;

        g_serverDriverHydra.LaunchHydraMonitor();

        // Set properties
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, m_sSerialNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sRenderModel.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ManufacturerName_String, m_sManufacturerName.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingFirmwareVersion_String, "cd.firmware_revision=" + m_firmware_revision);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_HardwareRevision_String, "cd.hardware_revision=" + m_hardware_revision);
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_FirmwareVersion_Uint64, m_firmware_revision);
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_HardwareRevision_Uint64, m_hardware_revision);

        // probably not needed
        //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);
        //vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        //vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
        //vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
        //vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false); // avoid "not fullscreen" warnings from vrmonitor
        //vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);

        // TODO
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{hydra}/input/hydra_profile.json");

        // create all the input components
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_compStart);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compBumper);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/button1/click", &m_compButton1);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/button2/click", &m_compButton2);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/button3/click", &m_compButton3);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/button4/click", &m_compButton4);
        //vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &m_compTriggerButtonEmulated);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &m_compTrigger, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &m_compJoystickAxisX, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &m_compJoystickAxisY, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &m_compJoystickButton);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &m_compJoystickTouch);
        vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

        return VRInitError_None;
    }

    virtual void Deactivate()
    {
        DriverLog("Deactivated device: %s (object id %d)\n", GetSerialNumber().c_str(), m_unObjectId);
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void EnterStandby()
    {
    }

    void *GetComponent(const char *pchComponentNameAndVersion)
    {
        // override this to add a component to a driver
        return NULL;
    }

    virtual void PowerOff()
    {
    }

    virtual DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        pose.poseIsValid = false;
        pose.result = TrackingResult_Calibrating_OutOfRange;
        pose.deviceIsConnected = true;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        return pose;
    }


    void RunFrame()
    {
#if defined( _WIN32 )
        // Your driver would read whatever hardware state is associated with its input components and pass that
        // in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
        // state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.
#endif
    }

    void ProcessEvent(const vr::VREvent_t & vrEvent)
    {
        switch (vrEvent.eventType)
        {
        case vr::VREvent_Input_HapticVibration:
        {
            if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
            {
                // This is where you would send a signal to your hardware to trigger actual haptic feedback
            }
        }
        break;
        }
    }


    std::string GetSerialNumber() const { return m_sSerialNumber; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;
    vr::ETrackedControllerRole m_eControllerRole;

    vr::VRInputComponentHandle_t m_compStart;
    vr::VRInputComponentHandle_t m_compJoystickButton;
    vr::VRInputComponentHandle_t m_compJoystickTouch;
    vr::VRInputComponentHandle_t m_compJoystickAxisX;
    vr::VRInputComponentHandle_t m_compJoystickAxisY;
    //vr::VRInputComponentHandle_t m_compTriggerButtonEmulated;
    vr::VRInputComponentHandle_t m_compTrigger;
    vr::VRInputComponentHandle_t m_compBumper;
    vr::VRInputComponentHandle_t m_compButton1;
    vr::VRInputComponentHandle_t m_compButton2;
    vr::VRInputComponentHandle_t m_compButton3;
    vr::VRInputComponentHandle_t m_compButton4;
    vr::VRInputComponentHandle_t m_compHaptic;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;
    std::string m_sManufacturerName;
    std::string m_sRenderModel;

    // Which Hydra controller
    int m_nBase;
    int m_nId;

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
    CHydraControllerDriver *m_pAlignmentPartner;

    // Timeout for system button chording
    std::chrono::steady_clock::time_point m_SystemButtonDelay;
    enum { k_eIdle, k_eWaiting, k_eSent, k_ePulsed, k_eBlocked } m_eSystemButtonState;

    // Cached for answering version queries from vrserver
    unsigned short m_firmware_revision;
    unsigned short m_hardware_revision;

    static const float k_fScaleSixenseToMeters;
    static const std::chrono::milliseconds k_SystemButtonChordingDelay;
    static const std::chrono::milliseconds k_SystemButtonPulsingDuration;

    typedef void (vr::IVRServerDriverHost::*ButtonUpdate)(uint32_t unWhichDevice, vr::EVRButtonId eButtonId, double eventTimeOffset);
    //void SendButtonUpdates(ButtonUpdate ButtonEvent, uint64_t ulMask);


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

    void UpdateControllerState(sixenseControllerData & cd)
    {
        /**
        * Handling button presses for the user switchable features go here,
        * before the driver state is updated.
        * 
        * 18/08/18 - Developer mode removed, because we want to forward all
        * button presses to the new input system from now on
        */
        /*
        // Developer mode
        if (m_bEnableDeveloperMode) {
            // Button 1 toggles imu emulation
            if (cd.buttons & SIXENSE_BUTTON_1) {
                m_bEnableIMUEmulation = !m_bEnableIMUEmulation;
                m_bHasUpdateHistory = false;
            }
            // Button 2 toggles angular velocity emulation
            if (cd.buttons & SIXENSE_BUTTON_2) {
                m_bEnableAngularVelocity = !m_bEnableAngularVelocity;
            }
        }
        */

        /**
         * "Hold Thumbpad" mode removes the joystick deadzone while Button 3 is pressed.
         * This is needed for full compatibility with the Vive's trackpad.
         * 
         * 18/08/18 - I disable this for now, we'll see if it's still needed with the new input system
         */
        float effectiveJoyDeadzone = m_fJoystickDeadzone;
        /*
        bool holdingThumbpad = false;
        if (m_bEnableHoldThumbpad && (cd.buttons & SIXENSE_BUTTON_3)) {
            effectiveJoyDeadzone = -1.0f;
            bool holdingThumbpad = true;
        }
        */

        /**
        * Update input components
        */
        // numbered buttons
        vr::VRDriverInput()->UpdateBooleanComponent(m_compButton1, cd.buttons & SIXENSE_BUTTON_1, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compButton2, cd.buttons & SIXENSE_BUTTON_2, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compButton3, cd.buttons & SIXENSE_BUTTON_3, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compButton4, cd.buttons & SIXENSE_BUTTON_4, 0);

        // bumper button
        vr::VRDriverInput()->UpdateBooleanComponent(m_compBumper, cd.buttons & SIXENSE_BUTTON_BUMPER, 0);

        // start button
        vr::VRDriverInput()->UpdateBooleanComponent(m_compStart, cd.buttons & SIXENSE_BUTTON_START, 0);

        // trigger axis
        vr::VRDriverInput()->UpdateScalarComponent(m_compTrigger, cd.trigger, 0);

        // trigger button
        // TODO?
        //vr::VRDriverInput()->UpdateBooleanComponent(m_compTriggerButtonEmulated, cd.trigger > 0.8f, 0);

        // joystick button
        vr::VRDriverInput()->UpdateBooleanComponent(m_compJoystickButton, cd.buttons & SIXENSE_BUTTON_JOYSTICK, 0);

        // joystick axis
        if (fabsf(cd.joystick_x) > effectiveJoyDeadzone || fabsf(cd.joystick_y) > effectiveJoyDeadzone)
        {
            vr::VRDriverInput()->UpdateScalarComponent(m_compJoystickAxisX, cd.joystick_x, 0);
            vr::VRDriverInput()->UpdateScalarComponent(m_compJoystickAxisY, cd.joystick_y, 0);
            vr::VRDriverInput()->UpdateBooleanComponent(m_compJoystickTouch, true, 0);
        }
        else
        {
            vr::VRDriverInput()->UpdateScalarComponent(m_compJoystickAxisX, 0, 0);
            vr::VRDriverInput()->UpdateScalarComponent(m_compJoystickAxisY, 0, 0);
            vr::VRDriverInput()->UpdateBooleanComponent(m_compJoystickTouch, cd.buttons & SIXENSE_BUTTON_JOYSTICK, 0);
        }
    }

    void UpdateTrackingState(sixenseControllerData & cd)
    {
        using namespace sixenseMath;

        // This is very hard to know with this driver, but CServerDriver_Hydra::ThreadFunc
        // tries to reduce latency as much as possible.  There is filtering in the Sixense SDK,
        // though, which causes additional unknown latency.  This time is used to know how much
        // extrapolation (via velocity and angular velocity) should be done when predicting poses.
        m_Pose.poseTimeOffset = -0.016f;

        // The "driver" coordinate system is the one that vecPosition is in.  This is whatever
        // coordinates the driver naturally produces for position and orientation.  The "world"
        // coordinate system is the one that is presented to vrserver.  This should include
        // fixing any tilt to the world (caused by a tilted camera, for example) and can include
        // any other useful transformation for the driver (e.g. the driver is tracking from a
        // secondary camera, but uses this transform to move this object into the primary camera
        // coordinate system to be consistent with other objects).
        //
        // This transform is multiplied on the left of the predicted "driver" pose.  That becomes
        // the vr::TrackingUniverseRawAndUncalibrated origin, which is then further offset for
        // floor height and tracking space center by the chaperone system to produce both the
        // vr::TrackingUniverseSeated and vr::TrackingUniverseStanding spaces.
        //
        // In the hydra driver, we use it to unify our coordinate system with the HMD.
        m_Pose.qWorldFromDriverRotation.w = m_WorldFromDriverRotation[3];
        m_Pose.qWorldFromDriverRotation.x = m_WorldFromDriverRotation[0];
        m_Pose.qWorldFromDriverRotation.y = m_WorldFromDriverRotation[1];
        m_Pose.qWorldFromDriverRotation.z = m_WorldFromDriverRotation[2];
        m_Pose.vecWorldFromDriverTranslation[0] = m_WorldFromDriverTranslation[0];
        m_Pose.vecWorldFromDriverTranslation[1] = m_WorldFromDriverTranslation[1];
        m_Pose.vecWorldFromDriverTranslation[2] = m_WorldFromDriverTranslation[2];

        // The "head" coordinate system defines a natural point for the object.  While the "driver"
        // space may be chosen for mechanical, eletrical, or mathematical convenience (e.g. being
        // the location of the IMU), the "head" should be a point meaningful to the user.  For HMDs,
        // it's the point directly between the user's eyes.  The origin of this coordinate system
        // is the origin used for the rendermodel.
        //
        // This transform is multiplied on the right side of the "driver" pose.
        //
        // This transform was inadvertently left at identity for the GDC 2015 controllers, creating
        // a defacto standard "head" position for controllers at the location of the IMU for that
        // particular controller.  We will remedy that later by adding other, explicitly named and
        // chosen spaces.  For now, mimicking that point in this driver lets us run content authored
        // for the HTC Vive Developer Edition controller.  This was done by loading an existing
        // controller rendermodel along side the Hydra model and rotating the Hydra model to roughly
        // align the main features like the handle and trigger.
        m_Pose.qDriverFromHeadRotation.w = 0.945519f;
        m_Pose.qDriverFromHeadRotation.x = 0.325568f;
        m_Pose.qDriverFromHeadRotation.y = 0.0f;
        m_Pose.qDriverFromHeadRotation.z = 0.0f;
        m_Pose.vecDriverFromHeadTranslation[0] = 0.000f;
        m_Pose.vecDriverFromHeadTranslation[1] = 0.06413f;
        m_Pose.vecDriverFromHeadTranslation[2] = -0.08695f;

        // Set position
        Vector3 pos = Vector3(cd.pos) * k_fScaleSixenseToMeters;
        m_Pose.vecPosition[0] = pos[0];
        m_Pose.vecPosition[1] = pos[1];
        m_Pose.vecPosition[2] = pos[2];

        // Angular acceleration: the Unity steamVR plugin only provides angular velocity 
        // for the controller, so probably this is not too important.
        m_Pose.vecAngularAcceleration[0] = 0.0f;
        m_Pose.vecAngularAcceleration[1] = 0.0f;
        m_Pose.vecAngularAcceleration[2] = 0.0f;

        // Set rotational coordinates
        m_Pose.qRotation.w = cd.rot_quat[3];
        m_Pose.qRotation.x = cd.rot_quat[0];
        m_Pose.qRotation.y = cd.rot_quat[1];
        m_Pose.qRotation.z = cd.rot_quat[2];

        // Update Sixense Utils data
        m_Deriv.update(&cd);

        if (!m_bEnableIMUEmulation) {
            // The tradeoff here is that setting a valid velocity causes the controllers
            // to jitter, but the controllers feel much more "alive" and lighter.
            // The jitter while stationary is more annoying than the laggy feeling caused
            // by disabling velocity (which effectively disables prediction for rendering).
            // Even the Hydra (without IMU) could probably produce a better velocity here
            // with a different filter on top of the raw position.  Perhaps someone feels
            // like writing one??
            m_Pose.vecVelocity[0] = 0.0;
            m_Pose.vecVelocity[1] = 0.0;
            m_Pose.vecVelocity[2] = 0.0;

            // True acceleration is highly volatile, so it's not really reasonable to
            // extrapolate much from it anyway.  Passing it as 0 from any driver should
            // be fine.
            m_Pose.vecAcceleration[0] = 0.0;
            m_Pose.vecAcceleration[1] = 0.0;
            m_Pose.vecAcceleration[2] = 0.0;

            // Unmeasured.  XXX with no angular velocity, throwing might not work in some games
            m_Pose.vecAngularVelocity[0] = 0.0;
            m_Pose.vecAngularVelocity[1] = 0.0;
            m_Pose.vecAngularVelocity[2] = 0.0;

        }
        else { // IMU Emulation

            int updatetime_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - m_ControllerLastUpdateTime).count();

            // Get velocity from sixense_utils
            Vector3 vel = m_Deriv.getVelocity() * k_fScaleSixenseToMeters;

            // Get acceleration from sixense_utils
            Vector3 acc = m_Deriv.getAcceleration() * k_fScaleSixenseToMeters;

            Eigen::Quaternionf rotation_ = Eigen::Quaternionf(cd.rot_quat[3], cd.rot_quat[0], cd.rot_quat[1], cd.rot_quat[2]);

            if (m_bHasUpdateHistory) {

                float expFactor_ = .1f; // smoothing factor

                                        // add sixense_utils velocity with smoothing
                m_Pose.vecVelocity[0] = expFactor_ * vel[0] + (1 - expFactor_) * m_LastVelocity[0];
                m_Pose.vecVelocity[1] = expFactor_ * vel[1] + (1 - expFactor_) * m_LastVelocity[1];
                m_Pose.vecVelocity[2] = expFactor_ * vel[2] + (1 - expFactor_) * m_LastVelocity[2];
                //DriverLog("Sixense vel: %f, %f, %f \n", vel[0], vel[1], vel[2]);

                // add sixense_utils acceleration with smoothing
                m_Pose.vecAcceleration[0] = expFactor_ * acc[0] + (1 - expFactor_) * m_LastAcceleration[0];
                m_Pose.vecAcceleration[1] = expFactor_ * acc[1] + (1 - expFactor_) * m_LastAcceleration[1];
                m_Pose.vecAcceleration[2] = expFactor_ * acc[2] + (1 - expFactor_) * m_LastAcceleration[2];
                //DriverLog("Sixense acc: %f, %f, %f \n", acc[0], acc[1], acc[2]);

                // Calculate angular velocity
                if (m_bEnableAngularVelocity) {

                    // the angular velocity's axis of rotation is the difference of the last two quats
                    Eigen::Quaternionf diff_ = m_ControllerLastRotation.conjugate() * rotation_;
                    Eigen::AngleAxisf angax_ = Eigen::AngleAxisf(diff_);
                    Eigen::Vector3f angvel_ = angax_.axis();

                    // get angular distance of current rotation from last rotation
                    float angdist_ = m_ControllerLastRotation.angularDistance(rotation_);

                    // the magnitude of the special angle/axis type vector is the speed of the rotation around the axis in rad/s
                    angvel_ = angvel_ * (angdist_ * 1000 * 1000 / updatetime_);

                    // add the calculated angular velocity with smoothing applied
                    m_Pose.vecAngularVelocity[0] = expFactor_ * angvel_.x() + (1 - expFactor_) * m_LastAngularVelocity.x();
                    m_Pose.vecAngularVelocity[1] = expFactor_ * angvel_.y() + (1 - expFactor_) * m_LastAngularVelocity.y();
                    m_Pose.vecAngularVelocity[2] = expFactor_ * angvel_.z() + (1 - expFactor_) * m_LastAngularVelocity.z();
                    //DriverLog("angvel: ad: %f, x: %f, y: %f, z: %f \n", m_ControllerLastRotation.angularDistance(rotation_), angvel_.x(), angvel_.y(), angvel_.z());

                    // refresh history
                    for (int i = 0; i < 3; i++)
                    {
                        m_LastAngularVelocity[i] = m_Pose.vecAngularVelocity[i];
                    }
                }

            }
            else {
                m_Pose.vecAcceleration[0] = .0f;
                m_Pose.vecAcceleration[1] = .0f;
                m_Pose.vecAcceleration[2] = .0f;

                m_Pose.vecVelocity[0] = .0f;
                m_Pose.vecVelocity[1] = .0f;
                m_Pose.vecVelocity[2] = .0f;

                m_Pose.vecAngularVelocity[0] = .0f;
                m_Pose.vecAngularVelocity[1] = .0f;
                m_Pose.vecAngularVelocity[2] = .0f;

                m_LastAngularVelocity[0] = .0f;
                m_LastAngularVelocity[1] = .0f;
                m_LastAngularVelocity[2] = .0f;

                m_bHasUpdateHistory = true;
            }


            // Refresh the history
            m_ControllerLastUpdateTime = std::chrono::steady_clock::now();
            m_ControllerLastRotation = rotation_;
            for (int i = 0; i < 3; i++)
            {
                m_LastVelocity[i] = m_Pose.vecVelocity[i];
                m_LastAcceleration[i] = m_Pose.vecAcceleration[i];
            }
        }

        // Don't show user any controllers until they have hemisphere tracking and
        // do the calibration gesture.  hydra_monitor should be prompting with an overlay
        if (m_eHemisphereTrackingState != k_eHemisphereTrackingEnabled)
            m_Pose.result = vr::TrackingResult_Uninitialized;
        else if (!m_bCalibrated)
            m_Pose.result = vr::TrackingResult_Calibrating_InProgress;
        else
            m_Pose.result = vr::TrackingResult_Running_OK;

        m_Pose.poseIsValid = m_bCalibrated;
        m_Pose.deviceIsConnected = true;

        // These should always be false from any modern driver.  These are for Oculus DK1-like
        // rotation-only tracking.  Support for that has likely rotted in vrserver.
        m_Pose.willDriftInYaw = false;
        m_Pose.shouldApplyHeadModel = false;

        // This call posts this pose to shared memory, where all clients will have access to it the next
        // moment they want to predict a pose.
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(DriverPose_t));
    }

    void DelaySystemButtonForChording(sixenseControllerData & cd)
    {
        // Delay sending system button to vrserver while we see if it is being
        // chorded with the other system button to reset the coordinate system
        if (cd.buttons & SIXENSE_BUTTON_START)
        {
            switch (m_eSystemButtonState)
            {
            case k_eIdle:
                m_eSystemButtonState = k_eWaiting;
                m_SystemButtonDelay = std::chrono::steady_clock::now() + k_SystemButtonChordingDelay;
                cd.buttons &= ~SIXENSE_BUTTON_START;
                break;

            case k_eWaiting:
                if (std::chrono::steady_clock::now() >= m_SystemButtonDelay)
                {
                    m_eSystemButtonState = k_eSent;
                    // leave button state set, will reach vrserver
                }
                else
                {
                    cd.buttons &= ~SIXENSE_BUTTON_START;
                }
                break;

            case k_eSent:
                // still held down, nothing to do
                break;

            case k_ePulsed:
                // user re-pressed within 1 frame, just ignore lift
                m_eSystemButtonState = k_eSent;
                break;

            case k_eBlocked:
                // was consumed by chording gesture -- never send until released
                cd.buttons &= ~SIXENSE_BUTTON_START;
                break;
            }
        }
        else
        {
            switch (m_eSystemButtonState)
            {
            case k_eIdle:
            case k_eSent:
            case k_eBlocked:
                m_eSystemButtonState = k_eIdle;
                break;

            case k_eWaiting:
                // user pressed and released the button within the timeout, so
                // send a quick pulse to the application
                m_eSystemButtonState = k_ePulsed;
                m_SystemButtonDelay = std::chrono::steady_clock::now() + k_SystemButtonPulsingDuration;
                cd.buttons |= SIXENSE_BUTTON_START;
                break;

            case k_ePulsed:
                // stretch fake pulse so client sees it
                if (std::chrono::steady_clock::now() >= m_SystemButtonDelay)
                {
                    m_eSystemButtonState = k_eIdle;
                }
                cd.buttons |= SIXENSE_BUTTON_START;
                break;
            }
        }
    }

    // Initially block all button presses, stealing the first one to mean
    // that the controller is pointing at the base and we should tell the Sixense SDK
    bool WaitingForHemisphereTracking(sixenseControllerData & cd)
    {
        switch (m_eHemisphereTrackingState)
        {
        case k_eHemisphereTrackingDisabled:
            if (cd.buttons || cd.trigger > 0.8f)
            {
                // First button press
                m_eHemisphereTrackingState = k_eHemisphereTrackingButtonDown;
            }
            return true;

        case k_eHemisphereTrackingButtonDown:
            if (!cd.buttons && cd.trigger < 0.1f)
            {
                // Buttons released (so they won't leak into application), go!
                sixenseAutoEnableHemisphereTracking(m_nId);
                m_eHemisphereTrackingState = k_eHemisphereTrackingEnabled;
            }
            return true;

        case k_eHemisphereTrackingEnabled:
        default:
            return false;
        }
    }

};

const std::chrono::milliseconds CHydraControllerDriver::k_SystemButtonChordingDelay(150);
const std::chrono::milliseconds CHydraControllerDriver::k_SystemButtonPulsingDuration(100);
const float CHydraControllerDriver::k_fScaleSixenseToMeters = 0.001;  // sixense driver in mm


//-----------------------------------------------------------------------------
// Purpose: IServerTrackedDeviceProvider
//-----------------------------------------------------------------------------
void CServerDriver_Hydra::LaunchHydraMonitor()
{
    LaunchHydraMonitor(m_sDriverInstallDir.c_str());
}

// The hydra_monitor is a companion program which can display overlay prompts for us
// and tell us the pose of the HMD at the moment we want to calibrate.
void CServerDriver_Hydra::LaunchHydraMonitor(const char * pchDriverInstallDir)
{
    if (m_bLaunchedHydraMonitor)
        return;

    m_bLaunchedHydraMonitor = true;

    std::ostringstream ss;

    ss << pchDriverInstallDir << "\\bin\\";
#if defined( _WIN64 )
    ss << "win64";
#elif defined( _WIN32 )
    ss << "win32";
#else
#error Do not know how to launch hydra_monitor
#endif
    DriverLog("hydra_monitor path: %s\n", ss.str().c_str());

#if defined( _WIN32 )
    STARTUPINFOA sInfoProcess = { 0 };
    sInfoProcess.cb = sizeof(STARTUPINFOW);
    PROCESS_INFORMATION pInfoStartedProcess;
    BOOL okay = CreateProcessA((ss.str() + "\\hydra_monitor.exe").c_str(), NULL, NULL, NULL, FALSE, 0, NULL, ss.str().c_str(), &sInfoProcess, &pInfoStartedProcess);
    DriverLog("start hydra_monitor okay: %d %08x\n", okay, GetLastError());
#else
#error Do not know how to launch hydra_monitor
#endif
}


CServerDriver_Hydra::CServerDriver_Hydra():
    m_Thread(NULL), // initialize m_Thread
    m_bStopRequested(false),
    m_bLaunchedHydraMonitor(false)
{
}

CServerDriver_Hydra::~CServerDriver_Hydra()
{
    // 10/10/2015 benj:  vrserver is exiting without calling Cleanup() to balance Init()
    // causing std::thread to call std::terminate
    Cleanup();
}

EVRInitError CServerDriver_Hydra::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    if (sixenseInit() != SIXENSE_SUCCESS)
        return vr::VRInitError_Driver_Failed;

    // Getting driver install dir from resource path
    char buf[1024];
    VRResources()->GetResourceFullPath("{hydra}", "", buf, sizeof(buf));
    m_sDriverInstallDir = buf;
    m_sDriverInstallDir += "..";

    // Will not immediately detect controllers at this point.  Sixense driver must be initializing
    // in its own thread...  It's okay to dynamically detect devices later, but if controllers are
    // the only devices (e.g. requireHmd=false) we must have GetTrackedDeviceCount() != 0 before returning.
    for (int i = 0; i < 20; ++i)
    {
        ScanForNewControllers(false);
        if (GetTrackedDeviceCount())
            break;
        Sleep(100);
    }

    m_Thread = new std::thread(ThreadEntry, this);   // use new operator now

    return VRInitError_None;
}

void CServerDriver_Hydra::ThreadEntry(CServerDriver_Hydra * pDriver)
{
    pDriver->ThreadFunc();
}

void CServerDriver_Hydra::ThreadFunc()
{
    // We know the sixense SDK thread is running at "60 FPS", but we don't know when
    // those frames are.  To minimize latency, we sleep for slightly less than the
    // target rate, and detect when the frame has not advanced to wait a bit longer.
    auto longInterval = std::chrono::milliseconds(16);
    auto retryInterval = std::chrono::milliseconds(2);
    auto scanInterval = std::chrono::seconds(1);
    auto pollDeadline = std::chrono::steady_clock::now();
    auto scanDeadline = std::chrono::steady_clock::now() + scanInterval;

#ifdef _WIN32
    // Request at least 2ms timing granularity for the life of this process
    timeBeginPeriod(2);
#endif

    while (!m_bStopRequested)
    {
        // Check for new controllers here because sixense API is modal
        // (e.g. sixenseSetActiveBase()) so it can't happen in parallel with pose updates
        if (pollDeadline > scanDeadline)
        {
            ScanForNewControllers(true);
            scanDeadline += scanInterval;
        }

        bool bAnyActivated = false;
        bool bAllUpdated = true;
        for (int base = 0; base < sixenseGetMaxBases(); ++base)
        {
            if (!sixenseIsBaseConnected(base))
                continue;

            sixenseAllControllerData acd;

            sixenseSetActiveBase(base);
            if (sixenseGetAllNewestData(&acd) != SIXENSE_SUCCESS)
                continue;
            for (int id = 0; id < sixenseGetMaxControllers(); ++id)
            {
                for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
                {
                    CHydraControllerDriver *pHydra = *it;
                    if (pHydra->IsActivated() && pHydra->HasControllerId(base, id))
                    {
                        bAnyActivated = true;
                        // Returns true if this is new data (so we can sleep for long interval)
                        if (!pHydra->Update(acd.controllers[id]))
                        {
                            bAllUpdated = false;
                        }
                        break;
                    }
                }
            }
        }

        CheckForChordedSystemButtons();

        // If everyone just got new data, we can wait about 1/60s, else try again soon
        pollDeadline += !bAnyActivated ? scanInterval :
            bAllUpdated ? longInterval : retryInterval;
        std::this_thread::sleep_until(pollDeadline);
    }

#ifdef _WIN32
    timeEndPeriod(2);
#endif
}

void CServerDriver_Hydra::CheckForChordedSystemButtons()
{
    std::vector<CHydraControllerDriver *> vecHeldSystemButtons;

    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        CHydraControllerDriver *pHydra = *it;

        if (pHydra->IsHoldingSystemButton())
        {
            vecHeldSystemButtons.push_back(pHydra);
        }
    }
    // If two or more system buttons are pressed together, treat them as a chord
    // requesting a realignment of the coordinate system
    if (vecHeldSystemButtons.size() >= 2)
    {
        if (vecHeldSystemButtons.size() == 2)
        {
            CHydraControllerDriver::RealignCoordinates(vecHeldSystemButtons[0], vecHeldSystemButtons[1]);
        }

        for (auto it = vecHeldSystemButtons.begin(); it != vecHeldSystemButtons.end(); ++it)
        {
            (*it)->ConsumeSystemButtonPress();
        }
    }
}


uint32_t CServerDriver_Hydra::GetTrackedDeviceCount()
{
    scope_lock lock(m_Mutex);

    return m_vecControllers.size();
}

CHydraControllerDriver * CServerDriver_Hydra::FindTrackedDeviceDriver(const char * pchId)
{
    scope_lock lock(m_Mutex);

    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        if (0 == strcmp((*it)->GetSerialNumber().c_str(), pchId))
        {
            return *it;
        }
    }
    return nullptr;
}

void CServerDriver_Hydra::ScanForNewControllers(bool bNotifyServer)
{
    for (int base = 0; base < sixenseGetMaxBases(); ++base)
    {
        if (sixenseIsBaseConnected(base))
        {
            sixenseSetActiveBase(base);
            for (int i = 0; i < sixenseGetMaxControllers(); ++i)
            {
                if (sixenseIsControllerEnabled(i))
                {
                    char buf[256];
                    GenerateSerialNumber(buf, sizeof(buf), base, i);
                    scope_lock lock(m_Mutex);

                    CHydraControllerDriver * hydra = FindTrackedDeviceDriver(buf);
                    if (!hydra)
                    {
                        DriverLog("Enumerated device: %s\n", buf);
                        hydra = new CHydraControllerDriver(base, i);
                        m_vecControllers.push_back(hydra);
                    }

                    if (bNotifyServer && !hydra->IsActivated())
                    {
                        DriverLog("Activating device: %s\n", buf);
                        vr::VRServerDriverHost()->TrackedDeviceAdded(hydra->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, hydra);
                    }
                }
            }
        }
    }
}

void CServerDriver_Hydra::Cleanup()
{
    CleanupDriverLog();

    if (m_Thread && m_Thread->joinable())  // test for NULL m_Thread
    {
        m_bStopRequested = true;
        m_Thread->join();
        m_Thread = NULL;  // force to NULL after thread join()
        sixenseExit();
    }

    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        delete *it;
        *it = NULL;
    }
}

void CServerDriver_Hydra::RunFrame()
{
    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        (*it)->RunFrame();
    }

    vr::VREvent_t vrEvent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
    {
        for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
        {
            (*it)->ProcessEvent(vrEvent);
        }
    }
}

