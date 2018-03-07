#ifndef _mtsNRITeleOperation_h
#define _mtsNRITeleOperation_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>

#include <sawNRIResearchKit/SlavePositionPayload.h>
#include <sawNRIResearchKit/SlaveVelocityPayload.h>
#include <sawNRIResearchKit/SlaveForcePayload.h>

#include <sawNRIResearchKit/ConfigParser.h>
#include <sawNRIResearchKit/mtsVFModel.h>
#include <sawNRIResearchKit/mtsTeleopModes.h>

typedef mtsTeleOperationPSM TeleopBaseType;

class mtsNRITeleOperation: public TeleopBaseType
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsNRITeleOperation(const std::string & componentName, const double periodInSeconds);
    mtsNRITeleOperation(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsNRITeleOperation(){}

    void Configure(const std::string & fileName = "");
    void ConfigureVF(const std::string & fileName);
    void UpdateNRIRobotStates(void);
    void UpdateContactStates(void);
    void UpdateOperationMode(void);

    void ComputeSlaveGoal(vctFrm4x4 &slaveGoal);
    void RunTeleop();
    void SendSlaveGoal(vctFrm4x4 &slaveGoal);
    
    void GetOperationMode(std::string & mode) const;
    void EnableVirtualTeleop(const bool & enable);
    void SendFeedbackToMaster(void);
    void SetConfigFile(const std::string & fileName){
        VFModelConfigFile = fileName;
    }
    void VirtualTeleopEventHandler(const prmEventButton &button);

public:
    class NRIRobotPSM : public RobotPSM {
    public:
        mtsFunctionRead GetTipOffset;
        mtsFunctionRead GetPositionCartesianDesired;
        mtsFunctionRead GetContactForce;
        mtsFunctionRead GetContactState;
        mtsFunctionRead GetPhantomRegistration;
        mtsFunctionRead GetTipCartesianVelocity;
        mtsFunctionRead GetIsInsideBounds;

        mtsFunctionWrite SetPositionCartesianIncrement;
        mtsFunctionWrite SetIsPalpationTaskEnabled;
        mtsFunctionWrite EnableForceCompliance;
        mtsFunctionWrite EnableFullMotion;
        mtsFunctionWrite SetLimitingForce;
        mtsFunctionWrite SetLimitingForceDirection;

        prmPositionCartesianGet PreviousCartesianDesiredGet;
        prmPositionCartesianSet PositionCartesianDesired;
        prmPositionCartesianGet CartesianDesiredGet;
        vctFrm4x4 CurrentTipPose;
        vctFrm4x4 PrevTipPose;
        bool InContact;
        bool FixtureEnabled;
        double LimitingForceMag;
        bool IsInsideBounds;
        vctDoubleVec ContactForce;
        vctFrm4x4 Frame7ToTip;
        vct3 CartesianTipVelocity;
    };

    class NRIRobotMTM : public RobotMTM {
    public:
        mtsFunctionWrite SetImpedanceForceTorque;
        mtsFunctionWrite SetImpedanceGains;
        mtsFunctionVoid EnableImpedanceVirtualFixture;
        mtsFunctionVoid DisableImpedanceVirtualFixture;
        mtsFunctionRead GetVelocityCartesian;

        prmPositionCartesianGet PositionCartesianPrevious;
        vctFrm4x4 CartesianPrevious;

        prmVelocityCartesianGet CartesianVelocityGetParam;
    };

protected:
    void InitNRI();
    void AddInterfaces(void);
    void RunEnabled(void);
    void RunAllStates(void);
    void EnterSettingArmsState(void);
    void TransitionSettingArmsState(void);
    void EnterEnabled(void);
    void TransitionEnabled(void);

    NRIRobotMTM *mMTM_NRI;
    NRIRobotPSM *mPSM_NRI;

    class NRIProxyPSM {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesianIncrement;
        mtsFunctionVoid UpdateFromSlave;

        prmPositionCartesianGet PositionCartesianCurrent;
        vctFrm4x4 CurrentTipPose;
        prmVelocityCartesianGet CartesianVelocity;
        bool InContact;
        vct3 ErrorVec;
    };
    NRIProxyPSM ProxySlave;

    struct {
        mtsFunctionWrite TextToSpeech;
    } NRIMessageEvents;
private:

    mtsTeleopOperationModes::TeleopMode OperationMode;
    vctFrm3 Offset;
    prmForceCartesianSet FeedbackToMaster;

    // Slave payload files
    SlavePositionPayload PositionPayload;
    SlaveVelocityPayload VelocityPayload;
    SlaveForcePayload    ForcePayload;

    prmPositionCartesianSet SlaveForceError;

    // VF
    mtsVFModel VFModel;
    ConfigParser Parser;
    std::string VFModelConfigFile;

    bool IsImpedenceVFConfigured;
    struct {
        vctDoubleVec LimitingForce;
        vctDoubleVec MasterFeedbackGain;
        vctDouble3x3 ForceComplianceGain;
        vctDouble3x3 PositionGain;
    } VFParams;

    bool SinusoidalMotionEnabled;
    bool IsPaused;
    bool UseVirtualTeleopOnly;

    // Filter imedance forces sent to master
    MovingAverageFilterDoubleVec *MAFilter;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsNRITeleOperation);

#endif // _mtsNRITeleOperation_h
