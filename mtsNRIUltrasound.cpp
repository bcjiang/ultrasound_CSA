// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

#include <sawNRIResearchKit/mtsNRITeleOperation.h>
#include <sawNRIResearchKit/mtsNRIResearchKitPSM.h>
#include <sawNRIResearchKit/mtsNRIResearchKitProxySlave.h>
#include <sawNRIResearchKit/mtsNRIResearchKitMTM.h>
#include <sawATIForceSensor/mtsATINetFTSensor.h>
#include <sawATIForceSensor/mtsATINetFTQtWidget.h>
#include <sawTextToSpeech/mtsTextToSpeech.h>

#include <QApplication>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <dvrk_utilities/dvrk_console.h>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    // cmnLogger::SetMaskClass("mtsIntuitiveResearchKitMTM", CMN_LOG_ALLOW_ALL);
    // cmnLogger::SetMaskClass("mtsIntuitiveResearchKitPSM", CMN_LOG_ALLOW_ALL);
    // cmnLogger::SetMaskClass("mtsIntuitiveResearchKitPSMOptimizer", CMN_LOG_ALLOW_DEBUG);
    // cmnLogger::SetMaskClass("mtsVFAdmittance", CMN_LOG_ALLOW_DEBUG);
    // cmnLogger::SetMaskClass("mtsNRIResearchKitPSM", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ---- WARNING: hack to remove ros args ----
    ros::V_string argout;
    ros::removeROSArgs(argc, argv, argout);
    argc = argout.size();
    // ------------------------------------------


    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();


    // Teleoperation : Make sure to add this to component manager before console is created.
    const std::string teleName = masterName + "-" + slaveName;
    mtsNRITeleOperation * tele = new mtsNRITeleOperation(teleName, 2.0 * cmn_ms);
    //    tele->Configure();

    //    // Default orientation between master and slave
    //    vctMatRot3 master2slave;
    //    master2slave.Assign(-1.0, 0.0, 0.0,
    //                        0.0,-1.0, 0.0,
    //                        0.0, 0.0, 1.0);
    //    tele->SetRegistrationRotation(master2slave);
    componentManager->AddComponent(tele);

    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    fileExists("console JSON configuration file", jsonMainConfigFile);
    console->Configure(jsonMainConfigFile);
    componentManager->AddComponent(console);
    console->Connect();

    // add all Qt widgets
    QApplication application(argc, argv);
    mtsIntuitiveResearchKitConsoleQt * consoleQt = new mtsIntuitiveResearchKitConsoleQt();
    consoleQt->Configure(console);
    consoleQt->Connect();

    // Custom Components
    // Text-to-speech
    mtsTextToSpeech * textToSpeech = new mtsTextToSpeech;
    textToSpeech->AddInterfaceRequiredForEventString("ErrorMsg", "ErrorMsg");
    textToSpeech->AddInterfaceRequiredForEventString("TextToSpeech", "TextToSpeech");
    textToSpeech->SetPreemptive(true);
    componentManager->AddComponent(textToSpeech);

    // Proxy Slave
    proxySlaveName = "/proxy_" + slaveName;
    mtsNRIResearchKitProxySlave *proxySlave = new mtsNRIResearchKitProxySlave(proxySlaveName, 1.0*cmn_ms);
    fileExists("Rob File for slave proxy", proxySlaveKinFile);
    proxySlave->Configure(proxySlaveKinFile);
    componentManager->AddComponent(proxySlave);

    // ATI Force Sensor
    std::string ftip = "192.168.1.1";
    mtsATINetFTSensor * forceSensor = new mtsATINetFTSensor("ForceSensor");       // Continuous
    forceSensor->SetIPAddress(ftip);      // IP address of the FT sensor
    //    forceSensor->Configure(ftConfig, true, 20001);
    forceSensor->Configure(ftConfig, 10 * cmn_ms);
    componentManager->AddComponent(forceSensor);

    mtsATINetFTQtWidget * forceSensorGUI = new mtsATINetFTQtWidget("ATINetFTGUI");
    componentManager->AddComponent(forceSensorGUI);
    consoleQt->addTab(forceSensorGUI, "FT Sensor");

    // Custom Connections
    componentManager->Connect(proxySlaveName, "SlaveRobot", slaveName, "Robot");
    componentManager->Connect(textToSpeech->GetName(), "TextToSpeech", teleName, "Setting");
    componentManager->Connect(teleName, "ProxySlave", proxySlave->GetName(), "Robot");
    componentManager->Connect(slaveName, "RequiresATINetFTSensor", "ForceSensor", "ProvidesATINetFTSensor");
    componentManager->Connect(textToSpeech->GetName(), "ErrorMsg", "ForceSensor", "ProvidesATINetFTSensor");
    componentManager->Connect("ATINetFTGUI", "RequiresATINetFTSensor", "ForceSensor", "ProvidesATINetFTSensor");

    //-------------------------------------------------------
    // Start ROS Bridge
    // ------------------------------------------------------
    // ros wrapper for arms and optionally IOs
    mtsROSBridge rosBridge("dVRKBridge", rosPeriod, true);
    dvrk::console * consoleROS = new dvrk::console(rosBridge, rosNamespace, console, dvrk_topics_version::v1_4_0);
    std::string masterNameSpace = rosNamespace  + masterName;
    std::string slaveNameSpace = rosNamespace  + slaveName;
    std::string proxySlaveNameSpace = rosNamespace + proxySlaveName;
    std::string teleopNameSpace = rosNamespace  + masterName + "_" + slaveName;
    std::string forceSensorNameSpace = rosNamespace  + forceSensor->GetName();

    // MTM
    rosBridge.AddPublisherFromCommandRead<mtsDoubleVec, geometry_msgs::WrenchStamped>(
                masterName, "GetVFWrench", masterNameSpace + "/wrench_without_frameid");
    rosBridge.AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>(
                masterName, "SetWrench", masterNameSpace + "/set_wrench");

    // PSM
    rosBridge.AddPublisherFromCommandRead<mtsDoubleVec, geometry_msgs::WrenchStamped>(
                slaveName, "GetContactForce", slaveNameSpace + "/wrench");
    rosBridge.AddPublisherFromCommandRead<vctFrm4x4, geometry_msgs::Pose>(
                slaveName, "GetTipPositionCartesian", slaveNameSpace + "/cartesian_pose_tip");

    rosBridge.AddSubscriberToCommandWrite<vctFrm4x4 , geometry_msgs::Pose>(
                slaveName, "SetPositionCartesianIncrement", slaveNameSpace + "/set_cartesian_incerement");
    rosBridge.AddSubscriberToCommandWrite<vctFrm4x4 , geometry_msgs::Pose>(
                slaveName, "SetRelativeGoalPosition", slaveNameSpace + "/set_relative_goal");
    rosBridge.AddSubscriberToCommandWrite<vctFrm4x4 , geometry_msgs::Pose>(
                slaveName, "SetAbsoluteGoalPosition", slaveNameSpace + "/set_absolute_goal");
    rosBridge.AddSubscriberToCommandWrite<double , std_msgs::Float32>(
                slaveName, "SetConstantVelocity", slaveNameSpace + "/set_constant_velocity");
    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
                slaveName, "EnableReOrientation", slaveNameSpace + "/enale_reorientation");

    rosBridge.AddSubscriberToCommandWrite<vct3 , geometry_msgs::Vector3>(
                slaveName, "SetTipOffset", slaveNameSpace + "/set_tip_offset");
    rosBridge.AddSubscriberToCommandWrite<double , std_msgs::Float32>(
                slaveName, "SetContactThreshold", slaveNameSpace + "/set_contact_threshold");
    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
                slaveName, "EnableHybridForce", slaveNameSpace + "/enable_hybrid_force");
    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
                slaveName, "EnableMasterFeedback", slaveNameSpace + "/enable_master_feedback");
    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
                slaveName, "EnablePalpation", slaveNameSpace + "/enable_palpation");
    rosBridge.AddSubscriberToCommandWrite<double , std_msgs::Float32>(
                slaveName, "SetAmplitude", slaveNameSpace + "/set_sinusoid_amplitude");
    rosBridge.AddSubscriberToCommandWrite<double , std_msgs::Float32>(
                slaveName, "SetFrequency", slaveNameSpace + "/set_sinusoid_frequency");

    // Proxy Slave
    rosBridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>(
                proxySlaveName, "GetStateJoint", proxySlaveNameSpace + "/state_joint_current");

    // Teleop
    rosBridge.AddSubscriberToCommandWrite<vctDoubleVec , cisst_msgs::vctDoubleVec>(
                teleName, "SetComplianceGains", teleopNameSpace + "/set_force_gains");
    rosBridge.AddSubscriberToCommandWrite<double , std_msgs::Float32>(
                teleName, "SetSlaveLimitingForceMagnitude", teleopNameSpace + "/set_limiting_force");
    rosBridge.AddSubscriberToCommandWrite<vctDoubleVec , cisst_msgs::vctDoubleVec>(
                teleName, "SetMasterFeedbackGain", teleopNameSpace + "/set_master_feedback_gains");
    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
                teleName, "EnableVirtualTeleop", teleopNameSpace + "/enable_virtual_teleop");

    // Teleop-Model
    rosBridge.AddSubscriberToCommandWrite<std::string , std_msgs::String>(
                teleName, "ConfigureVF", teleopNameSpace + "/configure_vf");
    rosBridge.AddSubscriberToCommandWrite<prmFixtureGainCartesianSet , cisst_msgs::prmFixtureGainCartesianSet>(
                teleName, "SetCurveVFGains", teleopNameSpace + "/vfgains_curve");
    rosBridge.AddSubscriberToCommandWrite<prmFixtureGainCartesianSet , cisst_msgs::prmFixtureGainCartesianSet>(
                teleName, "SetMeshVFGains", teleopNameSpace + "/vfgains_mesh");

    // Teleop-Palpation
    //    rosBridge.AddPublisherFromCommandRead<bool, std_msgs::Bool>(
    //                teleName, "GetPalpationLogEnabled", teleopNameSpace + "/is_log_enabled");

    //    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
    //                teleName, "SetPalpationStatus", teleopNameSpace + "/is_single_probe_done");
    //    rosBridge.AddSubscriberToCommandWrite<bool , std_msgs::Bool>(
    //                teleName, "SetIsPalpationTaskEnabled", teleopNameSpace + "/enable_palpation");

    rosBridge.AddPublisherFromCommandRead<bool , std_msgs::Bool>(
                teleName, "GetMasterContactStatus", teleopNameSpace + "/get_master_contact_status");
    rosBridge.AddPublisherFromCommandRead<bool , std_msgs::Bool>(
                teleName, "GetSlaveContactStatus", teleopNameSpace + "/get_slave_contact_status");
    rosBridge.AddPublisherFromCommandRead<std::string , std_msgs::String>(
                teleName, "GetTeleopMode", teleopNameSpace + "/get_mode");
    rosBridge.AddPublisherFromCommandRead<vctFrm4x4 , geometry_msgs::Pose>(
                teleName, "GetComplianceFrame", teleopNameSpace + "/get_compliance_frame");
    rosBridge.AddPublisherFromCommandRead<bool , std_msgs::Bool>(
                teleName, "GetIsVFCurveEnabled", teleopNameSpace + "/is_vfcurve_enabled");

    rosBridge.AddPublisherFromCommandRead<vctFrm4x4 , geometry_msgs::Pose>(
                teleName, "GetSlavePhantomRegistration", teleopNameSpace + "/slave_phantom_registration");
    rosBridge.AddPublisherFromCommandRead<vctFrm4x4 , geometry_msgs::Pose>(
                teleName, "GetProxySlavePhantomRegistration", teleopNameSpace + "/proxy_slave_phantom_registration");
    rosBridge.AddPublisherFromCommandRead<vctDoubleMat , sensor_msgs::PointCloud>(
                teleName, "GetCurvePoints", teleopNameSpace + "/proxy_slave_phantom_curve");
    rosBridge.AddPublisherFromCommandRead<prmFixtureGainCartesianSet , cisst_msgs::prmFixtureGainCartesianSet>(
                teleName, "GetImpedanceVFGains", teleopNameSpace + "/master_vf_gains");

    // ATI Force Sensor
    rosBridge.AddPublisherFromCommandRead<mtsDoubleVec, geometry_msgs::WrenchStamped>(
                forceSensor->GetName(), "GetFTData", forceSensorNameSpace + "/raw_wrench");


    // Component manager connections
    componentManager->AddComponent(&rosBridge);
    consoleROS->Connect();
    componentManager->Connect(rosBridge.GetName(), proxySlaveName, proxySlaveName, "Robot");
    componentManager->Connect(rosBridge.GetName(), forceSensor->GetName(), forceSensor->GetName(), "ProvidesATINetFTSensor");
    //-------------------------------------------------------
    // End ROS Bridge
    // ------------------------------------------------------


    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    application.exec();

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // stop all logs
    cmnLogger::Kill();

    return 0;
}