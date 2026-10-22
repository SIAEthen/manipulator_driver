#include "ManipulatorDriver.h"

#include <bitset>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "ManipulatorInterface.h"
#include "ManipulatorPlanner.h"
#include "canDriver.h"
#include "defines.h"

using namespace std::chrono_literals;

extern canDriver candriver;
extern ManipulatorInterface armInterface;
extern ManipulatorPlanner armPlanner;

ManipulatorDriver::ManipulatorDriver()
    : Node("manipulator_driver")
{
  RCLCPP_INFO(this->get_logger(),
              "ROS MAIN: Try to get parameters from ROS server, parameters are can_id and can_baud_rate");
  this->declare_parameter<std::int64_t>("can_id", 0);
  this->declare_parameter<std::int64_t>("can_baud_rate", 500);
  this->get_parameter("can_id", m_canId);
  this->get_parameter("can_baud_rate", m_canBaudRate);

  this->declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4",
                      "joint5", "joint6", "joint_gripper1", "joint_gripper2"});
  this->get_parameter("joint_names", m_jointNames);
  if (m_jointNames.size() != static_cast<std::size_t>(ManipulatorState.NumOfJoint) + 2)
    throw std::invalid_argument("joint_names must contain one name per joint and two gripper names.");
  for (const auto &name : m_jointNames) {
    if (name.empty())
      throw std::invalid_argument("joint_names must not contain empty names.");
  }

  this->declare_parameter<std::vector<double>>(
      "robot_joint_offsets", std::vector<double>(ManipulatorState.NumOfJoint, 0.0));
  this->declare_parameter<std::vector<double>>(
      "robot_joint_directions", std::vector<double>(ManipulatorState.NumOfJoint, 1.0));

  this->get_parameter("robot_joint_offsets", m_robotJointOffset);
  this->get_parameter("robot_joint_directions", m_robotJointDirection);
  if (m_robotJointOffset.size() != ManipulatorState.NumOfJoint)
    throw std::invalid_argument("robot_joint_offsets must contain one value per joint.");
  if (m_robotJointDirection.size() != ManipulatorState.NumOfJoint)
    throw std::invalid_argument("robot_joint_directions must contain one value per joint.");

  this->declare_parameter<double>("robot_gripper_torque_direction", 1.0);
  this->declare_parameter<double>("robot_gripper_joint_direction", 1.0);
  this->declare_parameter<double>("robot_gripper_angle_offset", -PI/4);
  this->get_parameter("robot_gripper_torque_direction", armInterface.m_gripperTorqueDirection);
  this->get_parameter("robot_gripper_joint_direction", armInterface.m_gripperJointDirection);
  this->get_parameter("robot_gripper_angle_offset", armInterface.m_gripperAngleOffset);
  if (armInterface.m_gripperTorqueDirection != 1.0 && armInterface.m_gripperTorqueDirection != -1.0)
    throw std::invalid_argument("robot_gripper_torque_direction must be 1.0 or -1.0.");

  RCLCPP_INFO(this->get_logger(), "ROS MAIN: Get params successfully! can_id (%d), can_baud_rate (%d)",
              m_canId, m_canBaudRate);
  RCLCPP_INFO(this->get_logger(), "ROS MAIN: Try to Initialize Manipulator");
  initManipulator();
  RCLCPP_INFO(this->get_logger(), "ROS MAIN: Initialize Success!");

  auto qos = rclcpp::QoS(10)
                 .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                 .reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

  m_jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", qos);
  m_controlLoopTimer = this->create_wall_timer(8ms, std::bind(&ManipulatorDriver::controlLoopCallback, this));
  m_canLoopTimer = this->create_wall_timer(2ms, std::bind(&ManipulatorDriver::handleCanLoopCallback, this));
  m_ros2LoopTimer = this->create_wall_timer(20ms, std::bind(&ManipulatorDriver::handleROS2Loop, this));
  m_debugLoopTimer = this->create_wall_timer(5000ms, std::bind(&ManipulatorDriver::handleDebugLoop, this));

  m_initService = this->create_service<Trigger>(
      "~/motion_init",
      std::bind(&ManipulatorDriver::motionInitCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_enableService = this->create_service<Trigger>(
      "~/motion_enable",
      std::bind(&ManipulatorDriver::motionEnableCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_disableService = this->create_service<Trigger>(
      "~/motion_disable",
      std::bind(&ManipulatorDriver::motionDisableCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_closeGripperService = this->create_service<Trigger>(
      "~/close_gripper",
      std::bind(&ManipulatorDriver::closeGripperCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_openGripperService = this->create_service<Trigger>(
      "~/open_gripper",
      std::bind(&ManipulatorDriver::openGripperCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_setGripperOpenPositionService = this->create_service<Trigger>(
      "~/set_gripper_open_position",
      std::bind(&ManipulatorDriver::setGripperOpenPositionCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_setGripperClosePositionService = this->create_service<Trigger>(
      "~/set_gripper_close_position",
      std::bind(&ManipulatorDriver::setGripperClosePositionCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_clearEmergenciesService = this->create_service<Trigger>(
      "~/clear_emergencies",
      std::bind(&ManipulatorDriver::clearEmergenciesCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_debugService = this->create_service<std_srvs::srv::SetBool>(
      "~/set_debug",
      std::bind(&ManipulatorDriver::setDebugCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  auto qos_cmd = rclcpp::QoS(10).reliable().history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  m_gripperCmdSub = this->create_subscription<std_msgs::msg::Bool>(
      "~/open_gripper_cmd", qos_cmd,
      std::bind(&ManipulatorDriver::gripperCmdCallback, this, std::placeholders::_1));
  m_jointVelCmdSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_vel_rads_cmd", qos_cmd,
      std::bind(&ManipulatorDriver::jointVelCmdCallback, this, std::placeholders::_1));
  m_jointPosCmdSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_pos_rad_cmd", qos_cmd,
      std::bind(&ManipulatorDriver::jointPosCmdCallback, this, std::placeholders::_1));

  printInitSummary();
}


ManipulatorDriver::~ManipulatorDriver()
{
  if (!armInterface.enableMotionFlag) {
    RCLCPP_INFO(this->get_logger(), "Manipulator already disabled; skipping disableMotionMode.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Disabling manipulator before CAN shutdown...");
    const bool previousReadWait = canReadWaitFlag;
    canReadWaitFlag = true;
    const bool disableSucceeded = armInterface.disableMotionMode() == 1;
    canReadWaitFlag = previousReadWait;
    if (disableSucceeded)
      RCLCPP_INFO(this->get_logger(), "Manipulator disable call succeeded.");
    else
      RCLCPP_ERROR(this->get_logger(), "Manipulator disable failed; continuing CAN shutdown.");
  }
  RCLCPP_INFO(this->get_logger(), "Closing CAN channel...");
  candriver.deinitialize();
  RCLCPP_INFO(this->get_logger(), "CAN deinitialize call completed.");
}

void ManipulatorDriver::motionInitCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = true;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    if (armInterface.enableMotionFlag)
      throw std::runtime_error("Disable the manipulator before initialization.");
    if (armInterface.initMotionFlag)
      throw std::runtime_error("Init is already finished!");
    if (armInterface.initMotionMode() != 1 || !armInterface.initMotionFlag)
      throw std::runtime_error("Motion mode initialization failed.");
    resetPlanner();
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::motionEnableCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = true;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    if (armInterface.enableMotionFlag) {
      canReadWaitFlag = previousReadWait;
      return;
    }
    if (!armInterface.initMotionFlag)
      throw std::runtime_error("Call motion_init before enabling.");
    armInterface.updateState();
    resetPlanner();
    if (MANIPULATOR_MODE_DEFINE == ManipulatorModeTypeDef::ManipulatorMode_IP_Vel ||
        MANIPULATOR_MODE_DEFINE == ManipulatorModeTypeDef::ManipulatorMode_IP_Pos)
      armInterface.sendPosCmd(armPlanner.m_posCmd);
    if (armInterface.enableMotionMode() != 1)
      throw std::runtime_error("Motion mode enable failed.");
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::motionDisableCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = true;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    if (!armInterface.enableMotionFlag) {
      canReadWaitFlag = previousReadWait;
      return;
    }
    m_velCmdPublishingFlag = false;
    if (armInterface.disableMotionMode() != 1)
      throw std::runtime_error("Motion mode disable failed.");
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::closeGripperCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = false;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    requireControlEnabled();
    if (armInterface.closeGripperPDOCmd() != 1)
      throw std::runtime_error("Close gripper command failed.");
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::openGripperCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = false;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    requireControlEnabled();
    if (armInterface.openGripperPDOCmd() != 1)
      throw std::runtime_error("Open gripper command failed.");
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::setGripperOpenPositionCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = false;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    armInterface.m_gripperOpenPosition = ManipulatorState.ActualPosition[6];
    armInterface.computeGripperState();
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::setGripperClosePositionCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  const bool previousReadWait = canReadWaitFlag;
  canReadWaitFlag = false;
  response->success = true;
  response->message = "API call completed; hardware completion is not verified.";
  try {
    armInterface.m_gripperClosePosition = ManipulatorState.ActualPosition[6];
    armInterface.computeGripperState();
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error while calling the manipulator API.";
  }
  canReadWaitFlag = previousReadWait;
}

void ManipulatorDriver::clearEmergenciesCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
{
  for (uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i)
    ManipulatorState.Emergency[i] = {};
  response->success = true;
  response->message = "Cached emergency codes cleared; drive faults were not reset.";
}

void ManipulatorDriver::setDebugCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  m_debugEnabled = request->data;
  response->success = true;
  response->message = m_debugEnabled ? "Debug output enabled." : "Debug output disabled.";
}

void ManipulatorDriver::requireControlEnabled() const
{
  if (!armInterface.enableMotionFlag)
    throw std::runtime_error("Control is not enabled!");
}

void ManipulatorDriver::resetPlanner()
{
  m_velCmdPublishingFlag = false;
  double zero_velocity[6] = {0};
  armPlanner.setDesiredVelocity(zero_velocity);
  for (unsigned char i = 0; i < ManipulatorState.NumOfJoint; ++i)
    armPlanner.m_velCmd[i] = 0.0;

  armPlanner.init();
}

void ManipulatorDriver::initManipulator()
{
  
  canReadWaitFlag = true; //马上要进行SDO的通信了，因此我要用readWait，等待接收驱动器的回复信息
  RCLCPP_INFO(this->get_logger(), "canReadWaitFlag: Set to true for SDO");
  candriver.setParam(m_canId, m_canBaudRate);
  if(candriver.initialize()){
    RCLCPP_INFO(this->get_logger(), "InitManipulator: Can driver init success");
  }else{
    throw std::runtime_error("InitManipulator: Can driver init failed.");
  }
  RCLCPP_INFO(this->get_logger(), "InitManipulator: Try to read 7 motors ratedCurrent");
  if(armInterface.readMotorRatedCurrent()!=1)
    throw std::runtime_error("readMotorRatedCurrent failed.");
  for (uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i) {
    RCLCPP_INFO(this->get_logger(), "Motor %u ratedCurrent=%u mA",
                static_cast<unsigned>(i + 1),
                static_cast<unsigned>(armInterface.m_ratedCurrent[i]));
  }
  
  RCLCPP_INFO(this->get_logger(), "InitManipulator: Try to init motion mode");
  switch (MANIPULATOR_MODE_DEFINE) {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
      RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint IP Vel Mode, gripper PT"); break;
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
      RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint IP Pos Mode, gripper PT"); break;
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
      RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint PV Mode, gripper PT"); break;
    default:
      RCLCPP_ERROR(this->get_logger(), "MANIPULATOR_MODE_DEFINE not defined"); break;
  }
  if (armInterface.initMotionMode() != 1)
    throw std::runtime_error("Motion mode initialization failed.");
  armPlanner.init();
  RCLCPP_INFO(this->get_logger(), "Motion Mode init success!");

  RCLCPP_INFO(this->get_logger(), "Try to enable motion...");
  if (armInterface.enableMotionMode() != 1)
    throw std::runtime_error("Motion mode enable failed.");
  RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint 1-6 enabled");
  canReadWaitFlag = false;
  RCLCPP_INFO(this->get_logger(), "canReadWaitFlag: Set to false for PDO");

}

void ManipulatorDriver::controlLoopCallback()
{
  if (armInterface.enableMotionFlag || armInterface.initMotionFlag){
    armInterface.updateState();
    armInterface.sendFdbRequest();
  }
  if(armInterface.enableMotionFlag){
    armPlanner.run();
  }
  return;
}

void ManipulatorDriver::handleCanLoopCallback()
{
  armInterface.handleCANFdb();
}

void ManipulatorDriver::handleROS2Loop()
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = this->now();
  joint_state.name = m_jointNames;
  joint_state.position.resize(ManipulatorState.NumOfJoint + 2, 0.0);
  joint_state.velocity.resize(ManipulatorState.NumOfJoint + 2, 0.0);
  joint_state.effort.resize(ManipulatorState.NumOfJoint + 2, 0.0);

  for (unsigned char i = 0; i < ManipulatorState.NumOfJoint; ++i) {
    joint_state.position[i] =
        m_robotJointDirection[i] * (armInterface.m_jointposition[i] - m_robotJointOffset[i]);
    joint_state.velocity[i] = m_robotJointDirection[i] * armInterface.m_jointvelocity[i];
    joint_state.effort[i] =
        m_robotJointDirection[i] * armInterface.m_jointcurrent[i] * armInterface.m_driverGain[i] / 1000.0;
  }
  joint_state.position[6] = armInterface.m_gripperAngleOffset +
      armInterface.m_gripperJointDirection * armInterface.m_gripperAngle;
  joint_state.position[7] = -1 * joint_state.position[6];
  joint_state.effort[6] = armInterface.m_gripperCurrent;
  joint_state.effort[7] = -1 * joint_state.effort[6];
  m_jointStatePublisher->publish(joint_state);
  checkCmdTimeout(1);
}

void ManipulatorDriver::handleDebugLoop()
{
  logEmergencies();

  if (!m_debugEnabled)
    return;
  RCLCPP_INFO(this->get_logger(), "Debug cmd1 %f cmd2 %f cmd3 %f cmd4 %f cmd5 %f cmd6 %f",
              armPlanner.m_posCmd[0], armPlanner.m_posCmd[1], armPlanner.m_posCmd[2],
              armPlanner.m_posCmd[3], armPlanner.m_posCmd[4], armPlanner.m_posCmd[5]);
  // 打印每个电机的状态字
  for (uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i) {
    const std::string statusBits = std::bitset<16>(ManipulatorState.ActualStatus[i]).to_string();
    RCLCPP_INFO(this->get_logger(), "Motor %u status=0b%.4s %.4s %.4s %.4s",
                static_cast<unsigned>(i + 1),
                statusBits.c_str(), statusBits.c_str() + 4,
                statusBits.c_str() + 8, statusBits.c_str() + 12);
  }
}

void ManipulatorDriver::logEmergencies() const
{
  for (uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i) {
    const auto &emergency = ManipulatorState.Emergency[i];
    if (emergency.errorCode == 0)
      continue;

    RCLCPP_ERROR(this->get_logger(),
                 "CANopen emergency Motor %u: code=0x%04X register=0x%02X elmo=0x%02X data1=0x%04X data2=0x%04X",
                 static_cast<unsigned>(i + 1),
                 static_cast<unsigned>(emergency.errorCode),
                 static_cast<unsigned>(emergency.errorRegister),
                 static_cast<unsigned>(emergency.elmoErrorCode),
                 static_cast<unsigned>(emergency.errorCodeDataField1),
                 static_cast<unsigned>(emergency.errorCodeDataField2));
  }
}

void ManipulatorDriver::gripperCmdCallback(const std_msgs::msg::Bool::SharedPtr msg) const
{
  if (!armInterface.enableMotionFlag) {
    RCLCPP_WARN(this->get_logger(), "Gripper command ignored: manipulator is disabled.");
    return;
  }

  if (msg->data) {
    if (armInterface.openGripperPDOCmd() != 1) {
      RCLCPP_ERROR(this->get_logger(), "Open gripper command failed.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Open Gripper with current %d (per thousand rated torque)", GripperTorque);
  } else {
    if (armInterface.closeGripperPDOCmd() != 1) {
      RCLCPP_ERROR(this->get_logger(), "Close gripper command failed.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Close Gripper with current %d (per thousand rated torque)", GripperTorque);
  }
}

void ManipulatorDriver::jointVelCmdCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!m_velCmdPublishingFlag)
    m_velCmdPublishingFlag = true;

  double cmd[6] = {0};
  for (unsigned char i = 0; i < armInterface.m_jointnum; ++i)
    cmd[i] = msg->data.at(i)*armInterface.m_jointDirection[i]*m_robotJointDirection[i];
  armPlanner.setDesiredVelocity(cmd);

  m_lastVelCmdTime = std::chrono::steady_clock::now();
  if(m_debugEnabled){
    RCLCPP_INFO(this->get_logger(), "joint Velocity Cmd cmd1 %.4f cmd2 %.4f cmd3 %.4f cmd4 %.4f cmd5 %.4f cmd6 %.4f",
              cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
  }
  
}

void ManipulatorDriver::jointPosCmdCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  double cmd[6] = {0};
  for (unsigned char i = 0; i < armInterface.m_jointnum; ++i) {
    cmd[i] = m_robotJointDirection[i] * msg->data.at(i) + m_robotJointOffset[i];
  }
  armPlanner.setDesiredPosition(cmd);

  RCLCPP_INFO(this->get_logger(), "joint Position Cmd cmd1 %f cmd2 %f cmd3 %f cmd4 %f cmd5 %f cmd6 %f",
              cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
}

void ManipulatorDriver::checkCmdTimeout(double timeout_sec)
{
  if (!m_velCmdPublishingFlag)
    return;
  if (std::chrono::steady_clock::now() - m_lastVelCmdTime > std::chrono::duration<double>(timeout_sec)) {
    m_velCmdPublishingFlag = false;
    RCLCPP_WARN(this->get_logger(), "ROS2 vel cmd timeout, set desired velocity to 0!");
    double velocity[6] = {0};
    armPlanner.setDesiredVelocity(velocity);
  }
}

void ManipulatorDriver::printInitSummary() const
{
  RCLCPP_INFO(this->get_logger(), "Manipulator driver initialized and enabled: %s",
              this->get_fully_qualified_name());
  RCLCPP_INFO(this->get_logger(), "CAN channel %d, baud rate %d kbit/s; %u arm joints and one torque-controlled gripper.",
              m_canId, m_canBaudRate, static_cast<unsigned>(ManipulatorState.NumOfJoint));
  const char *planningMode = "unsupported";
  const char *commandPath = "no command path configured";
  const char *commandTopic = "none";
  switch (MANIPULATOR_MODE_DEFINE) {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
      planningMode = "IP velocity planning";
      commandPath = "rad/s -> acceleration limiting -> position integration -> IP position PDO";
      commandTopic = m_jointVelCmdSub->get_topic_name();
      break;
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
      planningMode = "IP position planning";
      commandPath = "rad -> position planning -> IP position PDO";
      commandTopic = m_jointPosCmdSub->get_topic_name();
      break;
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
      planningMode = "PV velocity control";
      commandPath = "rad/s -> velocity limiting -> PV target velocity PDO; drive handles profile acceleration/deceleration";
      commandTopic = m_jointVelCmdSub->get_topic_name();
      break;
    default:
      break;
  }
  RCLCPP_INFO(this->get_logger(), "Configured planning mode: %s (MANIPULATOR_MODE_DEFINE=%d).",
              planningMode, MANIPULATOR_MODE_DEFINE);
  RCLCPP_INFO(this->get_logger(), "Command path: %s.", commandPath);
  RCLCPP_INFO(this->get_logger(), "Joint command topic for this mode: %s.", commandTopic);
  RCLCPP_INFO(this->get_logger(), "Timers: control 125 Hz, CAN receive 500 Hz, joint states 50 Hz.");
  RCLCPP_INFO(this->get_logger(), "Gripper command: %s (true=open, false=close); joint feedback: %s.",
              m_gripperCmdSub->get_topic_name(), m_jointStatePublisher->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Services under %s/: motion_init, motion_enable, motion_disable, open_gripper, close_gripper.",
              this->get_fully_qualified_name());
  RCLCPP_INFO(this->get_logger(), "Calibration services: set_gripper_open_position, set_gripper_close_position; calibration is stored in memory.");
  RCLCPP_INFO(this->get_logger(), "Diagnostics: clear_emergencies clears cached errors; set_debug enables command and motor status logs every 5 s (default off).");
}
