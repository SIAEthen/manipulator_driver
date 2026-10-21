#include "ManipulatorDriver.h"

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
  this->get_parameter("robot_gripper_torque_direction", armInterface.m_gripperTorqueDirection);
  this->get_parameter("robot_gripper_joint_direction", armInterface.m_gripperJointDirection);
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

  m_initService = createApiService("~/ip_pos_mode_init", [this]() {
    if (m_controlEnabled)
      throw std::runtime_error("Disable the manipulator before initialization.");
    if (ipModeFlags & IP_Flag_Init)
      throw std::runtime_error("Init is already finished!");
    if (armInterface.IPPosModeInit() != 1 || !(ipModeFlags & IP_Flag_Init))
      throw std::runtime_error("IP mode initialization failed.");
    resetPlanner();
  }, true);

  m_enableService = createApiService("~/ip_pos_mode_enable", [this]() {
    if (m_controlEnabled)
      return;
    if (!(ipModeFlags & IP_Flag_Init))
      throw std::runtime_error("Call ip_pos_mode_init before enabling.");
    armInterface.updateState();
    resetPlanner();
    armInterface.sendPosCmd(armPlanner.m_posCmd);
    if (armInterface.IPPosModeEnable() != 1)
      throw std::runtime_error("IP mode enable failed.");
    m_controlEnabled = true;
  }, true);

  m_disableService = createApiService("~/ip_pos_mode_disable", [this]() {
    if (!m_controlEnabled)
      return;
    m_controlEnabled = false;
    m_velCmdPublishingFlag = false;
    if (armInterface.IPPosModeDisable() != 1)
      throw std::runtime_error("IP mode disable failed.");
  }, true);

  m_closeGripperService = createApiService("~/close_gripper", [this]() {
    requireControlEnabled();
    if (armInterface.IPCloseGripper() != 1)
      throw std::runtime_error("Close gripper command failed.");
  }, false);

  m_openGripperService = createApiService("~/open_gripper", [this]() {
    requireControlEnabled();
    if (armInterface.IPOpenGripper() != 1)
      throw std::runtime_error("Open gripper command failed.");
  }, false);

  m_setGripperOpenPositionService = createApiService("~/set_gripper_open_position", []() {
    armInterface.m_gripperOpenPosition = ManipulatorState.ActualPosition[6];
    armInterface.computeGripperState();
  }, false);

  m_setGripperClosePositionService = createApiService("~/set_gripper_close_position", []() {
    armInterface.m_gripperClosePosition = ManipulatorState.ActualPosition[6];
    armInterface.computeGripperState();
  }, false);

  m_clearEmergenciesService = this->create_service<Trigger>(
      "~/clear_emergencies",
      [](const std::shared_ptr<Trigger::Request>,
         std::shared_ptr<Trigger::Response> response) {
        for (uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i)
          ManipulatorState.Emergency[i] = {};
        response->success = true;
        response->message = "Cached emergency codes cleared; drive faults were not reset.";
      });

  m_debugService = this->create_service<std_srvs::srv::SetBool>(
      "~/set_debug",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        m_debugEnabled = request->data;
        response->success = true;
        response->message = m_debugEnabled ? "Debug output enabled." : "Debug output disabled.";
      });

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

  RCLCPP_INFO(this->get_logger(), "Manipulator driver initialized and enabled: %s",
              this->get_fully_qualified_name());
  RCLCPP_INFO(this->get_logger(), "CAN channel %d, baud rate %d kbit/s; %u arm joints and one torque-controlled gripper.",
              m_canId, m_canBaudRate, static_cast<unsigned>(ManipulatorState.NumOfJoint));
  RCLCPP_INFO(this->get_logger(), "Arm control mode: %s; control 125 Hz, CAN receive 500 Hz, joint states 50 Hz.",
              ManipulatorPlannerMode == 1 ? "velocity" : "position");
  RCLCPP_INFO(this->get_logger(), "Joint commands: %s (rad), %s (rad/s); active mode selects which command is executed.",
              m_jointPosCmdSub->get_topic_name(), m_jointVelCmdSub->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Gripper command: %s (true=open, false=close); joint feedback: %s.",
              m_gripperCmdSub->get_topic_name(), m_jointStatePublisher->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Services under %s/: ip_pos_mode_init, ip_pos_mode_enable, ip_pos_mode_disable, open_gripper, close_gripper.",
              this->get_fully_qualified_name());
  RCLCPP_INFO(this->get_logger(), "Calibration services: set_gripper_open_position, set_gripper_close_position; calibration is stored in memory.");
  RCLCPP_INFO(this->get_logger(), "Diagnostics: clear_emergencies clears cached errors; set_debug enables command and motor status logs every 5 s (default off).");
  
}

ManipulatorDriver::~ManipulatorDriver()
{
  if (!m_controlEnabled) {
    RCLCPP_INFO(this->get_logger(), "Manipulator already disabled; skipping IPPosModeDisable.");
  } else {
    m_controlEnabled = false;
    RCLCPP_INFO(this->get_logger(), "Disabling manipulator before CAN shutdown...");
    const bool previousReadWait = canReadWaitFlag;
    canReadWaitFlag = true;
    const bool disableSucceeded = armInterface.IPPosModeDisable() == 1;
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

// 为同步服务临时选择 CAN 接收方式：wait_for_sdo 为 true 时等待 SDO 应答，
// 否则使用非阻塞读取；回调结束（包括异常）后恢复原来的读取方式。
rclcpp::Service<ManipulatorDriver::Trigger>::SharedPtr ManipulatorDriver::createApiService(
    const std::string &name, std::function<void()> action, bool wait_for_sdo)
{
  return this->create_service<Trigger>(
      name,
      [action, wait_for_sdo](const std::shared_ptr<Trigger::Request>,
                             std::shared_ptr<Trigger::Response> response) {
        struct ReadWaitGuard {
          bool previous = canReadWaitFlag;
          ~ReadWaitGuard() { canReadWaitFlag = previous; }
        } read_wait_guard;

        canReadWaitFlag = wait_for_sdo;
        response->success = false;
        try {
          action();
          response->success = true;
          response->message = "API call completed; hardware completion is not verified.";
        } catch (const std::exception &error) {
          response->message = error.what();
        } catch (...) {
          response->message = "Unknown error while calling the manipulator API.";
        }
      });
}

void ManipulatorDriver::requireControlEnabled() const
{
  if (!m_controlEnabled)
    throw std::runtime_error("Control is not enabled!");
}

void ManipulatorDriver::resetPlanner()
{
  m_velCmdPublishingFlag = false;
  double zero_velocity[6] = {0};
  armPlanner.setDesiredVelocity(zero_velocity);
  for (unsigned char i = 0; i < ManipulatorState.NumOfJoint; ++i)
    armPlanner.m_velCmd[i] = 0.0;

  switch (ManipulatorPlannerMode) {
    case 1:
      armPlanner.AutoVelModeInit();
      break;
    case 2:
      armPlanner.AutoPosModeInit();
      break;
    default:
      throw std::runtime_error("Unsupported manipulator mode.");
  }
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
  
  if (armInterface.IPPosModeInit() != 1)
    throw std::runtime_error("IP mode initialization failed.");
  RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint 1-6 IP POS Mode, Joint 7 Torque Mode");

  switch (ManipulatorPlannerMode) {
    case 1:
      armPlanner.AutoVelModeInit();
      RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint 1-6 controlled in Vel Mode");
      break;
    case 2:
      armPlanner.AutoPosModeInit();
      RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint 1-6 controlled in Pos Mode");
      break;
    default:
      break;
  }

  if (armInterface.IPPosModeEnable() != 1)
    throw std::runtime_error("IP mode enable failed.");
  m_controlEnabled = true;
  RCLCPP_INFO(this->get_logger(), "InitManipulator: Joint 1-6 enabled");
  canReadWaitFlag = false;
  RCLCPP_INFO(this->get_logger(), "canReadWaitFlag: Set to false for PDO");

}

void ManipulatorDriver::controlLoopCallback()
{
  if (m_controlEnabled || (ipModeFlags & IP_Flag_Init)){
    armInterface.updateState();
    armInterface.sendFdbRequest();
  }
  if(m_controlEnabled){
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
  joint_state.position[6] = armInterface.m_gripperJointDirection * armInterface.m_gripperAngle;
  joint_state.position[7] = -1 * joint_state.position[6];

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
    RCLCPP_INFO(this->get_logger(), "Motor %u status=0x%04X",
                static_cast<unsigned>(i + 1),
                static_cast<unsigned>(ManipulatorState.ActualStatus[i]));
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
  if (!m_controlEnabled) {
    RCLCPP_WARN(this->get_logger(), "Gripper command ignored: manipulator is disabled.");
    return;
  }

  if (msg->data) {
    if (armInterface.IPOpenGripper() != 1) {
      RCLCPP_ERROR(this->get_logger(), "Open gripper command failed.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Open Gripper with current %d (per thousand rated torque)", GripperTorque);
  } else {
    if (armInterface.IPCloseGripper() != 1) {
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
    cmd[i] = msg->data.at(i);
  armPlanner.setDesiredVelocity(cmd);

  m_lastVelCmdTime = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "joint Velocity Cmd cmd1 %f cmd2 %f cmd3 %f cmd4 %f cmd5 %f cmd6 %f",
              cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
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
