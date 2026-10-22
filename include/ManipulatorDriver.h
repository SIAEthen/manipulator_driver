#ifndef MANIPULATOR_DRIVER_H
#define MANIPULATOR_DRIVER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class ManipulatorDriver : public rclcpp::Node
{
public:
  ManipulatorDriver();
  ~ManipulatorDriver();

private:
  using Trigger = std_srvs::srv::Trigger;

  void motionInitCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void motionEnableCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void motionDisableCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void closeGripperCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void openGripperCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void setGripperOpenPositionCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void setGripperClosePositionCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void clearEmergenciesCallback(const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void setDebugCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void requireControlEnabled() const;
  void resetPlanner();
  void initManipulator();
  void controlLoopCallback();
  void handleCanLoopCallback();
  void handleROS2Loop();
  void handleDebugLoop();
  void logEmergencies() const;
  void gripperCmdCallback(const std_msgs::msg::Bool::SharedPtr msg) const;
  void jointVelCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void jointPosCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void checkCmdTimeout(double timeout_sec);

  rclcpp::TimerBase::SharedPtr m_controlLoopTimer;
  rclcpp::TimerBase::SharedPtr m_canLoopTimer;
  rclcpp::TimerBase::SharedPtr m_ros2LoopTimer;
  rclcpp::TimerBase::SharedPtr m_debugLoopTimer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatePublisher;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_jointVelCmdSub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_jointPosCmdSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_gripperCmdSub;

  rclcpp::Service<Trigger>::SharedPtr m_initService;
  rclcpp::Service<Trigger>::SharedPtr m_enableService;
  rclcpp::Service<Trigger>::SharedPtr m_disableService;
  rclcpp::Service<Trigger>::SharedPtr m_closeGripperService;
  rclcpp::Service<Trigger>::SharedPtr m_openGripperService;
  rclcpp::Service<Trigger>::SharedPtr m_setGripperOpenPositionService;
  rclcpp::Service<Trigger>::SharedPtr m_setGripperClosePositionService;
  rclcpp::Service<Trigger>::SharedPtr m_clearEmergenciesService;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_debugService;

  bool m_controlEnabled = false;
  bool m_debugEnabled = false;
  std::chrono::steady_clock::time_point m_lastVelCmdTime;
  bool m_velCmdPublishingFlag = false;
  int m_canId;
  int m_canBaudRate;
  std::vector<std::string> m_jointNames;
  std::vector<double> m_robotJointOffset;
  std::vector<double> m_robotJointDirection;
};

#endif
