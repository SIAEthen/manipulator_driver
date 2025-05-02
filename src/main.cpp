#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp/qos.hpp"

#include "ManipulatorPlanner.h"
#include "ManipulatorInterface.h"
#include "canDriver.h"
#include "defines.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
canDriver candriver(0,500);
int32_t jointoffset[6] = {32000, 13930,13156,31861,31692,32000+29038-65536};
uint8_t joint_num = JointNum;
uint8_t control_frequency = 125;
//ManipulatorInterface(int32_t *joint_offset,uint8_t joint_num, uint8_t control_frequency, canDriver &driver);
ManipulatorInterface armInterface(jointoffset,joint_num,control_frequency,candriver);
ManipulatorPlanner armPlanner(armInterface);

class ManipulatorDriver : public rclcpp::Node
{
  public:
    ManipulatorDriver()
    : Node("manipulator_driver")
    {
        // 创建QoS配置 (工业级可靠传输)
        // 或使用链式配置（推荐）
        auto qos = rclcpp::QoS(10) // 保持最后10条
          .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
          .reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT); // 1秒截止时间  

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",qos);
        timerControlLoop = this->create_wall_timer(8ms, std::bind(&ManipulatorDriver::ControlLoopCallback, this));
        timerHandleCanLoop = this->create_wall_timer(1ms, std::bind(&ManipulatorDriver::HandleCanLoopCallback, this));
        timerHandleROS2Loop = this->create_wall_timer(20ms, std::bind(&ManipulatorDriver::HandleROS2Loop, this));
        timerHandleDebugLoop = this->create_wall_timer(2000ms, std::bind(&ManipulatorDriver::HandleDebugLoop, this));
        
        auto qos_cmd = rclcpp::QoS(10).reliable().history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        gripperCmdSub = this->create_subscription<std_msgs::msg::Bool>("/open_gripper_cmd",qos_cmd,
                                                                std::bind(&ManipulatorDriver::gripperCmdCallback, this, std::placeholders::_1));
        jointVelCmdSub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/joint_vel_cmd",qos_cmd,
                                                                std::bind(&ManipulatorDriver::jointVelCmdCallback, this, std::placeholders::_1));
        // 初始化关节速度，测试一下
        // double velDes[6] = {0,0.1,0,0,0,0};
        // armPlanner.setDesiredVelocity(velDes);
        // armInterface.IPOpenGripper();
        // armInterface.IPCloseGripper();
    }
    ~ManipulatorDriver(){
      armInterface.IPPosModeDisable();
      candriver.deinitialize();
    }

  private:
    void ControlLoopCallback(){
      armInterface.updateState();
      armPlanner.run();
      armInterface.sendFdbRequest();
      return;
    }
    void HandleCanLoopCallback(){
      armInterface.handleCANFdb();
      return;
    }

    void HandleROS2Loop() {
        // 获取当前时间
        auto now = this->now();
        sensor_msgs::msg::JointState joint_state_;
        joint_state_.header.stamp = now;
        joint_state_.name = {
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
            "joint_gripper1", "joint_gripper2"
        };
        joint_state_.position.resize(JointNum+2, 0.0);
        joint_state_.velocity.resize(JointNum+2, 0.0);
        joint_state_.effort.resize(JointNum+2, 0.0);
        for(unsigned char i=0;i<JointNum;i++){
          joint_state_.position[i] = armInterface.m_jointposition[i];
          joint_state_.velocity[i] = armInterface.m_jointvelocity[i];
          joint_state_.effort[i] = armInterface.m_jointcurrent[i];
        }
        // 发布消息
        publisher_->publish(joint_state_);
        
        checkCmdTimeout(1);
  
    }

    
    void HandleDebugLoop(){
      RCLCPP_INFO(this->get_logger(), "Debug cmd1 %f cmd2 %f cmd3 %f cmd4 %f cmd5 %f cmd6 %f",
        armPlanner.m_posCmd[0],armPlanner.m_posCmd[1],armPlanner.m_posCmd[2],
        armPlanner.m_posCmd[3],armPlanner.m_posCmd[4],armPlanner.m_posCmd[5]);
      return;
    }
    
    void gripperCmdCallback(const std_msgs::msg::Bool::SharedPtr msg) const
    {
        if(msg->data){
          armInterface.IPOpenGripper();
          RCLCPP_INFO(this->get_logger(), "Open Gripper with current %d",GripperTorque);}
        else{
          armInterface.IPCloseGripper();
          RCLCPP_INFO(this->get_logger(), "Close Gripper with current %d",GripperTorque);}
    }
    void jointVelCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    { if(!velCmdPublishingFlag){
          velCmdPublishingFlag = true; }//标志开始收到消息，CheckCmdTimeout开始运行
      double cmd[6] = {0};
      for(unsigned char i=0;i<armInterface.m_jointnum;i++){
          cmd[i] = msg->data.at(i);
      }    
      armPlanner.setDesiredVelocity(cmd);

      auto system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
      last_cmd_time_ = system_clock->now(); //记录当前的时间，可以用于检验。

      RCLCPP_INFO(this->get_logger(), "joint Velocity Cmd cmd1 %f cmd2 %f cmd3 %f cmd4 %f cmd5 %f cmd6 %f",
                                                                              cmd[0],cmd[1],cmd[2],
                                                                              cmd[3],cmd[4],cmd[5]);}
    void checkCmdTimeout(double timeout){
        if(velCmdPublishingFlag){
            auto system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
            auto time_since_last = (system_clock->now() - last_cmd_time_).seconds();
            if (time_since_last > timeout) {
              velCmdPublishingFlag = false; //若是超时了，就说明没有命令发布，报一个warning之后就不再检查了
              RCLCPP_WARN(this->get_logger(), "ROS2 vel cmd timeout, set desired velocity to 0!");
              double velDes[6] = {0};
              armPlanner.setDesiredVelocity(velDes);}
            }
          return;
      
    }
    rclcpp::TimerBase::SharedPtr timerControlLoop;
    rclcpp::TimerBase::SharedPtr timerHandleCanLoop;
    rclcpp::TimerBase::SharedPtr timerHandleROS2Loop;
    rclcpp::TimerBase::SharedPtr timerHandleDebugLoop;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jointVelCmdSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripperCmdSub;

    rclcpp::Time last_cmd_time_; //for joint
    bool velCmdPublishingFlag = false; //当接收vel消息时，由ros接收消息回调函数将该cmdflag为true，并开始循环检查接收消息是否超时。
                                          //当消息cmd超时后，ROS——WARN报一次警告，将该flag为false，并且不再循环检查
};

int main(int argc, char * argv[])
{
  //初始化candriver
  candriver.initialize();
  //初始化机械臂内所有驱动器，1-6初始化为IP POS模式，7初始化为力矩模式
  armInterface.IPPosModeInit();

  //机械臂发布指令只能通过armPlanner发布，初始化规划的模式，才能在循环中循环调用run().
  //初始化模式标志位，初始化指令，让指令与当前位置相同
  armPlanner.AutoVelModeInit();

  //机械臂1-6关节使能
  armInterface.IPPosModeEnable();
  // std::cout << armPlanner.m_posCmd[0]<< armPlanner.m_posCmd[1]<< armPlanner.m_posCmd[2]<< 
  //         armPlanner.m_posCmd[3]<< armPlanner.m_posCmd[4]<< armPlanner.m_posCmd[5] << std::endl;
  canReadWaitFlag = false; //这个flag为true可以保证发消息有发有回，可以保证SDO配置参数都正常。但是在主循环中，不需要保证每发一个消息，就接收一个消息。接收消息是轮询机制。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManipulatorDriver>());
  rclcpp::shutdown();
  return 0;
}