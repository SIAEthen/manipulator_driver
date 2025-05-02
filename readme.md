# manipulator_driver
## main.cpp 
利用ROS实现定时器的功能，循环执行相关代码。

## how to use?

```#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 1.导入消息类型JointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,Bool

import threading
import time

class RobotVelocityCommand(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"node {name} init..")
        # 创建并初始化发布者成员属性pub_joint_states_
        self.jointVelocityCmdPublisher = self.create_publisher(Float64MultiArray,"joint_vel_cmd", 10) 
        self.gripperOpenCmdPublisher = self.create_publisher(Bool,"open_gripper_cmd", 10) 


        # 控制gripper， False就是关闭，True就是打开
        msg = Bool()
        msg.data = False
        self.gripperOpenCmdPublisher.publish(msg)


        # 以10Hz的频率发布机械臂关节速度，关节速度采用Float64MultiArray封装。
        # 当改程序取消后，机械臂驱动会自动检测指令超时并赋各关节速度值为0
        self.pub_rate = self.create_rate(10)
        self.thread_ = threading.Thread(target=self._thread_pub)
        self.thread_.start()

    def _thread_pub(self):
        last_update_time = time.time()
        while rclpy.ok():
            msg = Float64MultiArray()
            # 关节1-5速度为0,关节6的速度为0.5 rad/s
            msg.data = [0.0,0.0,0.0,0.0,0.0,0.5]
            # 发布关节数据
            self.jointVelocityCmdPublisher.publish(msg)
            self.pub_rate.sleep()

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = RobotVelocityCommand("manipulator_vel_control")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy



if __name__ == "__main__":
	main()```

