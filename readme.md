# manipulator_driver

`joint_names` 为启动时读取的字符串数组参数，用于设置发布的 `JointState.name`。
按关节 1～6、夹钳 1、夹钳 2 的顺序配置 8 个非空名称，并与 URDF 中的关节名称一致。
各 launch 文件的 `parameters` 中均已提供默认名称，可分别修改左右机械臂的配置。

关节参数使用数组 `robot_joint_offsets`（默认全为 0.0，单位 rad）
和 `robot_joint_directions`（默认全为 1.0），每个数组按关节 1～6 的顺序配置 6 个值。
`robot_gripper_torque_direction` 控制夹钳开合力矩方向；
`robot_gripper_joint_direction` 控制夹钳发布角度的方向，两者默认 `1.0`，设为 `-1.0` 可反转。
CAN 参数为 `can_id`（默认 0）和 `can_baud_rate`（默认 500），名称末尾不带下划线。
这些参数在节点启动时读取，可在 launch 参数中设置。

## main.cpp 
利用ROS实现定时器的功能，循环执行相关代码。

### 初始化部分
```/初始化candriver
  candriver.initialize();
  //初始化机械臂内所有驱动器，1-6初始化为IP POS模式，7初始化为力矩模式
  armInterface.initIPPosMode();

  //机械臂发布指令只能通过armPlanner发布，初始化规划的模式，才能在循环中循环调用run().
  //初始化模式标志位，初始化指令，让指令与当前位置相同
  armPlanner.AutoVelModeInit();

  //机械臂1-6关节使能
  armInterface.enableIPPosMode();
  // std::cout << armPlanner.m_posCmd[0]<< armPlanner.m_posCmd[1]<< armPlanner.m_posCmd[2]<< 
  //         armPlanner.m_posCmd[3]<< armPlanner.m_posCmd[4]<< armPlanner.m_posCmd[5] << std::endl;
  canReadWaitFlag = false; 
```

### ControlLoopCallback
8ms执行一次，为IP模式插补周期。需要更新arminterface中的关节位置、速度等，通过armplanner发布指令，发送同步消息等等
### HandleCanLoopCallback
1ms执行一次，循环读can的消息
### HandleROS2Loop
20ms一次，包括2个任务，发布jointstates，以及时刻检查速度指令是否超时。（如果超时则报warning并把期望速度置0）
### HandleDebugLoop
2s一次，打印一些消息。

### ROS 2 services

以下服务均使用 `std_srvs/srv/Trigger`，请求为空，返回 `success` 和 `message`。
默认节点名下的服务如下；设置 namespace 或修改节点名时，服务名称会随之改变。

| 服务 | 封装接口 |
| --- | --- |
| `/manipulator_driver/motion_init` | `initIPPosMode()` |
| `/manipulator_driver/motion_enable` | `enableIPPosMode()` |
| `/manipulator_driver/motion_disable` | `disableIPPosMode()` |
| `/manipulator_driver/close_gripper` | `closeGripperPDOCmd()` |
| `/manipulator_driver/open_gripper` | `openGripperPDOCmd()` |
| `/manipulator_driver/set_gripper_open_position` | 将当前夹钳编码器反馈保存为全开标定位置 |
| `/manipulator_driver/set_gripper_close_position` | 将当前夹钳编码器反馈保存为全闭标定位置 |
| `/manipulator_driver/clear_emergencies` | 清除节点缓存的 emergency 错误字段 |
| `/manipulator_driver/set_debug` | 开启或关闭周期调试输出 |

节点启动时仍自动初始化并使能。需要重新初始化时，依次调用：

```bash
ros2 service call /manipulator_driver/motion_disable std_srvs/srv/Trigger "{}"
ros2 service call /manipulator_driver/motion_init std_srvs/srv/Trigger "{}"
ros2 service call /manipulator_driver/motion_enable std_srvs/srv/Trigger "{}"
```

清除已缓存的 emergency 错误码（不会向驱动器发送故障复位；若故障仍在，后续报文会再次记录）：

```bash
ros2 service call /manipulator_driver/clear_emergencies std_srvs/srv/Trigger "{}"
```

调试输出默认关闭，可通过 `SetBool` 服务控制：

```bash
ros2 service call /manipulator_driver/set_debug std_srvs/srv/SetBool "{data: true}"
ros2 service call /manipulator_driver/set_debug std_srvs/srv/SetBool "{data: false}"
```

去使能后停止规划下发；重新使能时清除旧运动目标，从当前位置重置规划器。
夹爪开合运动服务需要机械臂处于使能状态，以便控制循环继续发送 SYNC。
两个夹爪位置标定服务使用最近一次解析的编码器反馈，不发送运动指令；标定仅保存在内存中，节点重启后需重新标定。

```bash
ros2 service call /manipulator_driver/set_gripper_open_position std_srvs/srv/Trigger "{}"
ros2 service call /manipulator_driver/set_gripper_close_position std_srvs/srv/Trigger "{}"
```

驱动启动并使能后，可运行辅助脚本，依次打开夹钳、标定全开位置、关闭夹钳、标定全闭位置：

```bash
ros2 run manipulator_driver gripper_help
ros2 run manipulator_driver gripper_help --settle-time 5 --driver-node /manipulator_driver
```

也可通过 launch 启动辅助脚本（需先启动驱动，例如 `arm_launch_template.launch.py`）：

```bash
ros2 launch manipulator_driver gripper_help.launch.py
ros2 launch manipulator_driver gripper_help.launch.py driver_node:=/left/manipulator settle_time:=5.0
```

每次开合后默认等待 3 秒，请将 `--settle-time` 设置为足够到达机械端点的时间。
服务失败或超时会立即停止后续步骤；脚本结束时夹钳处于关闭状态。

初始化、使能和去使能期间临时开启 SDO 应答等待，退出回调时恢复原接收模式。
这些同步服务与 timer 串行执行，底层的等待和 sleep 会暂时阻塞其他回调，不能保持此期间的 8ms 控制周期。
底层 API 返回 `1` 表示 CAN/SDO 操作成功，返回 `0` 表示失败；服务据此设置 `success`。
`success=true` 仅表示这些底层调用成功，不表示硬件状态切换或夹爪动作已确认完成。

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
        self.jointVelocityCmdPublisher = self.create_publisher(Float64MultiArray,"/manipulator_driver/joint_vel_rads_cmd", 10)
        self.gripperOpenCmdPublisher = self.create_publisher(Bool,"/manipulator_driver/open_gripper_cmd", 10)


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
	main()
```


# working mode 20250521增加内容
## build the driver with pos mode or vel mode
In defines.h, 标志位ManipulatorMode控制机械臂运行在哪种模式，为1是速度模式，为2是位置模式。
驱动的指令话题使用节点私有名称：`~/joint_vel_rads_cmd`、`~/joint_pos_rad_cmd`、`~/open_gripper_cmd`。
默认节点下的完整路径分别为 `/manipulator_driver/joint_vel_rads_cmd`、`/manipulator_driver/joint_pos_rad_cmd`、`/manipulator_driver/open_gripper_cmd`。
前两个话题使用 `Float64MultiArray`，分别对应速度和位置控制模式；夹钳话题使用 `Bool`，true 打开、false 关闭。
修改节点名或 namespace 后，请相应修改发布端的话题路径；启动摘要会打印实际路径。

## 控制例程
robot_kinematic_control, ibvs.py有两种模式的控制程序框架

## 位置控制源码介绍
位置控制的源码在kinematicscontroller.cpp和相应的头文件中，在里面实现了一个类PosKinController
该类最重要的一个方法是通过当前期望位置、当前位置以及Kp参数获得速度，并固定了加速度以保证速度指令平滑，之后将平滑后的速度，以及上一时刻的命令积分，获得实际位置指令。

该重要方法有一个后续可以思考的点，出了bug可以修改。 是否可以直接用上一时刻命令代替当前位置，与期望位置用Kp生成速度呢？


# usage

` ros2 run manipulator_driver driver `
` ros2 launch seven_function_arm_urdf view2_robot_launch.py `
In VSCode, run ibvs.py


# debug

# 通过ns控制节点所有topic的名称
首先在指定话题名称时前面不要加斜线
避免在节点代码中使用全局名称（如/topic）。
优先用​​相对名称​​（如topic），使其自动继承namespace。
在launch文件中利用namespace配置
