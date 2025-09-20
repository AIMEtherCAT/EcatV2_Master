## EtherCAT Task Introduction

### LkTech Motor

#### Hardware preparation

Connect your motor to any ``CAN`` port of your EtherCAT module.

Note that a maximum of 3 motors in the same CAN network is suitable for a 1khz control frequency. If you connect more,
please reduce the motor feedback or control frequency.

> **Note:** all motors will be disabled before receiving any command.

#### Configuration items

* Control Period
    * This controls the frequency at which control frames are sent to the CAN network. For LkTech Motors, this task will
      forward control commands at this frequency.
* CAN
    * The CAN port you connected to.
* CAN Packer ID
    * The packet ID of the control/feedback frame.
* Control Type
    * The control type of motor.
        * Open-loop current (only supported in MS serial motors)
        * Torque (only supported in MF/MH/MG serial motors)
        * Speed With Torque Limit, the unit of control target is 0.01deg/s
        * Multi-Round Position, the unit of control target is 0.01deg
        * Multi-Round Position With Speed Limit, the unit of control target is 0.01deg, the unit of speed limitation is
          1deg/s
        * Single-Round Position, the unit of control target is 0.01deg
        * Single-Round Position With Speed Limit, the unit of control target is 0.01deg, the unit of speed limitation is
          1deg/s

More information about the protocol of LkTech Motor can be found [here](http://www.lkmotor.cn/Download.aspx?ClassID=47).

You can change the publisher topic name by inputting a new name in the ``Motor Feedback Publisher Topic Name`` input
box.

You can change the subscriber topic name by inputting a new name in the ``Motor Command Subscriber Topic Name`` input
box.

#### Related ROS2 Message Types

```c
/* Message type: custom_msgs/msg/ReadLkMotor */

std_msgs/Header header

int16 current       // 66/4096 A for MG serial motors, 33/4096 A for MF serial motors, [-1000, 1000] PWR for MS serial motors
int16 speed         // 1deg/s
uint16 encoder
uint8 temperature
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorOpenloopControl */
/* Only supported in MF/MH/MG serial motors */

uint8 enable    // 0 or 1

int16 torque    // [-850, 850]
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorTorqueControl */
/* Only supported in MS serial motors */

uint8 enable    // 0 or 1

int16 torque    // [-2048, 2048], mapped to [-16.5A, 16.5A] for MF serial motors, mapped to [-33A, 33A] for MG serial motors
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorSpeedControlWithTorqueLimit */

uint8 enable        // 0 or 1

int16 torque_limit  // [-2048, 2048], mapped to [-16.5A, 16.5A] for MF serial motors, mapped to [-33A, 33A] for MG serial motors
int32 speed         // 0.01deg/s
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorMultiRoundPositionControl */

uint8 enable    // 0 or 1

int32 angle     // 0.01deg
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorMultiRoundPositionControlWithSpeedLimit */

uint8 enable    // 0 or 1

int16 speed_limit   // deg/s
int32 angle         // 0.01deg
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorSingleRoundPositionControl */

uint8 enable    // 0 or 1

uint8 direction // 0 for clockwise, 1 for counter-clockwise
uint32 angle    // 0.01deg
```

```c
/* Message type: custom_msgs/msg/WriteLkMotorSingleRoundPositionControlWithSpeedLimit */

uint8 enable    // 0 or 1

uint8 direction     // 0 for clockwise, 1 for counter-clockwise
int16 speed_limit   // 1deg/s
uint32 angle        // 0.01deg
```