## EtherCAT Task Introduction

### DM Motor

#### Hardware preparation

Connect your motor to any ``CAN`` port of your EtherCAT module.

Note that a maximum of 3 motors in the same CAN network is suitable for a 1khz control frequency. If you connect more,
please reduce the motor feedback or control frequency.

> **Note:** all motors will be disabled before receiving any command.

#### Configuration items

* Control Period
    * This controls the frequency at which control frames are sent to the CAN network. For DM Motors, this task will
      forward control commands at this frequency.
* CAN
    * The CAN port you connected to.
* CAN ID
    * The packet ID of the control frame.
* Master ID
    * The packet ID of the feedback frame.
* P Max
    * Max position for data mapping.
* V Max
    * Max velocity for data mapping.
* T Max
    * Max torque for data mapping.

``CAN ID/Master ID`` and ``P/V/T Max`` can be found in the DM Debugging Tool as below.

![dm-debugging-tool.png](../img/dm-debugging-tool.png)

More information about DM Motor can be found [here](https://gl1po2nscb.feishu.cn/wiki/MZ32w0qnnizTpOkNvAZcJ9SlnXb).

You can change the publisher topic name by inputting a new name in the ``Motor Feedback Publisher Topic Name`` input
box.

You can change the subscriber topic name by inputting a new name in the ``Motor Command Subscriber Topic Name`` input
box.

#### Related ROS2 Message Types

```c
/* Message type: custom_msgs/msg/ReadDJICAN */

std_msgs/Header header

uint16 motor1_ecd // [0, 8191]
int16 motor1_rpm
int16 motor1_current
uint8 motor1_temperature

uint16 motor2_ecd // [0, 8191]
int16 motor2_rpm
int16 motor2_current
uint8 motor2_temperature

uint16 motor3_ecd // [0, 8191]
int16 motor3_rpm
int16 motor3_current
uint8 motor3_temperature

uint16 motor4_ecd // [0, 8191]
int16 motor4_rpm
int16 motor4_current
uint8 motor4_temperature
```

```c
/* Message type: custom_msgs/msg/WriteDJICAN */

uint8 motor1_enable // 0 or 1
int16 motor1_cmd    // current for openloop current mode, rpm for speed mode, ecd for position mode

uint8 motor2_enable s// 0 or 1
int16 motor2_cmd    // current for openloop current mode, rpm for speed mode, ecd for position mode

uint8 motor3_enable // 0 or 1
int16 motor3_cmd    // current for openloop current mode, rpm for speed mode, ecd for position mode

uint8 motor4_enable // 0 or 1
int16 motor4_cmd    // current for openloop current mode, rpm for speed mode, ecd for position mode
```