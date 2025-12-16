## EtherCAT Task Introduction

### DJI Motor

#### Hardware preparation

Connect your motor to any ``CAN`` port of your EtherCAT module.

Note that a maximum of 7 motors in the same CAN network is suitable for a 1khz control frequency. If you connect more,
please reduce the motor feedback or control frequency.

> **Note:** all motors will be disabled before receiving any command.

Original product information pages can be found [here](https://bbs.robomaster.com/wiki/20204847/817325?source=7).

#### Configuration items

* Control Period
    * This controls the frequency at which control frames are sent to the CAN network. In open-loop mode, this task will
      forward control commands at this frequency. In closed-loop mode (i.e., any PID is enabled), this task will
      calculate all PIDs and send control commands calculated by the PID at this frequency.
* CAN
    * The CAN port you connected to.
* Motor Control Packet ID
    * The packet ID of the control frame.
        * 0x200 for C610/C620 ESC with motor id 1-4
        * 0x1ff for C610/C620 ESC with motor id 5-8 and 6020 motor with motor id 1-4 in voltage control mode
        * 0x2ff for 6020 motor with motor id 5-8 in voltage control mode
        * 0x1fe for 6020 motor with motor id 1-4 in current control mode
        * 0x2fe for 6020 motor with motor id 5-7 in current control mode
* Motor<n> Enable
    * Should this motor be monitored and controlled.
* Motor<n> ID
    * This should be set within the range of the corresponding control packet ID.
* Motor<n> Control Type
    * The control type of motor.
        * Open-loop Current for direct command forward (Note, DJI esc may have their own current control loop; this
          ``openloop`` only means the EtherCAT module will not control it in a closed loop)
        * Speed for single pid, control target is rpm
        * Single-Round position for cascade PID
            * inner loop = speed loop, control target is rpm
            * outer loop = position loop, control target is ecd, with arc selection optimization (i.e., optimal vs.
              non-optimal arc)

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
// same as the original definition
// 0 = ok
// 2 = The MSC supply voltage is too high (detected only once during power-on self-test)
// 3 = The three-phase cable connector is not connected to the motor
// 4 = The signal is lost on the 4-pin position sensor cable connected to the motor
// 5 = The motor is stalled
// 6 = Motor calibration failed
// If multiple warnings or errors occur at the same time, the error code with higher priority (smaller value) will be reported first.
uint8 motor1_error_code

uint16 motor2_ecd // [0, 8191]
int16 motor2_rpm
int16 motor2_current
uint8 motor2_temperature
uint8 motor2_error_code

uint16 motor3_ecd // [0, 8191]
int16 motor3_rpm
int16 motor3_current
uint8 motor3_temperature
uint8 motor3_error_code

uint16 motor4_ecd // [0, 8191]
int16 motor4_rpm
int16 motor4_current
uint8 motor4_temperature
uint8 motor4_error_code
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