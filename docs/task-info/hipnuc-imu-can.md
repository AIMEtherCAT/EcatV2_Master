## EtherCAT Task Introduction

### DJI RC

#### Hardware preparation

Connect your CAN PMU receiver to the ``CAN2`` port of your EtherCAT module. All PMUs using the UAVCAN protocol and
sending BatteryInfo-type messages should be supported.

> **Note:** you can only connect **one** PMU into the same CAN network simultaneously.

#### Configuration items

This task does not have any configuration items.

You can only change the publisher topic name by inputting a new name in the ``PMU(CAN) Publisher Topic Name`` input box.

#### Related ROS2 Message Types

```c
/* Message type: custom_msgs/msg/ReadCANPMU */

std_msgs/Header header

float32 temperature // °C
float32 voltage     // V
float32 current     // A
```