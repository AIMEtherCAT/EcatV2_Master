## EtherCAT Task Introduction

### DJI RC

#### Hardware preparation

Connect your can pmu receiver into ``CAN2`` port of your EtherCAT module. All PMU using uavcan protocol should be
supported.

Note, you can only connect **one** ofpmu receiver into the same CAN network at the same time.

#### Configuration items

This task does not have any configuration items.

You can only change the publisher topic name by input new name at ``PMU(CAN) Publisher Topic Name`` input box.

#### Related ROS2 Message Types

```c
/* Message type: custom_msgs/msg/ReadCANPMU */

std_msgs/Header header

float32 temperature // °C
float32 voltage     // V
float32 current     // A
```