## EtherCAT Task Introduction

### SBUS RC

#### Hardware preparation

Connect your SBUS receiver to the ``DBUS`` port of your EtherCAT module.

#### Configuration items

This task does not have any configuration items.

You can only change the publisher topic name by inputting a new name in the ``SBUS RC Publisher Topic Name`` input box.

#### Related ROS2 Message Types

```c
/* Message type: custom_msgs/msg/ReadSBUSRC */

std_msgs/Header header

uint8 online        // 0 or 1
uint16[] channels   // length = 16

uint8 ch17
uint8 ch18
uint8 fail_safe     // 0 or 1
uint8 frame_lost    // 0 or 1
```