## EtherCAT Task Introduction

### DJI RC

#### Hardware preparation

Connect your DR16 receiver to the ``DBUS`` port of your EtherCAT module.

#### Configuration items

This task does not have any configuration items.

You can only change the publisher topic name by inputting a new name in the ``DJI RC Publisher Topic Name`` input box.

#### Related ROS2 Message Types

```c
/* Message type: custom_msgs/msg/ReadDJIRC */

std_msgs/Header header

uint8 online // 0 or 1

float32 left_x  // [-1, 1]
float32 left_y  // [-1, 1]
float32 right_x // [-1, 1]
float32 right_y // [-1, 1]
float32 dial    // [-1, 1]

uint8 left_switch   // 1 = up, 3 = mid, 2 = bottom
uint8 right_switch  // 1 = up, 3 = mid, 2 = bottom

uint8 w     // 0 or 1
uint8 s     // 0 or 1
uint8 a     // 0 or 1
uint8 d     // 0 or 1
uint8 q     // 0 or 1
uint8 e     // 0 or 1
uint8 r     // 0 or 1
uint8 f     // 0 or 1
uint8 g     // 0 or 1
uint8 z     // 0 or 1
uint8 x     // 0 or 1
uint8 c     // 0 or 1
uint8 v     // 0 or 1
uint8 b     // 0 or 1
uint8 shift // 0 or 1
uint8 ctrl  // 0 or 1

int16 mouse_x
int16 mouse_y
uint8 mouse_left_clicked    // 0 or 1
uint8 mouse_right_clicked   // 0 or 1
```