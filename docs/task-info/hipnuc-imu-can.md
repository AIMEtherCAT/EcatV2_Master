## EtherCAT Task Introduction

### HIPNUC CH0X0 IMU

#### Hardware preparation

Connect your HIPNUC CH0X0 IMU forwarding board into any ``CAN`` port of your EtherCAT module.

This task only supports our own forwarding board, firmware, and introduction can be
found [here](https://github.com/AIMEtherCAT/hipnucimu).

> **Note:** If you want to connect multiple imu forwarding boards to the same CAN network, you need to change the packet
> IDs of each. For detailed information, please refer to the repo mentioned above.

#### Configuration items

* CAN
  * The CAN port you connected to.
* Packet1 ID
  * ID of packet 1
* Packet2 ID
  * ID of packet 2
* Packet3 ID
  * ID of packet 3
* Frame name
  * The `frame_id` in the `header` of the Imu message

You can change the publisher topic name by inputting a new name in the ``HIPNUC IMU Publisher Topic Name`` input box.

#### Related ROS2 Message Types

```c
/* Message type: sensor_msgs/msg/Imu */

std_msgs/Header header

geometry_msgs/Quaternion orientation
#float64[9] orientation_covariance // always empty

geometry_msgs/Vector3 angular_velocity  // rad/s
#float64[9] angular_velocity_covariance // always empty

geometry_msgs/Vector3 linear_acceleration   // m/s^2
#float64[9] linear_acceleration_covariance  // always empty
```