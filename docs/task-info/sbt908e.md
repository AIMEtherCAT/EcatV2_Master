## EtherCAT Task Introduction

### SBT908E Loadcell Amplifier

#### Hardware preparation

Connect your SBT908E into the EtherCAT ring with its ``IN`` and ``OUT`` ports. Wire and calibrate the
load cells following the manual of the device.

> **Note:** The master disables the SM watchdog and switches the device to free run mode every time the device is
> configured.

#### Configuration items

This device is configured by its position in the ring, e.g. the first slave is ``slave1``.

* channel_count
    * The number of channels of the device, should be within 1-20.
* ad_speed
    * The AD conversions per second, it is the same for all channels. A lower speed gives a more precise reading value.
        * 0 = 10
        * 1 = 40
        * 2 = 640
        * 3 = 1280
* ad_resolution
    * The AD resolution in bits, it is the same for all channels, should be within 16-24.
* filter_type
    * The filter type, it is the same for all channels.
        * 0 = none
        * 1 = compound (recommended)
        * 2 = median
        * 3 = first order
        * 4 = moving average
        * 5 = median average
        * 6 = moving median average
        * 7 = average + first order
        * 8 = median + first order
        * 9 = moving average + first order
        * 10 = median average + first order
* filter1_level
    * The level of the first stage filter, it is the same for all channels, should be within 0-50.
* filter1_count
    * The loop count of the first stage filter, it is the same for all channels, should be within 0-9.
* filter2_level
    * The level of the second stage filter, it is the same for all channels, should be within 0-9.
* filter_stable
    * Should the stable filter be enabled, it is the same for all channels, 0 or 1.

Configuration example as follows:

```yaml
slaves:
  - slave1:
      pub_topic: !std::string '/ecat/sbt908e/force'
      channel_count: !uint8_t 6
      ad_speed: !uint8_t 2
      ad_resolution: !uint8_t 24
      filter_type: !uint8_t 9
      filter1_level: !uint8_t 20
      filter1_count: !uint8_t 0
      filter2_level: !uint8_t 0
      filter_stable: !uint8_t 0
```

You can change the publisher topic name by inputting a new name in the ``pub_topic`` config item.

#### Related ROS2 Message Types

The services below send the corresponding command, and respond after the device reports the execution result.

* ``<pub_topic>/zero``
    * Zero the channel by the PDO command, ``SBT908ECommand``.
* ``<pub_topic>/clear_peak``
    * Clear the peak value by the PDO command, ``SBT908ECommand``.
* ``<pub_topic>/set_calibration_parameters``
    * Set the unit, the division and the max weighing value by the SDO command (9.6.1 of the manual),
      ``SBT908ESetCalibrationParameters``.
* ``<pub_topic>/zero_calibration``
    * Calibrate the zero point with the current load by the SDO command (9.6.2 of the manual), ``SBT908ECommand``.
* ``<pub_topic>/gain_calibration``
    * Calibrate the gain with a known weight on the sensor by the SDO command (9.6.3 of the manual),
      ``SBT908EGainCalibration``.
* ``<pub_topic>/sensitivity_calibration``
    * Calibrate the gain with the sensitivity and the range of the sensor by the SDO command (9.6.4 of the manual),
      ``SBT908ESensitivityCalibration``.

> **Note:**
> * A request with channel 255 waits for the results of all configured channels.
> * The parameters of the calibration services are written into the device as they are, see 9.6 of the manual.
    The rated accuracy of the device is 1/10000 of the full range, a smaller division only shows noise.

```c
/* Message type: custom_msgs/msg/ReadSBT908E */

std_msgs/Header header

SBT908EChannel[] channels  // one per configured channel, starting from channel 1
```

```c
/* Message type: custom_msgs/msg/SBT908EChannel */

float64 force   // gross force, in the unit of the channel
float64 peak    // in the unit of the channel
```

```c
/* Service type: custom_msgs/srv/SBT908ECommand */

uint8 channel   // [1, channel_count], 255 = all channels
---
bool success    // true if the device reports the command was executed correctly
string message  // the execution result of each channel, e.g. CH1=ok
```

```c
/* Service type: custom_msgs/srv/SBT908ESetCalibrationParameters */

uint8 channel       // [1, channel_count], 255 = all channels
int32 unit          // Para1, 0 = none, 1 = g, 2 = kg, 3 = t, 4 = N
int32 div           // Para2, 0 = 0.0001, 1 = 0.0002, 2 = 0.0005, 3 = 0.001, 4 = 0.002, 5 = 0.005, 6 = 0.01,
                    // 7 = 0.02, 8 = 0.05, 9 = 0.1, 10 = 0.2, 11 = 0.5, 12 = 1, 13 = 2, 14 = 5, 15 = 10, 16 = 20,
                    // 17 = 50
int32 full_range    // Para3, max weighing value, 0-8000000, e.g. 10 kg accurate to 1 g is 10000
---
bool success        // true if the device reports the command was executed correctly
string message      // the execution result of each channel, e.g. CH1=ok
```

```c
/* Service type: custom_msgs/srv/SBT908EGainCalibration */

uint8 channel   // [1, channel_count], 255 = all channels
int32 weight    // Para1, weight on the sensor, -8000000-8000000, e.g. 10 kg accurate to 1 g is 10000
---
bool success    // true if the device reports the command was executed correctly
string message  // the execution result of each channel, e.g. CH1=ok
```

```c
/* Service type: custom_msgs/srv/SBT908ESensitivityCalibration */

uint8 channel           // [1, channel_count], 255 = all channels
int32 sensitivity       // Para1, 0.1 mV/V - 7.8 mV/V without the decimal point, e.g. 2.0000 mV/V is 20000
int32 sensor_range      // Para2, range of a single sensor, e.g. 10 kg accurate to 1 g is 10000
---
bool success            // true if the device reports the command was executed correctly
string message          // the execution result of each channel, e.g. CH1=ok
```
