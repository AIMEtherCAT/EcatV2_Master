# EcatV2 Master

**EcatV2 Master** is a EtherCAT master wrapper library based on **ROS 2** and **SOEM** (Simple Open EtherCAT Master,
version 1.4.0, available [here](https://github.com/OpenEtherCATsociety/SOEM/tree/v1.4.0)).

This project aims to provide a stable, real-time EtherCAT communication backend for robotics applications. By using
simplified YAML configurations and standardized ROS 2 interfaces, it provides an efficient way to test and verify
control algorithms.

## Key Features

* **ROS 2 Integration**: Fully compatible with ROS 2 (Humble/Jazzy verified), enabling data interaction via standard
  Topics.
* **Configuration Driven**: Supports dynamic definition of slave topology, data length, and task mapping via YAML files,
  eliminating the need for recompilation.
* **Real-time Optimization**: Optimized for Real-time Kernels, supporting CPU core isolation and binding to ensure low
  latency and low jitter for control loops.
* **Modular Task System**: Built-in driver tasks for various common devices (e.g., DJI motors, IMUs, PWM controllers)
  with easy support for extending custom tasks.
* **Auxiliary Tools**: Provides practical utilities for EEPROM flashing, slave information scanning, and more.

## Quick Start

This project relies on several system environment configurations to guarantee real-time performance. Please read the
tutorials in the following order:

* Environment Preparation: [1. Environment Setup](docs/environment-setup.md)
    * Covers BIOS settings, enabling Real-time kernel, and CPU core isolation.
* First Run: [2. First Run Test](docs/first-run-test.md)
    * Creating a Bringup package, flashing EEPROMs, and running your first test node.
* Custom Configuration: [3. Customize configuration](docs/configuration-generator.md)
    * Generate and customize the YAML configuration files based on your hardware topology.

**FAQ: If you encounter issues, please check [0. FAQ](docs/faq.md) first.**

## Directory Structure

The core file structure is as follows to help you quickly locate code:

```text
EcatV2_Master/
├── docs/                   # Detailed documentation (Setup, First Run, Configuration Guides)
├── eeproms/                # Pre-configured EEPROM firmware files for EtherCAT slaves
├── src/
│   ├── soem/               # SOEM native library source code (Submodule)
│   ├── soem_wrapper/       # Core wrapper code (ROS 2 Node)
│   └── custom_msgs/        # Custom ROS2 message definitions
└── tools/                  # Utility tools provided by SOEM
```

## Tools

The `tools/` directory contains useful utilities provided by SOEM:

* `eepromtool`: For flashing EtherCAT slave EEPROMs.
* `slaveinfo`: For reading slave information. You can use this to check if the system detected your slave boards.
* `simple_test`: For testing the connection between master and slaves.

## Additional Info

* EtherCAT currently running in `Free-run` mode.

* ROS2 Topic QOS is `Sensor Data QoS`
    * History: Keep last,
    * Depth: 5,
    * Reliability: Best effort,
    * Durability: Volatile,
    * Deadline: Default,
    * Lifespan: Default,
    * Liveliness: System default,
    * Liveliness lease duration: default,
    * avoid ros namespace conventions: false

## Simple Performance Test

This test measures the round-trip time (RTT) between a master node and a single slave node. The master sends a 1-byte
sequence number to the slave and records the send timestamp. Upon receiving the packet, the slave will send it back to
the master. The master then calculates the RTT by subtracting the original send timestamp from the time it receives the
reply and publishes this latency as a Float32 message on corresponding latency topic.

### Test Environment

* **Operating System:** Ubuntu 24.04
* **Kernel:** 6.8.1-1031-realtime
* **ROS 2:** Jazzy 0.11.0-1noble.20250814.111056 amd64
* **CPU:** 12th Gen Intel® Core™ i7-12650H
* **Boot Parameters:**
  ``BOOT_IMAGE=/boot/vmlinuz-6.8.1-1031-realtime root=UUID=fd122407-8308-41d2-abcb-ae2fe12fc3ae ro quiet splash vt.handoff=7 nohz=on nohz_full=0,1 rcu_nocbs=0,1 isolcpus=0,1 irqaffinity=2-15``
* **Network Interface Card (NIC):**
    * PCI Address: 03:00.0
    * Model: Realtek RTL8111/8168/8211/8411 PCI Express Gigabit Ethernet Controller (rev 15)
    * Driver: r8169
    * Firmware Version: rtl8168h-2_0.0.2 02/26/2015
* **Active Tasks:**
    * 1x DBUS
    * 1x DM motor in MIT mode on CAN1 (1 motor)
    * 1x LK motor in broadcast mode on CAN2 (4 motors)
* **Additional Applications Running:**
    * Foxglove Bridge
    * CLion IDE
    * Standalone Python node recording `/latency` topic data

### Test Result

| Mean (ms) | Max (ms) | Min (ms) | 50th percentile (ms) | 90th percentile (ms) | 95th percentile (ms) | 99th percentile (ms) |
|:---------:|:--------:|:--------:|:--------------------:|:--------------------:|:--------------------:|:--------------------:|
|   0.241   |  0.414   |  0.169   |        0.222         |        0.314         |        0.321         |        0.333         |

![rtt-graph.png](docs/img/rtt-graph.png)

## Maintainer

* Hang (scyhx9@nottingham.ac.uk)