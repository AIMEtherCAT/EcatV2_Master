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

## Maintainer

* Hang (scyhx9@nottingham.ac.uk)