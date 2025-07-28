#ifndef WRAPPER_H
#define WRAPPER_H

#include "defs/app_defs.hpp"
#include "rclcpp/time.hpp"

// slave2master status enums
#define SLAVE_INITIALIZING 1
#define SLAVE_READY 2
#define SLAVE_CONFIRM_READY 3

// master2slave status enums
#define MASTER_REQUEST_REBOOT 1
#define MASTER_SENDING_ARGUMENTS 2
#define MASTER_READY 3

#endif
