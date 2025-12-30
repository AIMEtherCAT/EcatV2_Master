cmake_minimum_required(VERSION 3.8)
project(soem_wrapper)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -fno-omit-frame-pointer")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soem REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
)

add_executable(soem_backend
        src/soem_backend.cpp
        src/ecat_node.cpp

        src/tasks/dji_motor.cpp
        src/tasks/dji_rc.cpp
        src/tasks/dm_motor.cpp
        src/tasks/dshot.cpp
        src/tasks/hipnuc_imu_can.cpp
        src/tasks/lk_motor.cpp
        src/tasks/onboard_pwm.cpp
        src/tasks/sbus_rc.cpp

        src/utils/io_utils.cpp
        src/utils/logger_utils.cpp
        src/utils/sys_utils.cpp
        src/utils/task_utils.cpp
        src/utils/config_utils.cpp
)

ament_target_dependencies(soem_backend
        soem rclcpp std_msgs custom_msgs sensor_msgs fmt yaml-cpp
)

target_link_libraries(soem_backend fmt yaml-cpp)

install(TARGETS soem_backend
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the reline when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
