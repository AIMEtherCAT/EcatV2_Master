#ifndef PACKET_H
#define PACKET_H

#define UNKNOWN_APP_ID 999
#define DJIRC_APP_ID 1
#define LK_APP_ID 2
#define HIPNUC_IMU_CAN_APP_ID 3
#define DSHOT_APP_ID 4
#define DJICAN_APP_ID 5
#define VANILLA_PWM_APP_ID 6
#define EXTERNAL_PWM_APP_ID 7
#define MS5876_30BA_APP_ID 8
#define ADC_APP_ID 9
#define CAN_PMU_APP_ID 10
#define SBUS_RC_APP_ID 11
#define DM_MOTOR_APP_ID 12

#define CAN_PORT_1 1
#define CAN_PORT_2 2

#define TIM_2 2
#define TIM_3 3

#define USART1 1
#define UART4 4
#define UART8 8

#define I2C3 3

#define LK_CTRL_TYPE_OPENLOOP_CURRENT 0x01
#define LK_CTRL_TYPE_TORQUE 0x02
#define LK_CTRL_TYPE_SPEED_WITH_TORQUE_LIMIT 0x03
#define LK_CTRL_TYPE_MULTI_ROUND_POSITION 0x04
#define LK_CTRL_TYPE_MULTI_ROUND_POSITION_WITH_SPEED_LIMIT 0x05
#define LK_CTRL_TYPE_SINGLE_ROUND_POSITION 0x06
#define LK_CTRL_TYPE_SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT 0x07

#define DM_CTRL_TYPE_MIT 0x01
#define DM_CTRL_TYPE_POSITION_WITH_SPEED_LIMIT 0x02
#define DM_CTRL_TYPE_SPEED 0x03

#define DJIMOTOR_CTRL_TYPE_CURRENT 0x01
#define DJIMOTOR_CTRL_TYPE_SPEED 0x02
#define DJIMOTOR_CTRL_TYPE_SINGLE_ROUND_POSITION 0x03

#define MS5876_30BA_OSR_256 0x01
#define MS5876_30BA_OSR_512 0x02
#define MS5876_30BA_OSR_1024 0x03
#define MS5876_30BA_OSR_2048 0x04
#define MS5876_30BA_OSR_4096 0x05
#define MS5876_30BA_OSR_8192 0x06

typedef struct {
    uint16_t w: 1;
    uint16_t s: 1;
    uint16_t a: 1;
    uint16_t d: 1;
    uint16_t shift: 1;
    uint16_t ctrl: 1;
    uint16_t q: 1;
    uint16_t e: 1;
    uint16_t r: 1;
    uint16_t f: 1;
    uint16_t g: 1;
    uint16_t z: 1;
    uint16_t x: 1;
    uint16_t c: 1;
    uint16_t v: 1;
    uint16_t b: 1;
} Key_t;

typedef struct {
    uint16_t channel[5];
    uint8_t switcher[2];

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;

    double mouse_x_kalman;
    double mouse_y_kalman;

    double mouse_x_total;
    double mouse_y_total;

    uint8_t mouse_left_clicked;
    uint8_t mouse_right_clicked;

    Key_t key;
} controller_t;

#endif
