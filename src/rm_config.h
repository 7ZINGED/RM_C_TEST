 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _RM_CONFIG_H
#define _RM_CONFIG_H

#define CPU_FREQUENCY 168     /* CPU主频(mHZ) */

/* 底盘和云台分别对应的 can 设备名称 */
#define CAN_CHASSIS    "can1"
#define CAN_GIMBAL     "can2"

/* 磁力计所挂载的 i2c 设备名称(软件i2c) */
#define I2C_MAG        "i2c1"    //"Notice: PA8 --> 8; PC9 --> 41"

/* 陀螺仪所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_GYRO       "spi1"
#define SPI_GYRO_CS    16
/* 加速度计所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_ACC        "spi1"
#define SPI_ACC_CS     4

/* 弹仓盖舵机所挂载的 PWM 设备及通道号 */
#define PWM_COVER        "pwm1"
#define PWM_COVER_CH     2

/* 遥控器所挂载的 usart 设备名称 */
#define USART_RC       "uart3"

/* ---------------------------------- 遥控器相关 --------------------------------- */
#define RC_MAX_VALUE      784.0f  /* 遥控器通道最大值 */
#define RC_DBUS_MAX_VALUE      660.0f  /* DBUS遥控器通道最大值 */
#define RC_RATIO          0.0009f
#define KB_RATIO          0.010f
/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 0.8f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* 鼠标键盘模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_PC_MOVE_RATIO_R 5.0f

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f

/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.1f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.5f

/* 遥控器拨杆对应档位值 */
#define RC_UP_VALUE 240
#define RC_MID_VALUE 0
#define RC_DN_VALUE 15

/* ---------------------------------- 底盘相关 ---------------------------------- */
/* 底盘轮距(mm) */
#define WHEELTRACK        340
/* 底盘轴距(mm) */
#define WHEELBASE         388
/* 底盘轮子周长(mm) */
#define WHEEL_PERIMETER   471

#define LENGTH_RADIUS 230 //底盘的半径(mm)

/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转 */
#define MAX_WHEEL_RPM        9000   //8347rpm = 3500mm/s

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 7000
#define MAX_CHASSIS_VY_SPEED 7000

#define MAX_CHASSIS_VX_SPEED_HIGH 11000
#define MAX_CHASSIS_VY_SPEED_HIGH 11000

#define MAX_CHASSIS_VX_SPEED_LOW 5000
#define MAX_CHASSIS_VY_SPEED_LOW 5000

/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 8

/* --------------------------------- 底盘PID参数 -------------------------------- */
/* 电机速度环 */
#define CHASSIS_KP_V_MOTOR              6
#define CHASSIS_KI_V_MOTOR              0
#define CHASSIS_KD_V_MOTOR              0.0001
#define CHASSIS_INTEGRAL_V_MOTOR        2000
#define CHASSIS_MAX_V_MOTOR             16000
// TODO: 参数待整定
/* 跟随云台PID */
#define CHASSIS_KP_V_FOLLOW             0.1f
#define CHASSIS_KI_V_FOLLOW             0
#define CHASSIS_KD_V_FOLLOW             0.001f
#define CHASSIS_INTEGRAL_V_FOLLOW       0
#define CHASSIS_MAX_V_FOLLOW            20

/* ---------------------------------- 云台相关 ---------------------------------- */
#define YAW_MOTOR_ID     0x207
#define PITCH_MOTOR_ID   0x208

/*云台编码器归中*/
//#define GIMBAL_SIDEWAYS
#ifdef GIMBAL_SIDEWAYS
#define SIDEWAYS_ANGLE   36
#define CENTER_ECD_YAW   3818         //云台yaw轴编码器归中值(侧身)
#else
#define CENTER_ECD_YAW   7913         //云台yaw轴编码器归中值
#define SIDEWAYS_ANGLE   0
#endif


#define CENTER_ECD_PITCH 2717         //云台pitch轴编码器归中值
/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        30.0f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -20.0f

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 100

/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU             5000
#define YAW_KI_V_IMU             0
#define YAW_KD_V_IMU             0.5
#define YAW_INTEGRAL_V_IMU       0
#define YAW_MAX_V_IMU            30000
/* imu角度环 */
#define YAW_KP_A_IMU             0.35f
#define YAW_KI_A_IMU             0
#define YAW_KD_A_IMU             0.001f
#define YAW_INTEGRAL_A_IMU       0
#define YAW_MAX_A_IMU            25
/* auto速度环 */
#define YAW_KP_V_AUTO            5000
#define YAW_KI_V_AUTO            0
#define YAW_KD_V_AUTO            0.5
#define YAW_INTEGRAL_V_AUTO      0
#define YAW_MAX_V_AUTO           30000
/* auto角度环 */
#define YAW_KP_A_AUTO            0.35f
#define YAW_KI_A_AUTO            0
#define YAW_KD_A_AUTO            0.001f
#define YAW_INTEGRAL_A_AUTO      0
#define YAW_MAX_A_AUTO           25

/* 云台PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU           8000
#define PITCH_KI_V_IMU           4000
#define PITCH_KD_V_IMU           0.001
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          30000

/* imu角度环 */
#define PITCH_KP_A_IMU           0.35f
#define PITCH_KI_A_IMU           0.0f
#define PITCH_KD_A_IMU           0.0001f
#define PITCH_INTEGRAL_A_IMU     0.0f
#define PITCH_MAX_A_IMU          20

/* auto速度环 */
#define PITCH_KP_V_AUTO          8000
#define PITCH_KI_V_AUTO          4000
#define PITCH_KD_V_AUTO          0.001
#define PITCH_INTEGRAL_V_AUTO    1500
#define PITCH_MAX_V_AUTO         30000
/* auto角度环 */
#define PITCH_KP_A_AUTO          0.35f
#define PITCH_KI_A_AUTO          0.0f
#define PITCH_KD_A_AUTO          0.001f
#define PITCH_INTEGRAL_A_AUTO    0.0f
#define PITCH_MAX_A_AUTO         20

/* ---------------------------------- 发射相关 ---------------------------------- */
// TODO: 实际值待整定
#define RIGHT_FRICTION_MOTOR_ID     0x201
#define LEFT_FRICTION_MOTOR_ID   0x202
#define MIDDLE_FRICTION_MOTOR_ID   0x204
#define TRIGGER_MOTOR_ID  0x203

#define TRIGGER_MOTOR_45_TO_ANGLE 45
/* -------------------------------- 发射电机PID参数 ------------------------------- */
// TODO: 速度期望应改为变量应对速度切换。初次参数调整已完成
/* 右摩擦轮M3508电机PID参数 */
/* 速度环 */
#define RIGHT_KP_V             23
#define RIGHT_KI_V             0.1
#define RIGHT_KD_V             0.001f
#define RIGHT_INTEGRAL_V       50
#define RIGHT_MAX_V            30000

/* 左摩擦轮M3508电机PID参数 */
/* 速度环 */
#define LEFT_KP_V           23
#define LEFT_KI_V           0.1
#define LEFT_KD_V           0.001f
#define LEFT_INTEGRAL_V     50
#define LEFT_MAX_V          30000
/* 中摩擦轮M3508电机PID参数 */
/* 速度环 */
#define MIDDLE_KP_V             23
#define MIDDLE_KI_V             0.1
#define MIDDLE_KD_V             0.001f
#define MIDDLE_INTEGRAL_V       50
#define MIDDLE_MAX_V            30000

// TODO：PID参数初次微调已完成，期待后续微调
/* 拨弹电机M2006电机PID参数 */
/* 速度环 */
#define TRIGGER_KP_V           10
#define TRIGGER_KI_V           5
#define TRIGGER_KD_V           0.01f
#define TRIGGER_INTEGRAL_V     1500
#define TRIGGER_MAX_V          20000
/* 角度环 */
#define TRIGGER_KP_A           5
#define TRIGGER_KI_A           0
#define TRIGGER_KD_A           0
#define TRIGGER_INTEGRAL_A     0
#define TRIGGER_MAX_A          10000

#endif /* _RM_CONFIG_H */