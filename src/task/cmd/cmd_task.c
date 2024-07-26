/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include "cmd_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"
#include "rc/sbus/rc_sbus.h"


#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static publisher_t *pub_gim, *pub_chassis, *pub_shoot;
static subscriber_t *sub_gim, *sub_shoot,*sub_trans,*sub_ins,*sub_referee;
static struct gimbal_cmd_msg gim_cmd;
static struct gimbal_fdb_msg gim_fdb;
static struct shoot_cmd_msg  shoot_cmd;
static struct shoot_fdb_msg  shoot_fdb;
static struct chassis_cmd_msg chassis_cmd;
static struct trans_fdb_msg  trans_fdb;
static struct ins_msg ins_data;
static struct referee_fdb_msg referee_fdb;

//static rc_dbus_obj_t *rc_now, *rc_last;
static rc_obj_t *rc_now,*rc_last;

static void cmd_pub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/*发射停止标志位*/
static int trigger_flag=0;
/*自瞄鼠标累计操作值*/
static float mouse_accumulate_x=0;
static float mouse_accumulate_y=0;
/*自瞄相对角传参反馈*/
extern auto_relative_angle_status_e auto_relative_angle_status;
/*堵转电流反转记次*/
static int reverse_cnt;
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;
/*用于清除环形缓冲区buffer的指针*/
extern rt_uint8_t *r_buffer_point;
/*----------------------------------裁判系统数据接收/比赛状态-------------------------------------*/
//extern robot_status_t robot_status;
//extern ext_power_heat_data_t power_heat_data_t;
/*按键状态标志位*/
static int key_e_status=-1;
static int key_f_status=-1;
static int key_q_status=-1;
static int key_v_status=-1;

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
//static void remote_to_cmd_dbus(void);
static void remote_to_cmd_sbus(void);
/*大疆DT7遥控器的控制链路接口，包涵键盘控制驱动*/
//static void remote_to_cmd_pc_DT7(void);
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式
/*键盘加速度的斜坡*/
//ramp_obj_t *km_vx_ramp;//x轴控制斜坡
//ramp_obj_t *km_vy_ramp;//y周控制斜坡
/*储存鼠标坐标数据*/
First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;
//float Ballistic;  //对自瞄数据进行手动鼠标弹道补偿
/* --------------------------------- cmd线程入口 -------------------------------- */
static float cmd_dt;

void cmd_thread_entry(void *argument)
{
    static float cmd_start;

    cmd_pub_init();
    cmd_sub_init();

    //rc_now = dbus_rc_init();
    rc_now = sbus_rc_init();
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    /* 鼠标一阶滤波初始化*/
   // First_Order_Filter_Init(&mouse_x_lpf,0.014f,0.1f);
   // First_Order_Filter_Init(&mouse_y_lpf,0.014f,0.1f);
    /* 初始化拨杆为上位 */
    rc_now->sw1 = 240;
    rc_now->sw2 = 240;
    rc_now->sw3 = 240;
    rc_now->sw4 = 240;
    /* 键盘控制急停斜坡的注册*/
   // km_vx_ramp = ramp_register(100, 2500000);
   // km_vy_ramp = ramp_register(100, 2500000);
    LOG_I("Cmd Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        cmd_sub_pull();

        /* 将遥控器原始数据转换为控制指令 */

//#ifdef BSP_USING_RC_KEYBOARD \
      // PC_Handle_kb();//处理PC端键鼠控制
     //   remote_to_cmd_pc_DT7();
//#endif
//        /* 将遥控器原始数据转换为控制指令 */
//        #ifdef BSP_USING_RC_DBUS
//        //remote_to_cmd_dbus();
//
//        #endif
        /* 将遥控器原始数据转换为控制指令 */
//#ifdef BSP_USING_RC_SBUS
        remote_to_cmd_sbus();
//#endif

        /* 更新发布该线程的msg */
        cmd_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
                LOG_E("Cmd Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者初始化
 */
static void cmd_pub_init(void)
{
    pub_gim = pub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    pub_chassis = pub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    pub_shoot= pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_cmd);
    pub_push_msg(pub_chassis, &chassis_cmd);
    pub_push_msg(pub_shoot, &shoot_cmd);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
    sub_shoot= sub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_referee= sub_register("referee_fdb",sizeof(struct referee_fdb_msg));
}

/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    sub_get_msg(sub_gim, &gim_fdb);
    sub_get_msg(sub_shoot, &shoot_fdb);
    sub_get_msg(sub_trans,&trans_fdb);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_referee, &referee_fdb);
}

///* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
///**
// * @brief 将遥控器数据转换为控制指令
// */
//#ifdef BSP_USING_RC_DBUS
//
//#endif


/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
#ifdef BSP_USING_RC_SBUS
static void remote_to_cmd_sbus(void)
{
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
  //  float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
   // float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
   // Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    //这里好像是忘了改成subs？
    chassis_cmd.vx += rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X /  784.0f* MAX_CHASSIS_VX_SPEED; // + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy += rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / 784.0f * MAX_CHASSIS_VY_SPEED ; //+ km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw += rc_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / 784.0f * MAX_CHASSIS_VR_SPEED ;//+ rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW ;//-fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;// - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =ins_data.pitch;

    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO) {

        gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;//上位机自瞄
        gim_cmd.pitch = trans_fdb.pitch + 100* rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT - Ballistic * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;//上位机自瞄
    }
    /* 限制云台角度 */

    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    //尽量实现DT7状态机的全部功能
    //优先考虑失能状态设计



    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    //实现连发即可


    /*堵弹反转检测*/
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }
}

#endif

#ifdef BSP_USING_RC_KEYBOARD
static void remote_to_cmd_pc_DT7(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;


// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/


}

static void remote_to_cmd_sbus(void)
{
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
    //  float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
    // float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
    // Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx += rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_SBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED; // + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy += rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_SBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED ; //+ km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw += rc_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_SBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED ;//+ rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW / RC_SBUS_MAX_VALUE * RC_DBUS_MAX_VALUE ;//-fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT / RC_SBUS_MAX_VALUE * RC_DBUS_MAX_VALUE;// - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =ins_data.pitch;

    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO) {

        gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW/ RC_SBUS_MAX_VALUE * RC_DBUS_MAX_VALUE;//上位机自瞄
        gim_cmd.pitch = trans_fdb.pitch + 100* rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT / RC_SBUS_MAX_VALUE * RC_DBUS_MAX_VALUE;//- Ballistic * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;//上位机自瞄
    }
    /* 限制云台角度 */

    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    //尽量实现DT7状态机的全部功能
    //优先考虑失能状态设计
    //sw3为上时，均RELAX,为中云台为GYRO,为下时为AUTO
    //SW2为上时底盘为FOLLOW,无需follow则为开环，为下时为SPIN
    switch (rc_now->sw2)
    {
        case RC_UP:
            //当云台为auto或GYRO才需要底盘跟随
            if(gim_cmd.ctrl_mode != GIMBAL_INIT && gim_cmd.ctrl_mode != GIMBAL_RELAX)
            {
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            else{
                chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            }
            break;
        case RC_DN:
            if(gim_cmd.ctrl_mode != GIMBAL_INIT && gim_cmd.ctrl_mode != GIMBAL_RELAX) {
                chassis_cmd.ctrl_mode = CHASSIS_SPIN;
            }
            if ((gim_cmd.ctrl_mode==GIMBAL_GYRO)||(gim_cmd.ctrl_mode==GIMBAL_AUTO))
            {
                chassis_cmd.vw =  rc_now->ch5 ;//*k
            }
            break;
    }
    switch(rc_now->sw3)
    {
        case RC_UP:
            gim_cmd.ctrl_mode = GIMBAL_RELAX;
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            shoot_cmd.ctrl_mode=SHOOT_STOP;
            gim_cmd.pitch=0;
            gim_cmd.yaw=0;
            break;
        case RC_MI:
            //open先判断是否已经归中，若之前状态为relax，则先归中
            if(gim_cmd.last_mode == GIMBAL_RELAX)
            {
                gim_cmd.ctrl_mode = GIMBAL_INIT;
            }
            else
            {
                //判断归中是否完成
                if(gim_fdb.back_mode == BACK_IS_OK)
                {
                    gim_cmd.ctrl_mode = GIMBAL_GYRO;
                }
            }
            break;
        case RC_DN:
            if(gim_cmd.last_mode == GIMBAL_RELAX)
            {//先判断是否已经归中，若之前状态为relax，则先归中
                gim_cmd.ctrl_mode = GIMBAL_INIT;
            }
            else
            {   //判断归中是否完成
                if(gim_fdb.back_mode == BACK_IS_OK)
                {
                    gim_cmd.ctrl_mode = GIMBAL_AUTO;
                }
            }
            break;
    }


    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    //实现连发即可
    // sw1使能发射
    //sw4切换单发与连发
    //ch6控制弹频与弹舱开合

    switch(rc_now->sw1)
    {
        case RC_UP:
        {
            shoot_cmd.ctrl_mode=SHOOT_STOP;
            shoot_cmd.trigger_status=TRIGGER_OFF;
            break;
        }
        case RC_DN:
        {
            switch(rc_now->sw4)
            {
                case RC_DN:
                {
                    if(shoot_cmd.trigger_status == TRIGGER_ON)
                    {
                        shoot_cmd.ctrl_mode = SHOOT_ONE;
                    }
                    break;
                }
                case RC_UP:
                {
                    if(shoot_cmd.trigger_status == TRIGGER_ON)
                    {
                        shoot_cmd.ctrl_mode = SHOOT_COUNTINUE;
                    }
                    break;
                }
            }
            break;
        }


    }
    //若扳机开火，且为发射状态
  if((shoot_cmd.trigger_status == TRIGGER_ON)&&(shoot_cmd.ctrl_mode!=SHOOT_STOP)&&(shoot_cmd.ctrl_mode!=SHOOT_REVERSE)&&( rc_now->ch6>0))
    {
        shoot_cmd.shoot_freq = rc_now->ch6;//*k,k不确定，为过编译注释掉
    }
    //当旋钮值足够小开启弹舱填弹
    else if(rc_now->ch6<=-700)
    {
        shoot_cmd.cover_open=1;
    }
    //其余状态关闭
    else
    {
        shoot_cmd.cover_open=0;
    }
    /*堵弹反转检测*/
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }
}

#endif

