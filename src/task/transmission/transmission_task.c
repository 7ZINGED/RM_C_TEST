/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
* 2023-12-09      YangShuo     USB虚拟串口
*/

#include "transmission_task.h"
#include "drv_gpio.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#define HEART_BEAT 500 //ms
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct ins_msg ins_data;
static struct gimbal_fdb_msg gim_fdb;
static struct trans_fdb_msg trans_fdb;
/*------------------------------传输数据相关 --------------------------------- */
#define RECV_BUFFER_SIZE 64  // 接收环形缓冲区大小
static rt_uint8_t r_buffer[RECV_BUFFER_SIZE];  // 接收环形缓冲区
struct rt_ringbuffer receive_buffer ; // 环形缓冲区对象控制块指针
rt_uint8_t *r_buffer_point; //用于清除环形缓冲区buffer的指针
rt_uint8_t buf[31] = {0};
RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = GIMBAL,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
RpyTypeDef rpy_rx_data; //接收解析结构体
static rt_uint32_t heart_dt;//心跳数据相关
/* ---------------------------------usb虚拟串口相关 --------------------------------- */
static rt_device_t vs_port = RT_NULL;
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static publisher_t *pub_trans;
static subscriber_t *sub_cmd,*sub_ins,*sub_gim;
static void trans_sub_pull(void);
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_pub_init(void);

/*------------------------------自瞄相对角传参反馈--------------------------------------*/
extern auto_relative_angle_status_e auto_relative_angle_status;
/**
 * @brief trans 线程中所有订阅者初始化（如有其它数据需求可在其中添加）
 */
static void trans_sub_init(void)
{
    sub_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));

}

/**
 * @brief trans 线程中所有订阅者获取更新话题（如有其它数据需求可在其中添加）
 */
static void trans_sub_pull(void)
{
    sub_get_msg(sub_cmd, &gim_cmd);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_gim, &gim_fdb);
}

/**
 * @brief cmd 线程中所有发布者初始化
 */
static void trans_pub_init(void)
{
    pub_trans = pub_register("trans_fdb",sizeof(struct trans_fdb_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void trans_pub_push(void)
{
    pub_push_msg(pub_trans,&trans_fdb);
}

/* --------------------------------- 通讯线程入口 --------------------------------- */
static float trans_dt;

void transmission_task_entry(void* argument)
{
    static float trans_start;
    static float heart_start;

    /*订阅数据初始化*/
    trans_sub_init();
    /*发布数据初始化*/
    trans_pub_init();
    /* step1：查找名为 "vcom" 的虚拟串口设备*/
    vs_port = rt_device_find("vcom");
    /* step2：打开串口设备。以中断接收及轮询发送模式打开串口设备*/
   rt_device_open(vs_port, RT_DEVICE_FLAG_INT_RX);
    /*环形缓冲区初始化*/
    rt_ringbuffer_init(&receive_buffer, r_buffer, RECV_BUFFER_SIZE);
    /*清除buffer的指针赋地址*/
    r_buffer_point = r_buffer;
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(vs_port, usb_input);
    LOG_I("Transmission Task Start");
    heart_dt = dwt_get_time_ms();
    while (1)
    {
        trans_start = dwt_get_time_ms();
        /*订阅数据更新*/
        trans_sub_pull();
        /* 发布数据更新 */
        trans_pub_push();
/*--------------------------------------------------具体需要发送的数据--------------------------------- */
//加分项：添加心跳数据判断异常并重启虚拟串口
//当有心跳数据但心跳数据内容异常
//为确认下位机可以接收到上位机数据，故这里加一个延时来确保有心跳和无心跳时，线程运行时间不同，接收数据间隔也不同
        if((dwt_get_time_ms()-heart_dt)>HEARTBEAT)
        {
            rt_device_close(vs_port);
            rt_device_open(vs_port, RT_DEVICE_FLAG_INT_RX);;
           // rt_device_control(vs_port, RT_DEVICE_CTRL_RESUME, (void *) RT_DEVICE_FLAG_INT_RX);
            rt_thread_delay(5000);
        }



//发送数据，将ins_data结构体中的yaw,pitch,roll数据打包成特定格式的数据帧，并进行校验后通过串口发送到 PC。
Send_to_pc(rpy_tx_data);
/*--------------------------------------------------具体需要发送的数据---------------------------------*/
        /* 用于调试监测线程调度使用 */
        trans_dt = dwt_get_time_ms() - trans_start;
        if (trans_dt > 1)
            LOG_E("Transmission Task is being DELAY! dt = [%f]", &trans_dt);
        //TODO:为保证流畅运行这里改为1000，原为1
        rt_thread_mdelay(1000);
    }
}
//发送函数
void Send_to_pc(RpyTypeDef data_r)
{
    /*填充数据*/
    float yaw = ins_data.yaw;
    float roll = ins_data.yaw;
    float pitch = ins_data.yaw;
//应与初始位置计算得知
    pack_Rpy(&data_r, -(gim_fdb.yaw_offset_angle - ins_data.yaw), gim_fdb.pit_offset_angle-ins_data.pitch, ins_data.roll);
    Check_Rpy(&data_r);
    /*发送数据*/
    rt_device_write(vs_port, 0, (uint8_t*)&data_r, sizeof(data_r));
}

//数据打包函数，参考提供的代码填写
void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch,float roll)
{
    //DATA第0位为为0，为绝对角度控制
    //小端模式，低位在前，高位在后
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    int32_t rpy_practical_data = 0;
    //将data的地址转化为uint32整型的地址并给rpy，则对rpy_tx_buffer元素的地址直接对*rpy移位即可
    uint32_t *rpy = (uint32_t *)&rpy_practical_data;
    rpy_tx_buffer[0] = 0;

    rpy_practical_data = yaw * 1000;
    rpy_tx_buffer[1] = *rpy;
    rpy_tx_buffer[2] = *rpy >> 8;
    rpy_tx_buffer[3] = *rpy >> 16;
    rpy_tx_buffer[4] = *rpy >> 24;

    rpy_practical_data = pitch * 1000;
    rpy_tx_buffer[5] = *rpy;
    rpy_tx_buffer[6] = *rpy >> 8;
    rpy_tx_buffer[7] = *rpy >> 16;
    rpy_tx_buffer[8] = *rpy >> 24;

    rpy_practical_data = roll *1000;
    rpy_tx_buffer[9]  = *rpy;
    rpy_tx_buffer[10] = *rpy >> 8;
    rpy_tx_buffer[11] = *rpy >> 16;
    rpy_tx_buffer[12] = *rpy >> 24;
    memcpy(&frame->DATA[0], rpy_tx_buffer,13);
    frame->LEN = FRAME_RPY_LEN;
}

//木鸢通讯协议的校验函数
void Check_Rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;

}

/**
 * @brief 接收数据回调函数
 *该函数 `usb_input` 是一个处理从串口接收到的数据的回调函数。具体功能如下：

1. 重置初始化缓冲区

2.从虚拟串口device读取数据

3. 保存数据到环形缓冲区receive_buffer或者自定义正常数组缓冲区

4. 从缓冲区取出数据

5. 数据帧内容拷贝rpy_rx_data结构体中

6. 处理不同类型的数据帧并将数据存放至结构体trans_fdb汇总

7. 清零存放数据帧的数据结构体

8. 返回值
 * @param
 * @param
 */
//接收数据解析开始，根据ID判断信息种类，将解析后信息填写至结构体trans_fdb中
//接收数据解析结束
static rt_err_t usb_input(rt_device_t dev, rt_size_t size)
{
    heart_dt = dwt_get_time_ms();
//   1. 重置初始化缓冲区
    memset(buf, 0, sizeof(buf));
//   2.从虚拟串口device读取数据
    rt_uint32_t length;
//数据帧读取完成后才能进入下一步
    while ((length = rt_device_read(vs_port, 0, buf, sizeof(buf))) > 0)
    {
        //   3. 保存数据到环形缓冲区receive_buffer或者自定义正常数组缓冲区
//?
      //  rt_ringbuffer_put(&receive_buffer,buf,rx_lenth);
        rt_ringbuffer_put_force(&receive_buffer,buf,length);
    }


//   4. 从缓冲区取出数据
    rt_uint8_t rx_buf[sizeof(RpyTypeDef)]={0};
    rt_ringbuffer_get(&receive_buffer,rx_buf,sizeof(rx_buf));
//   5. 数据帧内容拷贝rpy_rx_data结构体中
    if( r_buffer[0]==0xFF)
    {
        memcpy(&rpy_rx_data, &rx_buf, sizeof(rpy_rx_data));


//  6. 处理不同类型的数据帧进行解包，并将数据存放至结构体trans_fdb汇总，参考提供的代码
        //心跳
        switch (rpy_rx_data.ID) {
            case GIMBAL: {
                if (rpy_rx_data.DATA[0])
                {
                    trans_fdb.yaw = (*(int32_t *) &rpy_rx_data.DATA[1] / 1000.0);
                    trans_fdb.pitch = (*(int32_t *) &rpy_rx_data.DATA[5] / 1000.0);
                }else {
                    //u1s1这里传值要不要有区别
                    trans_fdb.yaw = (*(int32_t *) &rpy_rx_data.DATA[1] / 1000.0);
                    trans_fdb.pitch = (*(int32_t *) &rpy_rx_data.DATA[5] / 1000.0);
                }

            }
                break;
            case HEARTBEAT:
            {
                trans_fdb.heartbeat = rpy_rx_data.DATA[0];
                //测试时未用到
                heart_dt = dwt_get_time_ms();

            }
                break;
        }

//   7. 清零存放数据帧的数据结构体
        memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
    }
//   8. 返回值
//    return RT_EOK;
    return RT_EOK;
}


