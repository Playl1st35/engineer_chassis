// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD
static Chassis_Ctrl_Local_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static FS_RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据


static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    rc_data = FS_RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口


#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 40 * (float)rc_data[TEMP].rc.rocker_l_; // _水平方向
    chassis_cmd_send.vy = 40 * (float)rc_data[TEMP].rc.rocker_l1; // 1数值方向
    chassis_cmd_send.wz = 3 * (float)rc_data[TEMP].rc.rocker_r_;
    if(chassis_cmd_send.vx < 50 && chassis_cmd_send.vx > -50) chassis_cmd_send.vx = 0; // 遥控器死区
    if(chassis_cmd_send.vy < 50 && chassis_cmd_send.vy > -50) chassis_cmd_send.vy = 0;
    if(chassis_cmd_send.wz < 50 && chassis_cmd_send.wz > -50) chassis_cmd_send.wz = 0;
        if (rc_data->rc.switch_right_1 == 0)
    {
        if(chassis_fetch_data.lift_motor_state != LIFT_DOWN_LOCK)
        {
            chassis_cmd_send.lift_motor_state = LIFT_DOWN;
        }
        else if(chassis_fetch_data.lift_motor_state == LIFT_DOWN_LOCK)
        {
            chassis_cmd_send.lift_motor_state = LIFT_DOWN_LOCK;
        }
    }
    else if (rc_data->rc.switch_right_1 == 2 && chassis_fetch_data.lift_init_flag == 1) // 升降电机初始化完成后才允许升起
    {
        if(chassis_fetch_data.lift_motor_state != LIFT_UP_LOCK)
        {
            chassis_cmd_send.lift_motor_state = LIFT_UP;
        }
        else if(chassis_fetch_data.lift_motor_state == LIFT_UP_LOCK)
        {
            chassis_cmd_send.lift_motor_state = LIFT_UP_LOCK;
        }
    }
    if(rc_data->rc.switch_right_2 == 0)
    {
        if(chassis_fetch_data.leg_motor_state != LIFT_DOWN_LOCK)
        {
            chassis_cmd_send.leg_motor_state = LIFT_DOWN;
        }
        else if(chassis_fetch_data.leg_motor_state == LIFT_DOWN_LOCK)
        {
            chassis_cmd_send.leg_motor_state = LIFT_DOWN_LOCK;
        }
    }
    else if(rc_data->rc.switch_right_2 == 2)
    {
        if(chassis_fetch_data.leg_motor_state != LIFT_UP_LOCK)
        {
            chassis_cmd_send.leg_motor_state = LIFT_UP;
        }
        else if(chassis_fetch_data.leg_motor_state == LIFT_UP_LOCK)
        {
            chassis_cmd_send.leg_motor_state = LIFT_UP_LOCK;
        }
    }
}


/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    if (rc_data->rc.switch_left_1 == 0) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        LOGERROR("[CMD] emergency stop!");
    }
    else if (rc_data->rc.switch_left_1 == 2)
    {
        robot_state = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }

}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    //CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠

    //RemoteControlSet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    VisionSend(&vision_send_data);
}
