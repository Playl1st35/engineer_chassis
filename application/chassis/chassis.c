/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <math.h>

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define SPEED_COEF (-6876.6f)
#define OMEGA_COEF (6.0f)
#define GO_UP_FORWARD_SPEED (-1.0f)
#define GO_UP_FORWARD_TIME (2.0f)
/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chassis_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Local_s chassis_cmd_local;         // 底盘本地控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

#define CHASSIS_TASK_FREQUENCY 200 
#define LIFT_PERIOD (3.0f)
#define LIFT_HEIGHT 14.3f
#define LEG_HEIGHT (0.57)
#define LEFT_LEG_GRAVITY_COMPENSATION 1.0 // 左腿重力补偿,根据实际情况调整
#define EXTRA_TORQUE 0.0f
#define LEG_LIFT (0.03)

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static DMMotorInstance *lift_motor,*leg_left_motor,*leg_right_motor;
static Lift_Motor_State_e lift_motor_state;
static Lift_Motor_State_e leg_motor_state;
static GoUpStairs_Step_e goupstairs_step = STEP_ONE_LIFT_ALL;
static float goupstairs_step_enter_ms =0.0f;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5, // 
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.can_init_config.can_handle = &hcan1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.can_init_config.can_handle = &hcan1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    MIT_Init_Config_s dm_motor_config = {
        .controller_param_init_config = {
            .Kp = 400,
            .Kd = 5,
        },
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    };
    //两个腿电机，总是同时运动
    dm_motor_config.can_init_config.tx_id = 0x03;
    dm_motor_config.can_init_config.rx_id = 0x13;
    dm_motor_config.can_init_config.can_handle = &hcan2;
    dm_motor_config.motor_type = DM8009;
    leg_left_motor = MITMotorInit(&dm_motor_config);

    dm_motor_config.can_init_config.tx_id = 0x04;
    dm_motor_config.can_init_config.rx_id = 0x14;
    dm_motor_config.can_init_config.can_handle = &hcan1;
    dm_motor_config.motor_type = DM8009;
    leg_right_motor = MITMotorInit(&dm_motor_config);
    // 升降电机
    dm_motor_config.controller_param_init_config.Kp = 200;//20
    dm_motor_config.controller_param_init_config.Kd = 5;
    dm_motor_config.can_init_config.tx_id = 0x05;
    dm_motor_config.can_init_config.rx_id = 0x15;
    dm_motor_config.can_init_config.can_handle = &hcan2;
    dm_motor_config.motor_type = DM4340;
    lift_motor = MITMotorInit(&dm_motor_config);


    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI

    // SuperCap_Init_Config_s cap_conf = {
    //     .can_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 0x302, // 超级电容默认接收id
    //         .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    //     }};
    // cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan3,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chassis_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_local.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_local.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy - chassis_cmd_local.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_local.wz * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据 
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}
//暂定从底部到顶部共2s，使用差值控制

static void LiftMotorControl()
{
    static float step = (float)LIFT_HEIGHT / ((LIFT_PERIOD * 0.85) * CHASSIS_TASK_FREQUENCY);
    static float target_pos;
    static Lift_Motor_State_e last_state;
    lift_motor_state = chassis_cmd_local.lift_motor_state;
    static int lift_down_cnt = 0;
    static uint8_t state_init_flag;
    static uint8_t set_zero_point_flag;
    if(lift_motor_state != last_state){
        state_init_flag = 0; // 状态改变，重置状态初始化标志
    }
    switch (lift_motor_state)
    {
    case LIFT_DOWN:
        if(chassis_feedback_data.lift_init_flag == 0 && lift_motor->stop_flag == MOTOR_ENALBED){//该初始化完成后才可以进行后续操作！
            lift_motor->angle_PID.Kp = 0;//不开启位置环
            DMMotorSetMITSpeed(lift_motor, -6);
            if(lift_motor->measure.velocity > -2) lift_down_cnt++;
            else lift_down_cnt = 0;
            if(lift_down_cnt > 150) // 连续150次检测到速度大于-10则认为已经到底,需要调整这个阈值和次数
            {
                lift_motor_state = LIFT_DOWN_LOCK;
                lift_down_cnt = 0;
                chassis_feedback_data.lift_init_flag = 1; // 升降电机完成初始化
                lift_motor->bottom_pos = lift_motor->measure.position; // 记录升降电机的最低位置
                lift_motor->top_pos = lift_motor->bottom_pos + (float)LIFT_HEIGHT;
                if(lift_motor->top_pos <= 12.5){
                    set_zero_point_flag = 1; 
                }
                DMMotorSetMITSpeed(lift_motor, 0);
                lift_motor->angle_PID.Kp = 20; // 恢复正常的Kp
            }
        }
        else if(chassis_feedback_data.lift_init_flag == 1){
            if(!state_init_flag){
                target_pos = lift_motor->measure.position; // 状态改变时以当前位置为起点
                state_init_flag = 1;
            }
            target_pos -= step; // 以固定步长降低目标位置
            if(target_pos < lift_motor->bottom_pos + 0.1) 
            {
                target_pos = lift_motor->bottom_pos; // 不超过最低位置
                lift_motor_state = LIFT_DOWN_LOCK; // 到达最低位置后切换状态
            }
            DMMotorSetMITAngle(lift_motor, target_pos); // 位置控制
        }
        break;    
    case LIFT_DOWN_LOCK:
        DMMotorSetMITAngle(lift_motor, lift_motor->bottom_pos - 0.5);
        break;
    case LIFT_UP:
        if(!set_zero_point_flag){
            if(!state_init_flag){
                target_pos = lift_motor->measure.position; // 状态改变时以当前位置为起点
                state_init_flag = 1;
            }
            if(lift_motor->measure.position >= 10.0f){
                DMMotorSetMode(DM_CMD_ZERO_POSITION,lift_motor);
                target_pos -= lift_motor->measure.position; // 更新目标位置,使其相对于新的零点
                set_zero_point_flag = 1;
                lift_motor->bottom_pos = -lift_motor->measure.position; // 将当前位置设为新的零点
                lift_motor->top_pos = lift_motor->bottom_pos + (float)LIFT_HEIGHT;

            }
            DMMotorSetMITAngle(lift_motor, target_pos); // 位置控制
            target_pos += step; // 以固定步长提高目标位置
        }
        if(set_zero_point_flag){
            if(chassis_feedback_data.lift_init_flag == 1){
                if(!state_init_flag){
                    target_pos = lift_motor->measure.position; // 状态改变时以当前位置为起点
                    state_init_flag = 1;
                }
                target_pos += step; // 以固定步长提高目标位置
                if(target_pos > lift_motor->top_pos - 0.1) 
                {
                    target_pos = lift_motor->top_pos; // 不超过最高位置
                    lift_motor_state = LIFT_UP_LOCK; // 到达最高位置后切换状态
                }
                DMMotorSetMITAngle(lift_motor, target_pos); // 位置控制
            }
        }
        break;
     case LIFT_UP_LOCK:
        DMMotorSetMITAngle(lift_motor, lift_motor->top_pos);
        break;
    default:
        break;
    }
    last_state = lift_motor_state;
}
//默认腿部在为收起状态，此时即为最低点
static void LegMotorControl()
{
    static float target_pos_left, target_pos_right;
    static float step = ((float)LEG_HEIGHT) / (LIFT_PERIOD * CHASSIS_TASK_FREQUENCY);
    leg_motor_state = chassis_cmd_local.leg_motor_state;
    static uint8_t state_init_flag;
    static Lift_Motor_State_e last_state;
    if(leg_motor_state != last_state){
        state_init_flag = 0; // 状态改变，重置状态初始化标志
    }
    switch (leg_motor_state)
    {
    case LIFT_DOWN:
        if(chassis_feedback_data.leg_init_flag == 0 && leg_left_motor->stop_flag == MOTOR_ENALBED){
            leg_left_motor->bottom_pos = leg_left_motor->measure.position; // 记录腿电机的最低位置（数值更大）
            leg_left_motor->top_pos = leg_left_motor->bottom_pos + (float)(LEG_HEIGHT); // 记录腿电机的最高位置（数值更小）
            leg_right_motor->bottom_pos = leg_right_motor->measure.position;
            leg_right_motor->top_pos = leg_right_motor->bottom_pos - (float)(LEG_HEIGHT);
            chassis_feedback_data.leg_init_flag = 1; // 腿电机完成初始化
            leg_motor_state = LIFT_DOWN_LOCK;
        }
        else if(chassis_feedback_data.leg_init_flag == 1){
            DMMotorSetMITTorque(leg_left_motor, LEFT_LEG_GRAVITY_COMPENSATION); // 开启重力补偿,需要根据实际情况调整补偿力矩的大小
            if(!state_init_flag){
                target_pos_left = leg_left_motor->measure.position; // 状态改变时以当前位置为起点
                target_pos_right = leg_right_motor->measure.position;
                state_init_flag = 1;
            }
            // 物理上，最高点的pos比最低点小（pos减小为抬起），因此向下（LIFT_DOWN）应当使pos增加
            target_pos_left -= step; // 以固定步长降低目标位置（数值增加表示下降）
            target_pos_right += step;
            if(target_pos_left <= leg_left_motor->bottom_pos + 3.0f * step + LEG_LIFT  && target_pos_right >= leg_right_motor->bottom_pos - 3.0f * step - LEG_LIFT){
                target_pos_left = leg_left_motor->bottom_pos + LEG_LIFT; // 不超过最低位置
                target_pos_right = leg_right_motor->bottom_pos - LEG_LIFT;
                leg_motor_state = LIFT_DOWN_LOCK; // 到达最低位置后切换状态
            }
            DMMotorSetMITAngle(leg_left_motor, target_pos_left); // 位置控制
            DMMotorSetMITAngle(leg_right_motor, target_pos_right);
        }
        break;
    case LIFT_DOWN_LOCK:
        DMMotorSetMITAngle(leg_left_motor, leg_left_motor->bottom_pos + LEG_LIFT);
        DMMotorSetMITAngle(leg_right_motor, leg_right_motor->bottom_pos - LEG_LIFT);
        break;
    case LIFT_UP:
        if(chassis_feedback_data.leg_init_flag == 1){
            if(!state_init_flag){
                target_pos_left = leg_left_motor->measure.position; // 状态改变时以当前位置为起点
                target_pos_right = leg_right_motor->measure.position;
                state_init_flag = 1;
            }
            // 抬起时pos应当减小
            target_pos_left += step; // 以固定步长提高目标位置（数值减小表示抬起）
            target_pos_right -= step;
            if(target_pos_left >= leg_left_motor->top_pos - 3.0f * step && target_pos_right <= leg_right_motor->top_pos + 3.0f * step) 
            {
                target_pos_left = leg_left_motor->top_pos; // 不超过最高位置
                target_pos_right = leg_right_motor->top_pos;
                leg_motor_state = LIFT_UP_LOCK; // 到达最高位置后切换状态
            }
            DMMotorSetMITAngle(leg_left_motor, target_pos_left); // 位置控制
            DMMotorSetMITAngle(leg_right_motor, target_pos_right);
        }
        DMMotorSetMITTorque(leg_left_motor, LEFT_LEG_GRAVITY_COMPENSATION + 
                                            EXTRA_TORQUE - (EXTRA_TORQUE / LEG_HEIGHT) * (leg_left_motor->measure.position)); // 抬起时增加额外的力矩
        DMMotorSetMITTorque(leg_right_motor, -(EXTRA_TORQUE - (EXTRA_TORQUE / LEG_HEIGHT) * (-leg_right_motor->measure.position)));
        break;
    case LIFT_UP_LOCK:
        DMMotorSetMITTorque(leg_left_motor, LEFT_LEG_GRAVITY_COMPENSATION);
        DMMotorSetMITAngle(leg_left_motor, leg_left_motor->top_pos);
        DMMotorSetMITAngle(leg_right_motor, leg_right_motor->top_pos);
        break;
    default:
        break;
    }
    // if(leg_left_motor->measure.position > leg_left_motor->bottom_pos) leg_left_motor->bottom_pos = leg_left_motor->measure.position; // 如果检测到比记录的最低位置更低的位置则更新最低位置
    // if(leg_right_motor->measure.position < leg_right_motor->bottom_pos) leg_right_motor->bottom_pos = leg_right_motor->measure.position;
    last_state = leg_motor_state;
}

static void Recv2Local()
{
    chassis_cmd_local.chassis_mode = chassis_cmd_recv.chassis_mode;
    chassis_cmd_local.vx = chassis_cmd_recv.vx * SPEED_COEF;
    chassis_cmd_local.vy = chassis_cmd_recv.vy * SPEED_COEF;
    chassis_cmd_local.wz = chassis_cmd_recv.wz * OMEGA_COEF;
    if(chassis_cmd_recv.chassis_mode == CHASSIS_GO_UP_STAIRS){
        return;
    }
    if(chassis_cmd_recv.chassis_rise_flag == CHASSIS_RISE_ON){
        if((chassis_cmd_local.lift_motor_state == LIFT_DOWN_LOCK || chassis_cmd_local.lift_motor_state == LIFT_DOWN) && (chassis_feedback_data.lift_init_flag == 1)){
            chassis_cmd_local.lift_motor_state = LIFT_UP;
        }
        if((chassis_cmd_local.leg_motor_state == LIFT_DOWN_LOCK || chassis_cmd_local.leg_motor_state == LIFT_DOWN) && (chassis_feedback_data.leg_init_flag == 1)){
            chassis_cmd_local.leg_motor_state = LIFT_UP;
        }
    }
    else if(chassis_cmd_recv.chassis_rise_flag == CHASSIS_RISE_OFF){
        if(chassis_cmd_local.lift_motor_state == LIFT_UP_LOCK || chassis_cmd_local.lift_motor_state == LIFT_UP){
            chassis_cmd_local.lift_motor_state = LIFT_DOWN;
        }
        if(chassis_cmd_local.leg_motor_state == LIFT_UP_LOCK || chassis_cmd_local.leg_motor_state == LIFT_UP){
            chassis_cmd_local.leg_motor_state = LIFT_DOWN;
        }
    }
}

static void GoUpStairs()
{

    switch (goupstairs_step)
    {
    case STEP_ONE_LIFT_ALL:
        chassis_cmd_local.vx = 0;
        chassis_cmd_local.vy = 0;
        chassis_cmd_local.wz = 0;
        chassis_cmd_local.lift_motor_state = LIFT_UP;
        chassis_cmd_local.leg_motor_state = LIFT_UP;
        if(lift_motor_state == LIFT_UP_LOCK && leg_motor_state == LIFT_UP_LOCK){
            goupstairs_step = STEP_TWO_MOVE_FRONT;
            goupstairs_step_enter_ms = DWT_GetTimeline_ms();
        }
        break;
    case STEP_TWO_MOVE_FRONT:
        chassis_cmd_local.vx = GO_UP_FORWARD_SPEED * SPEED_COEF;
        chassis_cmd_local.vy = 0;
        chassis_cmd_local.wz = 0;
        if(DWT_GetTimeline_ms() - goupstairs_step_enter_ms >= GO_UP_FORWARD_TIME * 1000.0f){
            goupstairs_step = STEP_THREE_LOWER_LIFT;
        }
        break;
    case STEP_THREE_LOWER_LIFT:
        chassis_cmd_local.vx = 0;
        chassis_cmd_local.vy = 0;
        chassis_cmd_local.wz = 0;
        chassis_cmd_local.lift_motor_state = LIFT_DOWN;
        if(lift_motor_state == LIFT_DOWN_LOCK){
            goupstairs_step = STEP_FOUR_LOWER_LEG;
            goupstairs_step_enter_ms = DWT_GetTimeline_ms();
        }
        break;
    case STEP_FOUR_LOWER_LEG:
        chassis_cmd_local.vx = GO_UP_FORWARD_SPEED * SPEED_COEF;
        chassis_cmd_local.vy = 0;
        chassis_cmd_local.wz = 0;
        if(DWT_GetTimeline_ms() - goupstairs_step_enter_ms >= GO_UP_FORWARD_TIME * 1000.0f){
            chassis_cmd_local.leg_motor_state = LIFT_DOWN;
            if(leg_motor_state == LIFT_DOWN_LOCK){
                goupstairs_step = STEP_FIVE_WAIT;
            }
        }
        break;
    case STEP_FIVE_WAIT:
        chassis_cmd_local.vx = GO_UP_FORWARD_SPEED * SPEED_COEF;
        chassis_cmd_local.vy = 0;
        chassis_cmd_local.wz = 0;
        break;
    default:
        break;
    }
}

/* 机器人底盘控制核心任务 */
//200Hz
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_mode_e last_chassis_mode = chassis_cmd_recv.chassis_mode;
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chassis_can_comm);
    if(last_chassis_mode == CHASSIS_GO_UP_STAIRS && chassis_cmd_recv.chassis_mode != CHASSIS_GO_UP_STAIRS){
        goupstairs_step = STEP_ONE_LIFT_ALL; // 如果从上楼模式切换到其他模式，则重置上楼步骤
        goupstairs_step_enter_ms = 0.0f;
    }
#endif // CHASSIS_BOARD
    Recv2Local();
    if (chassis_cmd_local.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DMMotorStop(lift_motor);
        DMMotorStop(leg_left_motor);
        DMMotorStop(leg_right_motor);
    }
    else if(chassis_cmd_local.chassis_mode == CHASSIS_MOVE_ROTATE)
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DMMotorEnable(lift_motor);
        DMMotorEnable(leg_left_motor);
        DMMotorEnable(leg_right_motor);
    }
    else if(chassis_cmd_local.chassis_mode == CHASSIS_GO_UP_STAIRS){
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DMMotorEnable(lift_motor);
        DMMotorEnable(leg_left_motor);
        DMMotorEnable(leg_right_motor);
        GoUpStairs();
    }
    else
    {
        // 其他模式暂不处理
    }
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    chassis_vx = chassis_cmd_local.vx;
    chassis_vy = chassis_cmd_local.vy;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    LiftMotorControl();
    LegMotorControl();
    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();


#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chassis_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}