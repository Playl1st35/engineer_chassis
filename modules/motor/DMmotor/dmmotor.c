#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
//#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    uint8_t ERR;
    uint8_t ID;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    ID = rxbuff[0] & 0x0F;
    measure->id = ID;

    ERR = (rxbuff[0]>>4) & 0x0F;
    measure->state = ERR;
    
    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);
    // DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    motor->working_type = current_mode;
    return motor;
}




DMMotorInstance *MITMotorInit(MIT_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings.motor_reverse_flag = config->motor_reverse_flag;

    motor->angle_PID.Kp = config->controller_param_init_config.Kp;
    motor->angle_PID.Kd = config->controller_param_init_config.Kd;


    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    motor->working_type = MIT_mode;

    //DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}


void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}


/// @brief MIT模式下设置各种参数，一般来说不调用速度函数，只调用角度函数来控制角度，力矩用来做前馈和重力补偿
void DMMotorSetMITAngle(DMMotorInstance *motor, float angle)//angle应该是rad
{
    motor->MIT_ref_angle = angle;
}

void DMMotorSetMITSpeed(DMMotorInstance *motor, float speed)
{
    motor->MIT_ref_speed = speed;
}

void DMMotorSetMITTorque(DMMotorInstance *motor, float torque)
{
    motor->MIT_ref_torque = torque;
}




void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{

    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

int a[3];
float torque_given;

void DMMotorTask(void const *argument)
{
    float  pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
   //DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    //CANInstance *motor_can = motor->motor_can_instace;
    //uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    if(motor->working_type == MIT_mode)
    {
        while (1)
        {

            if(motor->measure.state == 0x00){
                DMMotorEnable(motor);
                DMMotorSetMode(DM_CMD_MOTOR_MODE,motor);
            }
            pid_ref = motor->pid_ref;
            set = pid_ref;
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set *= -1;
            LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
            motor_send_mailbox.position_des = float_to_uint(motor->MIT_ref_angle, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(motor->MIT_ref_speed, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(motor->MIT_ref_torque, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = float_to_uint(motor->angle_PID.Kp, KP_MIN, KP_MAX, 12);
            motor_send_mailbox.Kd = float_to_uint(motor->angle_PID.Kd, KD_MIN, KD_MAX, 12);

            // if((motor->measure.torque >= 10  || motor->measure.torque <= -10))  a[0] = 1;
            // else                                                                a[0] = 0;

            // if(motor->motor_can_instace->tx_id == 0x07)                         a[1] = 1;
            // else                                                                a[1] = 0;

            // if((((motor->MIT_ref_angle - motor->measure.position) >= 0.08 || ((motor->MIT_ref_angle - motor->measure.position) <= -0.08))))
            // a[2] = 1;
            // else a[2] = 0;
            // if(motor->motor_can_instace->tx_id == 0x07)
            // {
            //     torque_given = (motor->MIT_ref_angle - motor->measure.position)*100 + (motor->MIT_ref_speed-motor->measure.velocity) * 5 + motor->MIT_ref_torque;
            //     if(torque_given >= 10)
            //     {
            //         motor_send_mailbox.torque_des = float_to_uint(10, DM_T_MIN, DM_T_MAX, 12);
            //         motor_send_mailbox.Kp =         float_to_uint(0, KP_MIN, KP_MAX, 12);
            //         motor_send_mailbox.Kd =         float_to_uint(0, KD_MIN, KD_MAX, 12);
            //     }
            //     else if(torque_given <= -10)
            //     {
            //         motor_send_mailbox.torque_des = float_to_uint(-10, DM_T_MIN, DM_T_MAX, 12);
            //         motor_send_mailbox.Kp =         float_to_uint(0, KP_MIN, KP_MAX, 12);
            //         motor_send_mailbox.Kd =         float_to_uint(0, KD_MIN, KD_MAX, 12);
            //     }
            //     else
            //     {
            //         motor_send_mailbox.Kp = float_to_uint(motor->angle_PID.Kp, KP_MIN, KP_MAX, 12);
            //         motor_send_mailbox.Kd = float_to_uint(motor->angle_PID.Kd, KD_MIN, KD_MAX, 12);
            //     }
            // }


            // if((motor->measure.torque >= 8  || motor->measure.torque <= -8) && 
            //    motor->motor_can_instace->tx_id == 0x07 && 
            //  (((motor->MIT_ref_angle - motor->measure.position) >= (0.06)) || ((motor->MIT_ref_angle - motor->measure.position) <= (-0.06))))
            // {
            //     if(motor->measure.torque >= 0)
            //     {
            //         motor_send_mailbox.torque_des = float_to_uint(9, DM_T_MIN, DM_T_MAX, 12);
            //         motor_send_mailbox.Kp =         float_to_uint(0, KP_MIN, KP_MAX, 12);
            //         motor_send_mailbox.Kd =         float_to_uint(0, KD_MIN, KD_MAX, 12);
            //     }
            //     else if(motor->measure.torque < 0)
            //     {
            //         motor_send_mailbox.torque_des = float_to_uint(-9, DM_T_MIN, DM_T_MAX, 12);
            //         motor_send_mailbox.Kp =         float_to_uint(0, KP_MIN, KP_MAX, 12);
            //         motor_send_mailbox.Kd =         float_to_uint(0, KD_MIN, KD_MAX, 12);
            //     }
            // }
            // else
            // {
            //     motor_send_mailbox.Kp = float_to_uint(motor->angle_PID.Kp, KP_MIN, KP_MAX, 12);
            //     motor_send_mailbox.Kd = float_to_uint(motor->angle_PID.Kd, KD_MIN, KD_MAX, 12);
            // }

            if(motor->stop_flag == MOTOR_STOP)
            {
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
                motor_send_mailbox.Kp =         float_to_uint(0, KP_MIN, KP_MAX, 12);
                motor_send_mailbox.Kd =         float_to_uint(0, KD_MIN, KD_MAX, 12);
            }
                
            
            motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
            motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
            motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
            motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
            motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
            motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
            motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
            motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

            CANTransmit(motor->motor_can_instace, 1);

            osDelay(2);
            
            
        }
    }
    else
    {
        while (1)
        {
            pid_ref = motor->pid_ref;
            
            set = pid_ref;
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set *= -1;
        
            LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;

            if(motor->stop_flag == MOTOR_STOP)
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

            motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
            motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
            motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
            motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
            motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
            motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
            motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
            motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

            CANTransmit(motor->motor_can_instace, 1);

            osDelay(2);
        }
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}