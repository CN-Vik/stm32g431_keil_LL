/**
 ****************************************************************************************************
 * @file        foc_motor.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       FOC电机状态机
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_motor.h"
#include <foc_math.h>
#include <stdio.h>
#include <delay.h>

/**
 * @brief 电机结构体初始化
 * 
 * @param foc_motor 
 */
void FOC_Motor_Init(FOC_MOTOR_t *foc_motor)
{
    foc_motor->motor_state = MOTOR_IDLE;
}


/**
 * @brief 清除电机的FOC各项计算参数
 * 
 * @param foc_motor 
 */
static void FOC_Motor_Clear(FOC_MOTOR_t *foc_motor)
{
    abc_t abc_NULL  = {0};
    qd_t qd_NULL    = {0};
    alphabeta_t alphabeta_NULL = {0};

    /*FOC参数清除*/
    foc_motor->foc_var.I_abc = abc_NULL;
    foc_motor->foc_var.I_qd  = qd_NULL;
    foc_motor->foc_var.U_qd  = qd_NULL;
    foc_motor->foc_var.I_alphabeta = alphabeta_NULL;
    
    foc_motor->foc_var.I_qd_ref = qd_NULL;

    /*关闭PWM*/
    FOC_PWM_StopALL(&foc_motor->pwm);
}

/**
 * @brief 切换状态机的下个状态
 * 
 * @param foc_motor 
 * @param new_state 
 */
static void Motor_NextState(FOC_MOTOR_t *foc_motor, MOTOR_State_t new_state)
{
    foc_motor->motor_state = new_state;
}

/*电机开环运行*/

/**
 * @brief 直接输出Uq进行开环运行
 * 
 * @param foc_motor 
 */
void Motor_Run_Open(FOC_MOTOR_t *foc_motor)
{
    FOC_VF_t *vf = &foc_motor->sensorless.vf;

    vf->Uqd.q = 1;
    FOC_VF_Angle_Calc(vf);

    float ElAngle = vf->speed.ElAngle;

    
    qd_t In = {0};
    /*更新电流*/
    
    foc_motor->foc_var.I_abc = (foc_motor->current.I_abc);
    /*克拉克变换*/
    foc_motor->foc_var.I_alphabeta  = FOC_Clarke(foc_motor->foc_var.I_abc);

    In = Foc_Park(foc_motor->foc_var.I_alphabeta, ElAngle);

    FOC_PWM_Run(&foc_motor->pwm, vf->Uqd, ElAngle);

    foc_motor->foc_var.I_qd = In;

}


/**
 * @brief 无感方式运行
 * 
 * @param foc_motor 
 */
void Motor_SL_RUN(FOC_MOTOR_t *foc_motor)
{
    FOC_SENSORLESS_t *foc_sl = &foc_motor->sensorless;

    qd_t In = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};
    alphabeta_t Last_Ualphabeta = foc_motor->pwm.alpha_beta;

    /*更新电流*/
    Iabc = foc_motor->current.I_abc;
    /*克拉克变换*/
    I_alphabeta  = FOC_Clarke(Iabc);
    
    FOC_ABZENC_Angle_Calc(&foc_motor->enc);
    FOC_HALL_Angle_Calc(&foc_motor->hall);
    

    /*进入无感观测*/
    I_alphabeta = FOC_SL_RUN(&foc_motor->sensorless, I_alphabeta, Last_Ualphabeta, foc_motor->target_speed);
    ElAngle = foc_motor->sensorless.speed.ElAngle;

    /*计算两个观测器之间的角度误差*/
    // ElAngle = foc_motor->enc.speed.ElAngle;
    // foc_sl->err = FOC_ERR(Limit_Angle(ElAngle), Limit_Angle(foc_sl->flux.speed.ElAngle));    

    /*帕克变换*/
    In = Foc_Park(I_alphabeta, ElAngle);

    /*速度环控制*/    
    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;

    /*电流环PID*/
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq,  In.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id,  In.d) + foc_sl->d_bias;

    switch(foc_sl->sta){
        case VF_STA:
            PID_Clear(&foc_motor->pid_cur_iq);
            PID_Clear(&foc_motor->pid_cur_id);
            PID_Clear(&foc_motor->pid_speed);
            Out = foc_sl->vf.Uqd;
            break;
        case HFI_POL_STA:
            PID_Clear(&foc_motor->pid_cur_iq);
            PID_Clear(&foc_motor->pid_cur_id);
            PID_Clear(&foc_motor->pid_speed);
            Out.d = foc_sl->d_bias; 
            Out.q = 0;
            break;
        case HFI_STA:

            /*高频注入的时候不要对d轴电流进行PID控制，否则堵转下容易反转 并且要及时清除d轴PID参数，不然跳转到SMO会出问题*/
            PID_Clear(&foc_motor->pid_cur_id);
            Out.d = foc_sl->d_bias;      

            if(foc_motor->pid_cur_iq.ActualValue_limit != foc_motor->pid_qd_limit_hif){
                /*清除积分，设置高频注入q轴电流限制*/
                PID_Clear(&foc_motor->pid_cur_iq);
                foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit_hif;         /*高频注入状态 需要限制电流环*/ 
            }
            
            break;
        case SMO_STA:
            foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit;
            break;
        case FLUX_STA:
            foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit;
            break;
    }

    // Out = foc_sl->vf.Uqd;
    // Out.q = foc_sl->vf.Uqd.q;
    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

    /*计算三相反电动势*/

    /*保存参数*/
    foc_motor->foc_var.speed = foc_motor->sensorless.speed;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}

/**
 * @brief ABZ编码器方式运行
 * 
 * @param foc_motor 
 */
void Motor_ABZ_ENC_RUN(FOC_MOTOR_t *foc_motor)
{
    qd_t In = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = foc_motor->current.I_abc;
    /*克拉克变换*/
    I_alphabeta  = FOC_Clarke(Iabc);
             
    ElAngle = FOC_ABZENC_Angle_Calc(&foc_motor->enc);

    I_alphabeta = FOC_SL_RUN(&foc_motor->sensorless, I_alphabeta, foc_motor->pwm.alpha_beta, foc_motor->target_speed);

    /*帕克变换*/
    In = Foc_Park(I_alphabeta, ElAngle);

    /*电流环PID*/
    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq,  In.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id,  In.d);

    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

    /*保存参数*/
    foc_motor->foc_var.speed = foc_motor->enc.speed;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}

/**
 * @brief ABZ编码器校准
 * 
 * @param foc_motor 
 */
bool Motor_ABZ_ENC_CALIB(FOC_MOTOR_t *foc_motor)
{
    qd_t Out = {0, 0.5f};

    bool ret = false;
    float ElAngle = 0.0f;

    FOC_SENSORLESS_t *foc_sl = &foc_motor->sensorless;
    FOC_ABZENC_t *enc = &foc_motor->enc;
    foc_sl->sta = VF_STA;       /*设置为无感的VF运行*/

    foc_sl->vf.Uqd.q = 0;
    foc_sl->vf.Uqd.d = Out.d/20.0f;

    FOC_VF_Angle_Calc(&foc_sl->vf);
    ElAngle = foc_sl->vf.speed.ElAngle;

    switch (foc_motor->abzenc_calib_state){
        case 0:                 /*强拖定位*/
            if(enc->align){
                foc_motor->abzenc_calib_state = 1;
            }
            break;
        case 1:
            /* code */
            if(fabsf(ElAngle)<(foc_sl->vf.step*2)){
                foc_motor->abzenc_calib_state = 2;
            }
            break;
        case 2:
            ElAngle = 0;
            break;
        default:
            foc_sl->sta = FLUX_STA;
            foc_motor->abzenc_calib_state = 0;
            break;
    }


    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

    ret = FOC_ABZENC_Angle_Clib(enc);
    if(true==ret){
        foc_sl->sta = FLUX_STA;
        foc_motor->abzenc_calib_state = 0;
    }

    return ret;
}

/**
 * @brief 霍尔传感器方式运行
 * 
 * @param foc_motor 
 */
void Motor_HALL_RUN(FOC_MOTOR_t *foc_motor)
{
    qd_t In = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = foc_motor->current.I_abc;
    /*克拉克变换*/
    I_alphabeta  = FOC_Clarke(Iabc);
             
    ElAngle = FOC_HALL_Angle_Calc(&foc_motor->hall);

    /*帕克变换*/
    In = Foc_Park(I_alphabeta, ElAngle);

    /*电流环PID*/
    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq,  In.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id,  In.d);

    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);
    
    /*保存参数*/
    foc_motor->foc_var.speed = foc_motor->hall.speed;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}


/**
 * @brief 霍尔自学习运行
 * 
 */
bool Motor_HALL_LEARN(FOC_MOTOR_t *foc_motor)
{
    float err = 0;
    FOC_HALL_t *foc_hall = &foc_motor->hall;
    FOC_FLUX_t *foc_flux = &foc_motor->sensorless.flux;
    foc_motor->sensorless.sta = FLUX_STA;                                           /*使用磁链的方式自己自学习运转*/

    foc_hall->Uq = foc_motor->pwm.Uqd.q;
    FOC_HALL_Angle_Calc(foc_hall);

    /*使用无感匀速跑*/
    Motor_SL_RUN(foc_motor);

    // return false;
    switch(foc_hall->learning_state){
        case 0:
            /*准备正转学习设置*/
            foc_hall->selflearn = true;                 /*自学习开启*/
            foc_motor->target_speed = 1000;             /*设置无感运行速度*/
            foc_hall->learning_tick = 20000*10;         /*设置正转运行tick，大概十秒*/
            foc_hall->learning_state = 1;

            foc_hall->PhaseShift = 0;
            foc_hall->PhaseShift_c = 0;

            for(int i=0;i<6;i++){
                foc_hall->sector_size[i]=0;
                foc_hall->sector_size_c[i]=0;
                foc_hall->sector_pos[i]=0;
                foc_hall->sector_pos_c[i]=0;
            }

            printf("正转测试中---------------\r\n");
            break;
        case 1:
            /*正转测试中*/

            foc_hall->learning_tick--;
            if(foc_hall->learning_tick<1000){
                foc_hall->selflearn = false;
            }

            if(foc_hall->learning_tick<=0){
                /*保存正转参数 准备反转自学习*/
                for(int i=0;i<6;i++){
                    foc_hall->hall_learn.sector_pos[i] = foc_hall->hall_sector_pos_debug[i] *1000000;
                    foc_hall->hall_learn.sector_size[i] = foc_hall->hall_sector_size_debug[i] *1000000;

                    foc_hall->sector_pos[i] = foc_hall->hall_sector_pos_debug[i];
                    foc_hall->sector_size[i] = foc_hall->hall_sector_size_debug[i];

                    Move_Filter_Clear(&foc_hall->hall_sector_tick_filter[i]);
                }

                foc_hall->selflearn = true;                 /*自学习开启*/
                foc_hall->learning_tick = 20000*10;         /*设置正转运行tick，大概十秒*/
                foc_hall->learning_state = 2;

                Move_Filter_Clear(&foc_hall->pos_filter);

                
                for(int i=0;i<6;i++){
                    printf("正转 扇区[%d] 位置:%f  大小:%f\r\n", i, foc_hall->hall_sector_pos_debug[i], foc_hall->hall_sector_size_debug[i]);
                }
                
            }
            break;
        case 2:
            /*计算正转方向与0°的偏移值*/
            foc_hall->learning_tick--;
            err = FOC_ERR(Limit_Angle(foc_flux->speed.ElAngle), Limit_Angle(foc_hall->speed.ElAngle));

            Move_Filter_fill(&foc_hall->pos_filter, 100000*err);
            if(foc_hall->learning_tick<=0){
                foc_hall->hall_learn.PhaseShift[0] = Move_Filter_calculate(&foc_hall->pos_filter);
                foc_hall->PhaseShift = (float)foc_hall->hall_learn.PhaseShift[0] /100000.0f;
                
                foc_hall->learning_tick = 20000*10;         /*设置正转运行tick，大概十秒*/
                foc_motor->target_speed = -1000; 
                foc_hall->learning_state = 3;
                
                printf("计算正转方向与0°的偏移值: %f\r\n", foc_hall->PhaseShift);

                printf("反转测试中---------------\r\n");
            }
            break;
        case 3:
            /*反转测试中*/
            foc_hall->learning_tick--;
            if(foc_hall->learning_tick<1000){
                foc_hall->selflearn = false;
            }

            if(foc_hall->learning_tick<=0){
                for(int i=0;i<6;i++){
                    foc_hall->hall_learn.sector_pos_c[i] = foc_hall->hall_sector_pos_debug[i] *1000000;
                    foc_hall->hall_learn.sector_size_c[i] = foc_hall->hall_sector_size_debug[i] *1000000;


                    foc_hall->sector_pos_c[i] = foc_hall->hall_sector_pos_debug[i];
                    foc_hall->sector_size_c[i] = foc_hall->hall_sector_size_debug[i];

                    Move_Filter_Clear(&foc_hall->hall_sector_tick_filter[i]);
                }
    
                foc_hall->selflearn = true;                 /*自学习开启*/
                foc_hall->learning_tick = 20000*10;         /*设置反转运行tick，大概十秒*/
                foc_hall->learning_state = 4;

                Move_Filter_Clear(&foc_hall->pos_filter);
                for(int i=0;i<6;i++){
                    printf("反转 扇区[%d] 位置:%f  大小:%f\r\n", i, foc_hall->hall_sector_pos_debug[i], foc_hall->hall_sector_size_debug[i]);
                }
            }
            
            break;
        case 4: /*保存反转参数 准备反转自学习*/
            /*计算反转方向与0°的偏移值*/
            foc_hall->learning_tick--;

            err = FOC_ERR(Limit_Angle(foc_flux->speed.ElAngle), Limit_Angle(foc_hall->speed.ElAngle));

            Move_Filter_fill(&foc_hall->pos_filter, 100000*err);
            if(foc_hall->learning_tick<=0){
                foc_hall->hall_learn.PhaseShift[1] = Move_Filter_calculate(&foc_hall->pos_filter);
                foc_hall->PhaseShift_c = (float)foc_hall->hall_learn.PhaseShift[1] /100000.0f;
            
                foc_motor->target_speed = 0; 
                foc_hall->learning_state = 5;

                printf("计算反转方向与0°的偏移值: %f\r\n", foc_hall->PhaseShift_c);
            }

            break;
        case 5:
            
            foc_hall->selflearn = false;
            // stmflash_write(HALL_ADDR, (uint32_t*)&foc_hall->hall_learn, sizeof(foc_hall->hall_learn)/sizeof(uint32_t));
            foc_motor->target_speed = 0;
            foc_hall->learning_state = 0;

            for(int i=0;i<6;i++){
                Move_Filter_Clear(&foc_hall->hall_sector_tick_filter[i]);
            }

            Move_Filter_Clear(&foc_hall->Period_filter);
            Move_Filter_Clear(&foc_hall->Speed_filter);

            printf("霍尔自学习完成，准备退出---------------\r\n");
            return true;
        
        default:
            foc_hall->selflearn = false;
            break;
    }

    return false;
}



/**
 * @brief 运行电机状态机
 * 
 * @param foc_motor 
 */
void FOC_Motor_Run(FOC_MOTOR_t *foc_motor)
{
	MOTOR_State_t state = foc_motor->motor_state;

    switch(state){
        case MOTOR_IDLE:  /*空闲状态*/
            FOC_PWM_StopALL(&foc_motor->pwm);                           /*关闭所有PWM通道*/
            Motor_NextState(foc_motor, MOTOR_IDLE_START);               /*跳转到空闲启动*/
            printf("IDLE\r\n");    
            break;
        case MOTOR_IDLE_START:    
            if(0==FOC_CURRENT_OffsetCalc(&foc_motor->current)){
                FOC_PWM_StartAll(&foc_motor->pwm);                      /*打开所有PWM通道*/
                Motor_NextState(foc_motor, MOTOR_CLEAR);        
                printf("IDLE_START\r\n"); 
            }           
            break;   
        case MOTOR_IDENTIFY:   
                /*参数辨识做的不好, 暂时不用*/

            break;      
        case MOTOR_CLEAR:
            FOC_Motor_Clear(foc_motor);

            FOC_PWM_StartAll(&foc_motor->pwm);                                  /*打开所有PWM通道*/
                            
            Motor_NextState(foc_motor, MOTOR_START);                            /*跳转到启动*/
            printf("CLEAR\r\n"); 
            break;

        case MOTOR_START:
            Motor_NextState(foc_motor, MOTOR_SL_RUN);                           /*跳转到开始启动*/
            printf("START\r\n"); 
            break;

        case MOTOR_OPEN_RUN:
            Motor_Run_Open(foc_motor);
            break;
        case MOTOR_SL_RUN:
            Motor_SL_RUN(foc_motor);
            break;  
        case MOTOR_ENC_RUN:
            /*ABZ型编码器，需要先执行开环，将ABZ电角度对齐*/
            if(foc_motor->enc.align){
                Motor_ABZ_ENC_RUN(foc_motor);
            }else{
                Motor_Run_Open(foc_motor);
            }
            break;
        
        case MOTOR_ABZENC_CALIB:
            if(true==Motor_ABZ_ENC_CALIB(foc_motor)){
                Motor_NextState(foc_motor, MOTOR_ENC_RUN);
            }
            break;
        case MOTOR_HALL_RUN:
            /*霍尔传感器运行*/
            Motor_HALL_RUN(foc_motor);
            break;
        case MOTOR_HALL_LEARN:
            /*霍尔自学习*/
            if(true==Motor_HALL_LEARN(foc_motor)){
                Motor_NextState(foc_motor, MOTOR_SL_RUN);
            }
            break;
        default:
            break;
    }
    
}

/**
 * @brief 电机速度计算
 * 
 * @param foc_motor 
 * @param Ts 该函数的调用周期: 单位s
 */
void FOC_Motor_Speed_Calc(FOC_MOTOR_t *foc_motor, float Ts)
{
    FOC_Sensorless_Speed_Cale(&foc_motor->sensorless, Ts);                   /*无感速度计算*/

    FOC_ABZENC_Speed_Calc_M(&foc_motor->enc, Ts);                            /*编码器M法速度计算*/

    FOC_Hall_Speed_Calc(&foc_motor->hall);                                   /*霍尔传感器计算速度*/
}

/**
 * @brief 速度环控制，返回电流环的设定值
 * 
 * @param foc_motor 
 */
void FOC_Motor_Speed_Ctrl(FOC_MOTOR_t *foc_motor)
{
    /*获取目标速度*/
    foc_motor->pid_speed.SetPoint = foc_motor->target_speed;

    /*获取当前速度*/
    float speed = foc_motor->foc_var.speed.AvrMecSpeed;

    /*速度环的输出值作为电流环的目标值*/
    foc_motor->foc_var.I_qd_ref.q = PID_Position_ctrl(&foc_motor->pid_speed, speed);

    foc_motor->foc_var.I_qd_ref.d = 0.00001f;                       /*不能设置为0, 为了避免pid函数里进入退饱和*/

}


/**
 * @brief 电机堵转后的操作，可自由开发
 * 
 * @param foc_motor 
 */
void FOC_Motor_Lock(FOC_MOTOR_t *foc_motor)
{
    /*清除PID参数*/
    PID_Clear(&foc_motor->pid_cur_iq);
    PID_Clear(&foc_motor->pid_cur_id);
    PID_Clear(&foc_motor->pid_speed);
}

