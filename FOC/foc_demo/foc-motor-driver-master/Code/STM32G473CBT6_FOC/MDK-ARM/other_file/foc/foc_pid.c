/**
 ****************************************************************************************************
 * @file        foc_pid.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       PID代码
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_pid.h"
#include "tim.h"
#include <math.h>
#include <foc_math.h>

#include <stdio.h>

/**
 * @brief PID初始化
 * 
 * @param foc_pid 
 * @param pid_freq PID执行频率
 */
void PID_Init(FOC_PID_t *foc_pid, float pid_freq)
{
    foc_pid->ActualValue    = 0.0f;
    foc_pid->Error          = 0.0f;
    foc_pid->LastError      = 0.0f;
    foc_pid->PrevError      = 0.0f;
    foc_pid->SumError       = 0.0f;
    foc_pid->Ts             = 1.0f/pid_freq;
}

/**
 * @brief 增量式PID控制
 * 
 * @param PID 
 * @param real_val 输入值
 * @return float PID输出
 */
float PID_Increment_ctrl(FOC_PID_t *PID, float real_val)
{
    float P = PID->P;
    float I = PID->I;
    float D = PID->D;

    if (isnan(real_val) || isinf(real_val) ||
        isnan(PID->P) || isinf(PID->P) ||
        isnan(PID->I) || isinf(PID->I) ||
        isnan(PID->D) || isinf(PID->D) ||
        isnan(PID->Ts) || isinf(PID->Ts)) {
        // 如果输入值或PID参数无效，返回0或某个安全值
        return 0.0;
    }

    PID->SetPoint = LIMIT_RANGE(PID->SetPoint, -PID->SetPoint_limit, PID->SetPoint_limit);

    PID->In = real_val;
    PID->Error = PID->SetPoint - real_val;                                                  /*获取偏差值*/

    PID->Up = P *(PID->Error - PID->LastError);                                             /*比例环节*/
    PID->Ui = I *(PID->Error)* PID->Ts;                                                     /*积分环节*/
    PID->Ud = D *(PID->Error -2*PID->LastError +PID->PrevError)* PID->Ts;                   /*微分环节*/

    float increment = PID->Up + PID->Ui + PID->Ud;
    increment = LIMIT_RANGE(increment, -PID->change_limit, PID->change_limit);              /*积分限制*/
    
    PID->ActualValue += increment;
    PID->PrevError = PID->LastError;
    PID->LastError = PID->Error;

    PID->ActualValue = LIMIT_RANGE(PID->ActualValue,  -PID->ActualValue_limit, PID->ActualValue_limit);/*输出限制*/

    if(PID->ActualValue > PID->OutMax){
        PID->OutMax = PID->ActualValue;
    }
    if(PID->ActualValue < PID->OutMin){
        PID->OutMin = PID->ActualValue;
    }

    return PID->ActualValue;
}


/**
 * @brief 位置式PID
 * 
 * @param PID 
 * @param real_val 输入值
 * @return float PID输出
 */
float PID_Position_ctrl(FOC_PID_t *PID, float real_val)
{
    if(PID->I==0){      /*清除积分*/
        PID->Ui = 0;
        PID->SumError = 0;
    }

    if (isnan(real_val) || isinf(real_val) ||
        isnan(PID->P) || isinf(PID->P) ||
        isnan(PID->I) || isinf(PID->I) ||
        isnan(PID->D) || isinf(PID->D) ||
        isnan(PID->Ts) || isinf(PID->Ts)) {
        // 如果输入值或PID参数无效，返回0或某个安全值
        // printf("isnan(PID->P) || isinf(PID->P) ||\
        // isnan(PID->I) || isinf(PID->I) ||\
        // isnan(PID->D) || isinf(PID->D) ||\
        // isnan(PID->Ts) || isinf(PID->Ts))\r\n");
        return 0.0;
    }

    if(PID->flag){
        PID->SumError = 0;
        PID->flag = 0;
    }

    PID->SetPoint = LIMIT_RANGE(PID->SetPoint, -PID->SetPoint_limit, PID->SetPoint_limit);

    PID->In = real_val;
    PID->Error = PID->SetPoint - real_val;
    PID->Up = PID->P * PID->Error;
    PID->Ui = PID->I * PID->SumError * PID->Ts;

    PID->ActualValue = PID->Up + PID->Ui;
    PID->ActualValue = LIMIT_RANGE(PID->ActualValue, -PID->ActualValue_limit, PID->ActualValue_limit);           /*输出限制*/

    


    if(PID->Error==0||PID->SetPoint==0){                  /*积分退饱和处理*/
        PID->SumError *= 0.99f;         /*清除累计误差*/
    }else{
        if( fabs(PID->ActualValue)< PID->ActualValue_limit){
            PID->SumError += PID->Error;
        }
    }
    PID->SumError = LIMIT_RANGE(PID->SumError, -PID->SumError_limit, PID->SumError_limit);                    /*误差饱和*/
         

    // if(PID->SetPoint>=0){               /*防止停机时过充反转*/
    //     if(PID->ActualValue<0)PID->ActualValue=0;
    // }else{
    //     if(PID->ActualValue>0)PID->ActualValue=0;
    // }

    if(PID->ActualValue > PID->OutMax){
        PID->OutMax = PID->ActualValue;
    }
    if(PID->ActualValue < PID->OutMin){
        PID->OutMin = PID->ActualValue;
    }

    return PID->ActualValue;
}

/**
 * @brief PID计算参数清除
 * 
 * @param PID 
 */
void PID_Clear(FOC_PID_t *PID)
{
    PID->SumError = 0.0f;
    PID->Up = 0.0f;
    PID->Ui = 0.0f;
    PID->Ud = 0.0f;
    PID->Error = 0.0f;
    PID->LastError = 0.0f;
    PID->PrevError = 0.0f;
    PID->IngMin = 0.0f;
    PID->IngMax = 0.0f;
    PID->OutMin = 0.0f;
    PID->OutMax = 0.0f;
    PID->ActualValue = 0.0f;
}



