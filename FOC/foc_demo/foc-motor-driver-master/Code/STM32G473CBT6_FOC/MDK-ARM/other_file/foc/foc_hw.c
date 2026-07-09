/**
 ****************************************************************************************************
 * @file        foc_hw.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       该FOC库所调用的一些硬件接口
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */



#include "foc_hw.h"

#include "foc_motor.h"
#include "foc_math.h"

#include "bsp.h"


/***************************************电流硬件相关**************************************/
#include "adc.h"
/**
 * @brief 电流部分的硬件初始化
 * 
 */
void FOC_CURRENT_HW_Init(void)
{   
    /*通道1不再使用, 该ADC用于母线电压检测*/
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    // HAL_Delay(100);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(100);

    /*注入通道方式*/
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);
}
/**
 * @brief 电流部分的硬件复位
 * 
 */
void FOC_CURRENT_HW_DeInit(void)
{
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop_IT(&hadc2);
    HAL_ADCEx_InjectedStop_IT(&hadc3);
}

/*更新FOC电流*/
void FOC_CURRENT_Update(FOC_CURRENT_t *foc_cur)
{
    foc_cur->mcu_ad.c = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - foc_cur->ad_offset.c;
    foc_cur->mcu_ad.b = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - foc_cur->ad_offset.b;
    foc_cur->mcu_ad.a = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - foc_cur->ad_offset.a;

    /*计算三相电流*/
    foc_cur->I_abc.a = CC6920SO_CalcCur(0, foc_cur->mcu_ad.a);
    foc_cur->I_abc.b = CC6920SO_CalcCur(1, foc_cur->mcu_ad.b);
    foc_cur->I_abc.c = CC6920SO_CalcCur(2, foc_cur->mcu_ad.c);
    // foc_cur->I_abc.c = -(foc_cur->I_abc.a + foc_cur->I_abc.b);
}


/****************************************************************************************/




/***************************************PWM硬件相关**************************************/

/**
 * @brief PWM硬件初始化
 * 
 */
void FOC_PWM_HW_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim1);                          /*开启PWM的定时器周期中断*/
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);             /*开启通道四的触发中断*/

    HAL_TIM_Base_Start_IT(&htim6);                          /*开启定时器周期中断， 用于做一些速度计算之类的*/

    U_H_SET_PWM(0);
    V_H_SET_PWM(0);
    W_H_SET_PWM(0);
}
void FOC_PWM_HW_DeInit(void)
{   
    HAL_TIM_Base_Stop_IT(&htim1);                          
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim6);                          
}

/**
 * @brief PWM硬件通道开关
 * 
 * @param A     A相上管通道
 * @param A_N   A相下管通道
 * @param B     B相上管通道
 * @param B_N   B相下管通道
 * @param C     C相下管通道
 * @param C_N   C相下管通道
 */
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N)
{
    if(A)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    if(A_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    if(B)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

    if(B_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    if(C)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    if(C_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);


}

/****************************************************************************************/







/***************************************ABZ编码器硬件相关**************************************/

/**
 * @brief ABZ编码器硬件初始化
 * 
 * @param enc 
 */
void FOC_ABZENC_HW_Init(FOC_ABZENC_t *enc)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         /*开启定时器编码器模式*/
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);             /*开启定时器捕获, 捕获编码器Z相*/
}

void FOC_ABZENC_HW_DeInit(FOC_ABZENC_t *enc)
{
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);          /*开启定时器编码器模式*/
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);              /*开启定时器捕获, 捕获编码器Z相*/
}

/**
 * @brief ABZ编码器中断中调用
 * 
 * @param enc 
 */
void FOC_ABZENC_HW_IRQHandler(FOC_ABZENC_t *enc)
{
    /*定位Z轴*/
    if(enc->dir==ENC_DOWN){
        __HAL_TIM_SET_COUNTER(&htim3, 0);                       
    }else{
        __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period);       
    }

    /*定时器溢出了*/
    if(enc->align==true){
        enc->over_count += 1;
    }else{
        enc->align = true;
        printf("ABZ编码器已对齐过一次Z轴: OK\r\n");
    }
}

/**
 * @brief 获取ABZ编码器当前的脉冲值
 * 
 * @param enc 
 * @return uint32_t 返回脉冲值
 */
uint32_t FOC_ABZENC_HW_GetCount(FOC_ABZENC_t *enc)
{
    return (uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
}


/**
 * @brief 获取编码器计数方向 0递增 1递减
 * 
 * @param enc 
 * @return ENC_Dir 返回编码器方向
 */
ENC_Dir FOC_ABZENC_HW_Dir(FOC_ABZENC_t *enc)
{
    ENC_Dir dir;
    if(0==READ_BIT(TIM3->CR1, TIM_CR1_DIR)){
        dir = ENC_UP;
    }else{
        dir = ENC_DOWN;
    }
    return dir;
}




/****************************************************************************************/




/***************************************霍尔编码器相关**************************************/

/**
 * @brief 霍尔编码器硬件初始化
 * 
 * @param foc_hall 
 */
void FOC_HALL_HW_Init(FOC_HALL_t *foc_hall)
{
    /* 霍尔编码器 */
    HAL_TIM_Base_Start_IT(&htim5);                          /* 开启周期中断 */
    HAL_TIMEx_HallSensor_Start_IT(&htim5);                  /* 开启霍尔中断 */
}

void FOC_HALL_HW_DeInit(FOC_HALL_t *foc_hall)
{
    /* 霍尔编码器 */
    HAL_TIM_Base_Stop_IT(&htim5);                          
    HAL_TIMEx_HallSensor_Stop_IT(&htim5);   
}


/**
 * @brief 霍尔中断处理
 * 
 * @param foc_hall 
 */
void FOC_HALL_HW_IRQ_Handler(FOC_HALL_t *foc_hall)
{
    /*获取霍尔的三线状态*/
    uint8_t HallState = HALL_U_GET 
                        | HALL_V_GET << 1
                        | HALL_W_GET << 2;

    /*记录当前捕获的定时器数值*/
    uint32_t hHighSpeedCapture = (int64_t)htim5.Instance->CCR1;                


    /*必须在切换扇区前计算上一个扇区的补偿系数*/
    Hall_SectorComp_Caculate(foc_hall, hHighSpeedCapture);

    switch (HallState)
    {
        case 4:
            if(foc_hall->sector_pre==6){
                foc_hall->speed.dir = COROTATION;
            }else if(foc_hall->sector_pre==5){
                foc_hall->speed.dir = REVERSAL;
            }
            foc_hall->sector = 0;
            break;
        case 5:
            if(foc_hall->sector_pre==4){
                foc_hall->speed.dir = COROTATION;
            }else if(foc_hall->sector_pre==1){
                foc_hall->speed.dir = REVERSAL;
            }
            foc_hall->sector = 1;
            break;
        case 1:
            if(foc_hall->sector_pre==5){
                foc_hall->speed.dir = COROTATION;
            }else if(foc_hall->sector_pre==3){
                foc_hall->speed.dir = REVERSAL;
            }
            foc_hall->sector = 2;
            break;
        case 3:
            if(foc_hall->sector_pre==1){
                foc_hall->speed.dir = COROTATION;
            }else if(foc_hall->sector_pre==2){
                foc_hall->speed.dir = REVERSAL;
            }
            foc_hall->sector = 3;
            break;
        case 2:
            if(foc_hall->sector_pre==3){
                foc_hall->speed.dir = COROTATION;
            }else if(foc_hall->sector_pre==6){
                foc_hall->speed.dir = REVERSAL;
            }
            foc_hall->sector = 4;
            break;
        case 6:
            if(foc_hall->sector_pre==2){
                foc_hall->speed.dir = COROTATION;
            }else if(foc_hall->sector_pre==4){
                foc_hall->speed.dir = REVERSAL;
            }
            foc_hall->sector = 5;
            break;
    }
    foc_hall->sector_pre = HallState;

    /*角度累计, 堵转补偿 积分补偿清零*/
    // foc_hall->Lock_ElAngle = 0.0f;
    foc_hall->Sector_ElAngle_Sum = 0.0f;
    foc_hall->Sector_CompIntegral = 0.0f;  

    if(foc_hall->speed.dir==COROTATION){
        foc_hall->Sector_ElAngle = foc_hall->sector_pos[foc_hall->sector];
    }else{
        foc_hall->Sector_ElAngle = foc_hall->sector_pos_c[foc_hall->sector];
    }

    /*存入滑动滤波器缓冲区*/
    Move_Filter_fill(&foc_hall->Period_filter, hHighSpeedCapture);
    /*计算扇区速度*/
    float AvrCount = Move_Filter_calculate(&foc_hall->Period_filter);               /* 平均每个扇区的计数值 */
    float t = AvrCount / foc_hall->hall_freq;                                       /* 一个周期的时间 */
    foc_hall->AvrElSpeed = _PI_3/t;                                                 /* 霍尔计算的电角速度 */
    foc_hall->AvrElSpeedDpp = foc_hall->AvrElSpeed /foc_hall->foc_freq;             /* 一个foc周期内增加的霍尔扇区内电角度*/

   
    /*用于调试霍尔位置偏差的*/
    Hall_Parameter_Calculate(foc_hall, hHighSpeedCapture);
    Hall_Parameter_Debug(foc_hall);


    
}



/****************************************************************************************/










/***************************************VBUS母线电压相关**************************************/

void FOC_VBUS_HW_Init(FOC_VBUS_t *vbus)
{
    // HAL_ADC_Start_DMA(&hadc1, vbus->adc_buf, VBUS_ADC_BUF_SIZE);
}


void FOC_VBUS_HW_DeInit(FOC_VBUS_t *vbus)
{
    // HAL_ADC_Stop_DMA(&hadc1);
}

/****************************************************************************************/

















/***************************************中断相关**************************************/

/*****************ADC************************* */
#include "adc.h"
#include "foc_app.h"
#include "foc_hw.h"

/*adc的分配，adc1用于母线电压检测，adc2和adc3用于电流检测*/

/*注入转换完成中断
整个FOC的周期是通过PWM固定的周期触发电流采样，电流采样结束后触发中断，执行每个FOC周期需要执行的任务
*/
bool adc_flag[3] = {0};
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    #if 1
    if(hadc==&hadc1){
        adc_flag[0] = true;
    }else if(hadc==&hadc2){
        adc_flag[1] = true;
    }else if(hadc==&hadc3){
        adc_flag[2] = true;
    }

    if(adc_flag[0]==true&&adc_flag[1]==true&&adc_flag[2]==true){

        adc_flag[0] = false;
        adc_flag[1] = false;
        adc_flag[2] = false;
        FOC_App_Run();                                      /*FOC周期运行*/
    }
   
    #else
    if(hadc==&hadc2){
        adc_flag[1] = true;
    }else if(hadc==&hadc3){
        adc_flag[2] = true;
    }

    if(adc_flag[1]==true&&adc_flag[2]==true){
        adc_flag[1] = false;
        adc_flag[2] = false;
        FOC_App_Run();                                      /*FOC周期运行*/
    }

     #endif
}

/*ADC普通中断 用于获取母线电压*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc==&hadc1){
        //printf("HAL_ADC_ConvCpltCallback\r\n");
        
        /*计算母线电压*/
        FOC_VBUS_Calc(&FOC_MOTOR.vbus);
    }
}
                                                           

/**********************************定时器***************************************** */
/*定时器捕获中断*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM2){
        /*触发了编码器Z相*/
        FOC_ABZENC_HW_IRQHandler(&FOC_MOTOR.enc);       /*执行编码器对齐函数*/
    }

    if(htim->Instance==TIM5){
        FOC_HALL_HW_IRQ_Handler(&FOC_MOTOR.hall);       /*霍尔捕获中断触发*/
    }
}

/*输出捕获中断*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{   
    /*运行周期为2xpwm周期 20khz*/
     if(htim->Instance==TIM1){

    }
}

/*定时器周期中断*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
    if(htim->Instance == TIM1){ /*20k Hz*/

        return;
    }

    if(htim->Instance == TIM6){     /*2000Hz的频率 0.5ms计算一次*/
        FOC_Motor_Speed_Calc(&FOC_MOTOR, 0.0005f);              /*速度计算*/

        FOC_Motor_Speed_Ctrl(&FOC_MOTOR);                       /*速度环运行*/
    }
}


/****************************************************************************************/

