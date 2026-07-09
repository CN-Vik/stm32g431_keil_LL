/**
 ****************************************************************************************************
 * @file        foc_app.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       FOC对外提供调用的接口
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */


#include "foc_app.h"
#include "stdio.h"
#include <bsp.h>
#include <stdint.h>

#include <foc_hw.h>

#include <foc_vbus.h>

#if 1

/*ProDrone 3505电机 kv650*/

#define L       (42.17819f/1000000.0f)      /* 5.74uH */
#define R       (98.91666f/1000.0f)        /* 78.12 mΩ */
#define FLUX    (0.91065f /1000.0f)       /* 磁通: 0.680 mWb */
#define Tc      L/R/10.0f

/*平稳的PID参数*/
// #define Kp_q  L/Tc/1000/5*2         
// #define Ki_q  R/Tc/1000000/5*2*3
// #define Kp_d       0.0533333309f
// #define Ki_d       0.0519480482f
// #define Kp_s    0.9375f         
// #define Ki_s    0.000464062527f


/*激进的PID参数*/
#define Kp_q  L*1000.0f/5
#define Ki_q  R/5
#define Kp_d       Kp_q*5.0f
#define Ki_d       Ki_q*10.0f
#define Kp_s    0.400000006f*1.5f/4.0f   
#define Ki_s    0.928125083f*3.0f


#define PID_QD_OUTLIMIT         15.0f              /*qd轴电流环 输出值限制*/
#define PID_QD_OUTLIMIT_HFI     1.5f               /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/

#define IQD_OVER                30.0f               /*qd轴过流值*/

FOC_MOTOR_t FOC_MOTOR = {
    .target_speed = 1000,                          /*目标速度*/
    .pole_pairs = 10,                                /*极对数*/
    .power = 24.0f,                                 /*供电电压*/

    /*电流输出限制*/
    .pid_qd_limit = PID_QD_OUTLIMIT,
    .pid_qd_limit_hif = PID_QD_OUTLIMIT_HFI,

    /*Q轴电流环 PID*/
    .pid_cur_iq ={
        .P = Kp_q,
        .I = Ki_q,
        .D = 0.0f,
    
        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,                   
        .ActualValue_limit = 20.0f,    
    },

    /*D轴电流环 PID*/
    .pid_cur_id ={
        .P = Kp_d,
        .I = Ki_d,
        .D = 0.0f,

        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,
        .ActualValue_limit = 20.0f,      
    },

    /*速度环 PID*/
    .pid_speed ={
        .P = Kp_s,        
        .I = Ki_s,
        
        .D = 0.000f,

        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 50.0f,
        .change_limit = 15.0f,    
    },
    .pid_speed_pos ={
        .P = 0.001f,     /*位置式速度PID参数*/
        .I = 0.007f,
        .D = 0.0000f,

        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 15.0f,
        .change_limit = 0.5f,    
    },

    /*电流设置*/
    .current = {
        .over = 20.0f,
    },

    .enc = {
        .align_dir = -1,
        .speed.PhaseShift = 5.33825159f,
    },

    /*无感设置*/
    .sensorless ={
        .hfi_freq = FOC_HFI_FREQ_10K,               /*高频注入频率*/
        .hfi_vol_ref = 0.5f,                        /*高频注入电压*/     
        .d_bias_vol = 0.2f,                         /*d轴偏置电压*/

        .vf.k = 0.0518748574f/2.0f,                 /*vf强拖系数*/

        .up_smo = 2000.0f,                          /*切入滑模的速度*/
        .smo_down = 1800.0f,                        /*切出滑模的速度*/

        .smo = {                                    /*滑模系数*/
            .k = 0.5f,
            .pll = {
                .Kp = 2700.0f,
                .Ki = 364975.0f *2,
            },  
            .speed.PhaseShift = 0.3f,
        },
        .hfi ={                                     /*高频注入系数*/
            .pll = {
                .Kp = 135.0f,
                .Ki = 9.23320198f,
            },
        },
        .vf.Uqd.q=0.2f,

        .flux = {
            .flux= FLUX,
            .G = 20000000,
            .pll = {
                .Kp = 1000000.0f,
                .Ki = 10000.0f,
            },
        },
        .speed.PhaseShift = -0.1f,
        
    },
};

#endif



#if 0

/*SPEED 2207 kv1700*/

/*驱动器辨识得到的参数
相电阻: 78.12mΩ
相电感: 5.74uH
Lq-Ld 0.67 uH
磁链 0.680mWb
KP:0.0057  KI 78.12
观察器增益2046.66M
*/

#define L       (5.74f/1000000.0f)      /* 5.74uH */
#define R       (78.12f/1000.0f)        /* 78.12 mΩ */
#define FLUX    (0.680f /1000.0f)       /* 磁通: 0.680 mWb */
#define Tc      L/R/10.0f

/*平稳的PID参数*/
// #define Kp_q  L/Tc/1000/5*2         
// #define Ki_q  R/Tc/1000000/5*2*3
// #define Kp_d       0.0533333309f
// #define Ki_d       0.0519480482f
// #define Kp_s    0.9375f         
// #define Ki_s    0.000464062527f


/*激进的PID参数*/
#define Kp_q  L*1000.0f
#define Ki_q  R
#define Kp_d       Kp_q*5.0f
#define Ki_d       Ki_q*10.0f
#define Kp_s    0.400000006f*1.5f/4.0f   
#define Ki_s    0.928125083f*3.0f


#define PID_QD_OUTLIMIT         20.0f              /*qd轴电流环 输出值限制*/
#define PID_QD_OUTLIMIT_HFI     1.5f               /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/

#define IQD_OVER                35.0f               /*qd轴过流值*/

FOC_MOTOR_t FOC_MOTOR = {
    .target_speed = 1000,                          /*目标速度*/
    .pole_pairs = 7,                                /*极对数*/
    .power = 24.0f,                                 /*供电电压*/

    /*电流输出限制*/
    .pid_qd_limit = PID_QD_OUTLIMIT,
    .pid_qd_limit_hif = PID_QD_OUTLIMIT_HFI,

    /*Q轴电流环 PID*/
    .pid_cur_iq ={
        .P = Kp_q,
        .I = Ki_q,
        .D = 0.0f,
    
        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,                   
        .ActualValue_limit = 20.0f,    
    },

    /*D轴电流环 PID*/
    .pid_cur_id ={
        .P = Kp_d,
        .I = Ki_d,
        .D = 0.0f,

        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,
        .ActualValue_limit = 20.0f,      
    },

    /*速度环 PID*/
    .pid_speed ={
        .P = Kp_s,        
        .I = Ki_s,
        
        .D = 0.000f,

        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 50.0f,
        .change_limit = 15.0f,    
    },
    .pid_speed_pos ={
        .P = 0.001f,     /*位置式速度PID参数*/
        .I = 0.007f,
        .D = 0.0000f,

        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 15.0f,
        .change_limit = 0.5f,    
    },

    /*电流设置*/
    .current = {
        .over = 20.0f,
    },

    .enc = {
        .speed.PhaseShift = -1.39172554f,
    },

    /*无感设置*/
    .sensorless ={
        .hfi_freq = FOC_HFI_FREQ_10K,               /*高频注入频率*/
        .hfi_vol_ref = 0.5f,                        /*高频注入电压*/     
        .d_bias_vol = 0.2f,                         /*d轴偏置电压*/

        .vf.k = 0.0518748574f/2.0f,                 /*vf强拖系数*/

        .up_smo = 2000.0f,                          /*切入滑模的速度*/
        .smo_down = 1800.0f,                        /*切出滑模的速度*/

        .smo = {                                    /*滑模系数*/
            .k = 0.5f,
            .pll = {
                .Kp = 2700.0f,
                .Ki = 364975.0f *2,
            },  
            .speed.PhaseShift = 0.3f,
        },
        .hfi ={                                     /*高频注入系数*/
            .pll = {
                .Kp = 135.0f,
                .Ki = 9.23320198f,
            },
        },
        .vf.Uqd.q=0.2f,

        .flux = {
            .flux= FLUX,
            .G = 20000000,
            .pll = {
                .Kp = 1000000.0f,
                .Ki = 10000.0f,
            },
        },
        .speed.PhaseShift = -0.1f,
        
    },
};

#endif



#if 0
/*酷飞2207 kv1960*/

/*
    驱动器辨识得到的参数
    相电阻: 70.24mΩ
    相电感: 4.87uH
    Lq-Ld 0.42 uH
    磁链 0.613mWb
    KP:0.0049  KI 70.24
    观察器增益2661.21M
*/

#define L   (4.87f/1000000.0f)      /* 4.87f uH */
#define R   (70.24f/1000.0f)        /* 70.24f mΩ */
#define FLUX  (0.613f /1000.0f)     /* 磁通: 0.613f mWb */
#define Tc  L/R/10.0f


/*平稳的PID参数*/
// #define Kp_q  L/Tc/1000/5*2         
// #define Ki_q  R/Tc/1000000/5*2*3

// #define Kp_d       0.0533333309f
// #define Ki_d       0.0519480482f

// #define Kp_s    0.9375f         
// #define Ki_s    0.000464062527f


/*激进的PID参数*/
#define Kp_q  L*1000.0f
#define Ki_q  R

#define Kp_d        Kp_q*5
#define Ki_d        Ki_q*10
#define Kp_s        0.400000006f*1.5f/4.0f   
#define Ki_s        0.928125083f*3.0f


#define PID_QD_OUTLIMIT         20.0f              /*qd轴电流环 输出值限制*/
#define PID_QD_OUTLIMIT_HFI     1.5f               /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/

#define IQD_OVER                35.0f               /*qd轴过流值*/

FOC_MOTOR_t FOC_MOTOR = {
    .target_speed = 5000,                          /*目标速度*/
    .pole_pairs = 7,                                /*极对数*/
    .power = 24.0f,                                 /*供电电压*/

    /*电流输出限制*/
    .pid_qd_limit = PID_QD_OUTLIMIT,
    .pid_qd_limit_hif = PID_QD_OUTLIMIT_HFI,

    /*Q轴电流环 PID*/
    .pid_cur_iq ={
        .P = Kp_q,
        .I = Ki_q,
        .D = 0.0f,
      
        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,                   
        .ActualValue_limit = 20.0f,    
    },

    /*D轴电流环 PID*/
    .pid_cur_id ={
        .P = Kp_d,
        .I = Ki_d,
        .D = 0.0f,

        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,
        .ActualValue_limit = 20.0f,      
    },

    /*速度环 PID*/
    .pid_speed ={
        .P = Kp_s,        
        .I = Ki_s,
        
        .D = 0.000f,
  
        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 50.0f,
        .change_limit = 15.0f,    
    },
    .pid_speed_pos ={
        .P = 0.001f,     /*位置式速度PID参数*/
        .I = 0.007f,
        .D = 0.0000f,
  
        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 15.0f,
        .change_limit = 0.5f,    
    },

    /*电流设置*/
    .current = {
        .over = 20.0f,
    },

    .enc = {
        .PhaseShift = -3.802737f +0.0621f,
    },

    /*无感设置*/
    .sensorless ={
        .hfi_freq = FOC_HFI_FREQ_10K,               /*高频注入频率*/
        .hfi_vol_ref = 0.5f,                        /*高频注入电压*/     
        .d_bias_vol = 0.2f,                         /*d轴偏置电压*/

        .vf.k = 0.0518748574f,                      /*vf强拖系数*/

        .up_smo = 2000.0f,                          /*切入滑模的速度*/
        .smo_down = 1800.0f,                        /*切出滑模的速度*/
  
        .smo = {                                    /*滑模系数*/
            .k = 0.5f,
            .pll = {
                .Kp = 2700.0f,
                .Ki = 364975.0f *2,
            },  
            .speed.PhaseShift = 0.3f,
        },
        .hfi ={                                     /*高频注入系数*/
            .pll = {
                .Kp = 135.0f,
                .Ki = 9.23320198f,
            },
        },
        .vf.Uqd.q=0.2f,

        .flux = {
            .flux= FLUX,
            .G = 20000000,
            
            .pll = {
                .Kp = 1000000.0f,
                .Ki = 10000.0f,
            },
            .speed.PhaseShift = -0.1f,
        },
        
    },
};

#endif





#if 0

/*原子PMSM电机*/

/*
驱动器辨识得到的参数
相电阻: 604.51mΩ
相电感: 226.34uH
Lq-Ld 7.34 uH
磁链 9.905 mWb
*/


#define L   (0.22634f/1000.0f)          /* 226.34uH */
#define R   (604.51f/1000.0f)           /* 604.51Ω */
#define FLUX    (9.905f /1000.0f)       /* 磁通: 9.905 mWb */
#define Tc  L/R/10.0f

#define Kp_q    L/Tc/1000
#define Ki_q    R/Tc/10000
#define Kp_d    0.0533333309f*100
#define Ki_d    0.0519480482f*10
#define Kp_s    0.9375f 
#define Ki_s    0.000464062527f

#define PID_QD_OUTLIMIT         20.0f              /*qd轴电流环 输出值限制*/
#define PID_QD_OUTLIMIT_HFI     8.0f               /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/
#define IQD_OVER                35.0f               /*qd轴过流值*/

FOC_MOTOR_t FOC_MOTOR = {
    .target_speed = 2000,                          /*目标速度*/
    .pole_pairs = 4,                                /*极对数*/
    .power = 24.0f,                                 /*供电电压*/

    /*电流输出限制*/
    .pid_qd_limit = PID_QD_OUTLIMIT,
    .pid_qd_limit_hif = PID_QD_OUTLIMIT_HFI,

    /*Q轴电流环 PID*/
    .pid_cur_iq ={
        .P = Kp_q,
        .I = Ki_q,
        .D = 0.0f,
      
        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,         /*根据电流传感器量程来选择*/
        .ActualValue_limit = 8.0f,    /*对应 3000r/min*/ 
    },

    /*D轴电流环 PID*/
    .pid_cur_id ={
        .P = Kp_d,
        .I = Ki_d,
        .D = 0.0f,

        .SumError_limit = 50000000000.0f,
        .SetPoint_limit = 100.0f,
        .ActualValue_limit = 8.0f,      
    },

    /*速度环 PID*/
    .pid_speed ={
        .P = Kp_s,        
        .I = Ki_s,
        
        .D = 0.000f,
  
        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 50.0f,
        .change_limit = 15.0f,    
    },
    .pid_speed_pos ={
        .P = 0.001f,     /*位置式速度PID参数*/
        .I = 0.007f,
        .D = 0.0000f,
  
        .SetPoint = 0.0f,

        .SumError_limit = 10000000.0f,
        .SetPoint_limit = 31000.0f,      /*最大转速范围*/
        .ActualValue_limit = 15.0f,
        .change_limit = 0.5f,    
    },

    /*电流设置*/
    .current = {
        .over = 20.0f,
    },

    .enc = {
        .PhaseShift = -0.666017532f,
    },

    /*无感设置*/
    .sensorless ={
        .hfi_freq = FOC_HFI_FREQ_5K,                /*高频注入频率  这个电机电感大，要降低注入频率*/
        .hfi_vol_ref = 2.5f,                        /*高频注入电压*/     
        .d_bias_vol = 0.3f,                         /*d轴偏置电压*/

        

        .up_smo = 500.0f,                           /*切入滑模的速度*/
        .smo_down = 400.0f,                         /*切出滑模的速度*/
        
        .vf = {
            .k = 0.00200000009f,                     /*vf强拖系数*/
            .Uqd.d = 3.0f,
        },
        
        .smo = {                                    /*滑模系数*/
            .k = 0.3f,
            .pll = {
                .Kp = 270.0f,
                .Ki = 36497.0f,
            },
            .speed.PhaseShift = 0.5f,
        },
        .hfi ={                                     /*高频注入系数*/
            .pll = {
                .Kp = 13.0f /2,
                .Ki = 9.23320198f /2,
            },
        },
        .flux = {
            .flux= FLUX,
            .G = 20000000,
            .pll = {
                .Kp = 1000000.0f,
                .Ki = 10000.0f,
            },
        },
        .speed.PhaseShift = -0.1f,

    },
    .hall.PhaseShift = 0.8f,                        /*霍尔偏移*/
};

#endif


/**
 * @brief FOC初始化
 * 
 */
void FOC_Init(void)
{
    /*定时器周期值*/
    uint32_t htim1_period = 2*(htim1.Init.Period+1);                     /*因为定时器1是中央对齐的方式, 所以实际上的周期是两倍Period*/

    uint32_t htim3_period = htim3.Init.Period+1;
    uint32_t htim6_period = htim6.Init.Period+1;
    uint32_t htim5_period = htim5.Init.Period+1;
    
    /*定时器频率*/
    uint32_t htim1_freq = 170000000/(htim1.Init.Prescaler+1)/htim1_period;   

    uint32_t htim5_freq = 170000000/(htim5.Init.Prescaler+1)/htim5_period;  
    uint32_t htim6_freq = 170000000/(htim6.Init.Prescaler+1)/htim6_period;

    printf("htim1_period: %d  htim1_freq: %d\r\n", htim1_period, htim1_freq);
    printf("htim6_period: %d  htim6_freq: %d\r\n", htim6_period, htim6_freq);

    // FOC_Identify_Init(&FOC_MOTOR.iden, htim1_period/2, FOC_MOTOR.power, 1.0f/htim1_freq);

    PID_Init(&FOC_MOTOR.pid_cur_iq,     htim1_freq);
    PID_Init(&FOC_MOTOR.pid_cur_id,     htim1_freq);
    PID_Init(&FOC_MOTOR.pid_speed,      htim6_freq);
    PID_Init(&FOC_MOTOR.pid_speed_pos,  htim6_freq);

    FOC_Motor_Init(&FOC_MOTOR);

    // FOC_VBUS_Init(&FOC_MOTOR.vbus);

    FOC_CURRENT_Init(&FOC_MOTOR.current, IQD_OVER);

    FOC_ABZENC_Init(&FOC_MOTOR.enc, htim3_period, FOC_MOTOR.pole_pairs);

    FOC_HALL_Init(&FOC_MOTOR.hall,  htim5_freq*htim5_period, htim1_freq, FOC_MOTOR.pole_pairs);

    FOC_Sensorless_Init(&FOC_MOTOR.sensorless, R, L, htim1_freq, FOC_MOTOR.pole_pairs, IQD_OVER);

    FOC_PWM_Init(&FOC_MOTOR.pwm, htim1_period, FOC_MOTOR.power);

    HAL_Delay(1000);        /*让电容冲一下电*/
}

void FOC_DeInit(void)
{

}


extern void Foc_Current_Refresh(FOC_MOTOR_t *foc_motor);

/**
 * @brief FOC的运行状态机，PWM定时器->ADC采样->执行此函数
 * 
 */
void FOC_App_Run(void)
{
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);              /*关闭捕获中断*/

    /*母线电压检测*/
    // FOC_VBUS_Protect(&FOC_MOTOR.vbus);

    /*更新电流*/
    FOC_CURRENT_Update(&FOC_MOTOR.current);
    
    /*电机状态机运行*/
    FOC_Motor_Run(&FOC_MOTOR);

    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);             /*打开捕获中断*/
}








