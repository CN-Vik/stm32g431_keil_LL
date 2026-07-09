#include "cc6920so.h"

#include "main.h"
#include "adc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>



/* DMA方式读取ADC */
#ifdef ADC_DMA

#define ADC_BUF_SIZE                        10

uint32_t uvwAdVal[3];                        /*UVW三相的ad值*/

static uint32_t UdmaAdBuf[ADC_BUF_SIZE];    /*UVW三相的dma缓冲区*/
static uint32_t VdmaAdBuf[ADC_BUF_SIZE];
static uint32_t WdmaAdBuf[ADC_BUF_SIZE];
float Iabc[3] = {0};

void CC6920SO_Init(void)
{
    /*DMA方式*/
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, WdmaAdBuf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc2, VdmaAdBuf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc3, UdmaAdBuf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
}

/*计算平均AD值 ch分别为UVW通道*/
static void CC6920SO_CalcAvr(uint8_t ch){
    uint32_t *adc_buf = NULL;

    uint32_t adc_sum = 0;
    uint32_t i = 0;
    switch(ch){
        case 0:
            adc_buf = UdmaAdBuf;
            break;
        case 1:
            adc_buf = VdmaAdBuf;
            break;
        case 2:
            adc_buf = WdmaAdBuf;
            break;
        default:
            return;
    }

    for(i=0; i<ADC_BUF_SIZE; i++){
        adc_sum += adc_buf[i];
    }

    uvwAdVal[ch] = adc_sum/ADC_BUF_SIZE;
}

/*DMA方式采用 规则通道完成 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
   
    uint8_t ch = 5;
    if(hadc==&hadc1){                   /*W相*/      
        ch = 2;
    }else if(hadc==&hadc2){             /*V相*/
        ch = 1;
    }else if(hadc==&hadc3){             /*U相*/
        ch = 0;
    }
    if(ch<3){
        CC6920SO_CalcAvr(ch);           /*计算平均值*/
        Iabc[ch] = CC6920SO_CalcCur(ch, uvwAdVal[ch]);
    }
}

#else

void CC6920SO_Init(void)
{

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(100);

    /*注入通道方式*/
    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc1)){
        printf("HAL_ADCEx_InjectedStart_IT: NO\r\n");
        while(1);
    }
    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc2)){
        printf("HAL_ADCEx_InjectedStart_IT: NO\r\n");
        while(1);
    }
    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc3)){
        printf("HAL_ADCEx_InjectedStart_IT: NO\r\n");
        while(1);
    }
}

#endif

#if 0
/* 20A电流换算 通道顺序UVW

    VOUT = VCC/ 2 +0.100 × IP(A)
*/
float CC6920SO_CalcCur(uint8_t ch, uint32_t adVal)
{
    if(ch>2)return 0;

    float Uout;
    float Uadc;     
    float I;

    /*ADC采集到的电压*/
    Uadc = (float)adVal/4096.0f *3.3f;

    /*霍尔元件输出的电压*/
    Uout = Uadc *3.0f /2.0f;    

    I = (Uout-2.5f)/0.1f;       /*电流计算  Uout = I*0.1*+0.25 */
    
    return I;                   /* 这里要注意电流的方向，因为后面的克拉克计算电流是按照从坐标轴原点流向外面，流出电机为正方向的 */
}
#endif

#if 0
/* 40A电流换算 通道顺序UVW

    VOUT = VCC/ 2 +0.05 × IP(A)
*/
float CC6920SO_CalcCur(uint8_t ch, uint32_t adVal)
{
    if(ch>2)return 0;

    float Uout;
    float Uadc;     
    float I;

    /*ADC采集到的电压*/
    Uadc = (float)adVal/4096.0f *3.3f;

    /*霍尔元件输出的电压*/
    Uout = Uadc *3.0f /2.0f;    

    I = (Uout-2.5f)/0.05f;       /*电流计算  Uout = I*0.05*+0.25 */
    
    return I;                   /* 这里要注意电流的方向，因为后面的克拉克计算电流是按照从坐标轴原点流向外面，流出电机为正方向的 */
}
#endif

#if 1
/* 50A电流换算 通道顺序UVW

    VOUT = VCC/ 2 +0.04 × IP(A)
*/
float CC6920SO_CalcCur(uint8_t ch, uint32_t adVal)
{
    if(ch>2)return 0;

    float Uout;
    float Uadc;     
    float I;

    /*ADC采集到的电压*/
    Uadc = (float)adVal/4096.0f *3.3f;

    /*霍尔元件输出的电压*/
    Uout = Uadc *3.0f /2.0f;    

    I = (Uout-2.5f)/0.04f;       /*电流计算  Uout = I*0.04*+0.25 */
    
    return I;                   /* 这里要注意电流的方向，因为后面的克拉克计算电流是按照从坐标轴原点流向外面，流出电机为正方向的 */
}
#endif


