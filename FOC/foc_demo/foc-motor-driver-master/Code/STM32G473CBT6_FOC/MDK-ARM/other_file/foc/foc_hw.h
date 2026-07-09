
#ifndef __FOC_HW_H
#define __FOC_HW_H

#include <stdbool.h>
#include <stdint.h>

#include <tim.h>

/*PWM接口*/
#define U_H_SET_PWM(x)       (htim1.Instance->CCR3 =x)
#define V_H_SET_PWM(x)       (htim1.Instance->CCR2 =x)
#define W_H_SET_PWM(x)       (htim1.Instance->CCR1 =x)
#define U_L_SET_PWM(x)       (x)
#define V_L_SET_PWM(x)       (x)
#define W_L_SET_PWM(x)       (x)

/*霍尔接口*/
#define HALL_U_GET             HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)
#define HALL_V_GET             HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)
#define HALL_W_GET             HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) 


#include <foc_current.h>
void FOC_CURRENT_HW_Init(void);
void FOC_CURRENT_HW_DeInit(void);
void FOC_CURRENT_Update(FOC_CURRENT_t *foc_cur);

void FOC_PWM_HW_Init(void);
void FOC_PWM_HW_DeInit(void);
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N);

#include <foc_abzenc.h>
void FOC_ABZENC_HW_Init(FOC_ABZENC_t *enc);
void FOC_ABZENC_HW_DeInit(FOC_ABZENC_t *enc);
void FOC_ABZENC_HW_IRQHandler(FOC_ABZENC_t *enc);
uint32_t FOC_ABZENC_HW_GetCount(FOC_ABZENC_t *enc);
ENC_Dir FOC_ABZENC_HW_Dir(FOC_ABZENC_t *enc);


#include <foc_hall.h>
void FOC_HALL_HW_Init(FOC_HALL_t *foc_hall);
void FOC_HALL_HW_DeInit(FOC_HALL_t *foc_hall);
void FOC_HALL_HW_IRQ_Handler(FOC_HALL_t *foc_hall);


#include "foc_vbus.h"
void FOC_VBUS_HW_Init(FOC_VBUS_t *vbus);
void FOC_VBUS_HW_DeInit(FOC_VBUS_t *vbus);

#endif
