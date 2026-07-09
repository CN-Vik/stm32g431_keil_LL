
#ifndef __FOC_APP_H
#define __FOC_APP_H


#include  "foc_motor.h"



extern FOC_MOTOR_t FOC_MOTOR;


extern void FOC_Init(void);

extern void FOC_App_Run(void);

void foc_key(void);

#endif




