#include "debug.h"

#include "bsp.h"
#include "debug.h"
#include "foc_app.h"
#include "foc_math.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"



/*vofaÊý¾ÝÖ¡*/
VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};


void Debug_Task(void)
{
    
    VOFA_Handle.fdata[0] = FOC_MOTOR.foc_var.I_abc.a;
    VOFA_Handle.fdata[1] = FOC_MOTOR.foc_var.I_abc.b;
    VOFA_Handle.fdata[2] = FOC_MOTOR.foc_var.I_abc.c;

    VOFA_Handle.fdata[3] = FOC_MOTOR.pwm._output.a;
    VOFA_Handle.fdata[4] = FOC_MOTOR.pwm._output.b;
    VOFA_Handle.fdata[5] = FOC_MOTOR.pwm._output.c;


    VOFA_Handle.fdata[6] = FOC_MOTOR.foc_var.I_qd.q;
    VOFA_Handle.fdata[7] = FOC_MOTOR.foc_var.I_qd.d;

    VOFA_Handle.fdata[8] = FOC_MOTOR.sensorless.vf.speed.ElAngle;
    VOFA_Handle.fdata[9] = FOC_MOTOR.sensorless.hfi.speed.ElAngle;
    VOFA_Handle.fdata[10] = FOC_MOTOR.sensorless.smo.speed.ElAngle;
    VOFA_Handle.fdata[11] = FOC_MOTOR.sensorless.flux.speed.ElAngle;
    VOFA_Handle.fdata[12] = FOC_MOTOR.enc.speed.ElAngle;
    VOFA_Handle.fdata[13] = FOC_MOTOR.hall.speed.ElAngle;

    
    VOFA_Handle.fdata[14] = FOC_MOTOR.sensorless.vf.speed.AvrMecSpeed;
    VOFA_Handle.fdata[15] = FOC_MOTOR.sensorless.hfi.speed.AvrMecSpeed;
    VOFA_Handle.fdata[16] = FOC_MOTOR.sensorless.smo.speed.AvrMecSpeed;
    VOFA_Handle.fdata[17] = FOC_MOTOR.sensorless.flux.speed.AvrMecSpeed;
    VOFA_Handle.fdata[18] = FOC_MOTOR.enc.speed.AvrMecSpeed;
    VOFA_Handle.fdata[19] = FOC_MOTOR.hall.speed.AvrMecSpeed;

    cdc_vcp_data_tx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
}


