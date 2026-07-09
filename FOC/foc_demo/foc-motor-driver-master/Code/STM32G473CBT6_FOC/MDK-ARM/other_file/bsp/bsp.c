#include "bsp.h"
#include <string.h>
#include <stdbool.h>
#include <tim.h>
#include "foc_app.h"

void BSP_Init(void)
{

    delay_init(170);
    uart_init(115200);

    delay_ms(100);

    MX_USB_Device_Init();

    printf("BSP_Init: OK\r\n");


    FOC_Init();
   

    
}








