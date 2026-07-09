#ifndef _CC6920SO_H
#define _CC6920SO_H


#include <stdint.h>

/*电流传感芯片*/
// #define ADC_DMA

void CC6920SO_Init(void);
float CC6920SO_CalcCur(uint8_t ch, uint32_t adVal);



#endif
