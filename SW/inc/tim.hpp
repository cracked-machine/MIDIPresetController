#ifndef __TIM_H__
#define __TIM_H__

#include <stdint.h>
#include <stm32l011xx.h>

extern "C"
{
    void TIM21_IRQHandler();
}

inline bool delay(uint32_t delay_us)
{
    // if (delay_us > 0xFFFE) { delay_us = 0xFFFE; }
    while (TIM21->CNT < delay_us);
    return true;
}

#endif // __TIM_H__