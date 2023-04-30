#ifndef __LED_H__
#define __LED_H__

#include <stm32l011xx.h>
#include <stdint.h>

#include <tim.hpp>

inline void enable_led(bool enable, uint32_t sleep = 0)
{
    if (enable)
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    delay(sleep);
}

inline void toggle_led(uint32_t sleep = 0)
{
    if (GPIOA->ODR & GPIO_ODR_OD7_Msk)
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    delay(sleep);
}

#endif // __LED_H__