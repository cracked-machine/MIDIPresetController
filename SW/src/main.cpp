
#include "main.hpp"
#include "stm32l011xx.h"

static inline void spin(volatile uint32_t count)
{
    while (count--)
        asm("nop");
}

int main(void)
{
    // enable GPIO RCC
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // config PA7 = output, pushpull, highspeed, no PU/PD
    GPIOA->MODER |= (GPIO_MODER_MODE7_0);
    GPIOA->MODER &= ~(GPIO_MODER_MODE7_1);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED7_Msk);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);

    while (1)
    {
        // blinky!
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
        spin(99999);
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
        spin(99999);
    }
}

void NMI_Handler(void)
{
    while (1)
    {
        // trouble
    }
}
void HardFault_Handler(void)
{
    while (1)
    {
        // trouble
    }
}
void SVC_Handler(void)
{
    while (1)
    {
        // trouble
    }
}
void PendSV_Handler(void)
{
    while (1)
    {
        // trouble
    }
}
void SysTick_Handler(void)
{
}