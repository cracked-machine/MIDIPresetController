
#include "main.hpp"
#include "stm32l011xx.h"

static inline void spin(volatile uint32_t count)
{
    while (count--)
        asm("nop");
}

void setup_led(void)
{
    // enable GPIO RCC
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    // config PA7 = output, pushpull, highspeed, no PU/PD
    GPIOA->MODER |= (GPIO_MODER_MODE7_0);
    GPIOA->MODER &= ~(GPIO_MODER_MODE7_1);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED7_Msk);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);
}

void toggle_led(bool enable, uint32_t sleep=0)
{
    if (enable)
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    spin(sleep);
}

int main(void)
{
    setup_led();


    while (1)
    {
        // blinky!
        toggle_led(true, 99999);
        toggle_led(false, 99999);
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