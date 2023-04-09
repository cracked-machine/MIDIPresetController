
#include "main.hpp"
#include "stm32l011xx.h"



static inline void spin(volatile uint32_t count)
{
    while (count--)
        asm("nop");
}

void setup_led(void)
{

    // config PA7 as output (0x01)
    GPIOA->MODER |= (GPIO_MODER_MODE7_0);
    GPIOA->MODER &= ~(GPIO_MODER_MODE7_1);
    // setup p7 as pushpull, highspeed, no PU/PD
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED7_Msk);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);
}

void setup_exti()
{
    // mux external interrupts line for pa10 (syscfg_exticr3) to the internal edge detector 
    // SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PA);

    // PA10
    // setup pa10 gpio as input (0x00)
    GPIOA->MODER &= ~((GPIO_MODER_MODE10_0) | (GPIO_MODER_MODE10_1));
    // set pa10 gpio pullup (0x01)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD10_0);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10_1);

    // enable interupt request in interrupt mask register
    EXTI->IMR |= EXTI_IMR_IM10_Msk;
    // program trigger resistors with edge detection
    EXTI->FTSR |= EXTI_FTSR_FT10_Msk;

    // PA9 
    // setup pa10 gpio as input (0x00)
    GPIOA->MODER &= ~((GPIO_MODER_MODE9_0) | (GPIO_MODER_MODE9_1));
    // set pa10 gpio pullup (0x01)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD9_0);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD9_1);

    // enable interupt request in interrupt mask register
    EXTI->IMR |= EXTI_IMR_IM9_Msk;
    // program trigger resistors with edge detection
    EXTI->FTSR |= EXTI_FTSR_FT9_Msk;

    // ENABLE EXTI9/EXTI10
    // configure nvic irq channel and prio
    NVIC_EnableIRQ(EXTI4_15_IRQn); 
    NVIC_SetPriority(EXTI4_15_IRQn,0); 
}



void enable_led(bool enable, uint32_t sleep=0)
{
    if (enable)
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    spin(sleep);
}

void toggle_led(uint32_t sleep=0)
{
    if (GPIOA->ODR & GPIO_ODR_OD7_Msk)
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
}

int main(void)
{
    // enable GPIO RCC
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    setup_led();
    setup_exti();

    while (1)
    {
        // blinky!
        // toggle_led(true, 99999);
        // toggle_led(false, 99999);
    }
}

void EXTI4_15_IRQHandler()
{
    // GPIOA->BSRR |= GPIO_BSRR_BS_7;
    toggle_led();
    // clear the pending bit for the interrupt line
    if (EXTI->PR == EXTI_IMR_IM10_Msk)
        EXTI->PR |= (EXTI_PR_PIF10_Msk);
    if (EXTI->PR == EXTI_IMR_IM9_Msk)
        EXTI->PR |= (EXTI_PR_PIF9_Msk);
}