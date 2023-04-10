
#include "main.hpp"
#include "stm32l011xx.h"


static inline void spin(volatile uint32_t count)
{
    while (count--)
        asm("nop");
}

void setup()
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;                                  // enable GPIO clock (used by multiple porta pins)
    
    // LED //
    /////////
    GPIOA->MODER |= (GPIO_MODER_MODE7_0);                               // config PA7 as output (0x01)
    GPIOA->MODER &= ~(GPIO_MODER_MODE7_1);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_7);                               // setup p7 as pushpull, highspeed, no PU/PD
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED7_Msk);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);    

    // EXTI //
    //////////
    // SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PA);                 // mux external interrupts line for pa10 (syscfg_exticr3) 
                                                                        // to the internal edge detector 
    // PA10
    GPIOA->MODER &= ~((GPIO_MODER_MODE10_0) | (GPIO_MODER_MODE10_1));   // setup pa10 gpio as input (0x00)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD10_0);                              // set pa10 gpio pullup (0x01)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10_1);
    EXTI->IMR |= EXTI_IMR_IM10_Msk;                                     // enable interupt request in interrupt mask register
    EXTI->FTSR |= EXTI_FTSR_FT10_Msk;                                   // program trigger resistors with edge detection

    // PA9 
    GPIOA->MODER &= ~((GPIO_MODER_MODE9_0) | (GPIO_MODER_MODE9_1));     // setup pa10 gpio as input (0x00)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD9_0);                               // set pa10 gpio pullup (0x01)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD9_1);
    EXTI->IMR |= EXTI_IMR_IM9_Msk;                                      // enable interupt request in interrupt mask register
    EXTI->FTSR |= EXTI_FTSR_FT9_Msk;                                    // program trigger resistors with edge detection
    
    NVIC_EnableIRQ(EXTI4_15_IRQn);                                      // configure nvic irq channel and prio
    NVIC_SetPriority(EXTI4_15_IRQn,0); 


    // TIMER21 //
    /////////////
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN_Msk;                            // enable the APB2 clock used by TIM21
    TIM21->CR1 |= (TIM_CR1_ARPE_Msk);                                   // preload buffer
    TIM21->DIER |= (TIM_DIER_UIE_Msk);                                  // enable interrupts
    TIM21->PSC = 32;                                                    // prescaler
    TIM21->ARR = 0xFFFFFFFF;                                            // auto-reload value
    NVIC_EnableIRQ(TIM21_IRQn); 
    NVIC_SetPriority(TIM21_IRQn,0); 
    TIM21->CR1 |= (TIM_CR1_CEN_Msk);                                    // start the timer

}

void enable_led(bool enable, uint32_t sleep = 0)
{
    if (enable)
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    spin(sleep);
}

void toggle_led(uint32_t sleep = 0)
{
    if (GPIOA->ODR & GPIO_ODR_OD7_Msk)
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    spin(sleep);
}

int main()
{

    setup();

    while (1)
    {
        // blinky!
        // toggle_led(true, 99999);
        // toggle_led(false, 99999);
    }
}

void EXTI4_15_IRQHandler()
{
    // do stuff
    toggle_led();
    // clear the pending bit for the interrupt line
    if (EXTI->PR == EXTI_IMR_IM10_Msk)
        EXTI->PR |= (EXTI_PR_PIF10_Msk);
    if (EXTI->PR == EXTI_IMR_IM9_Msk)
        EXTI->PR |= (EXTI_PR_PIF9_Msk);
}

void TIM21_IRQHandler()
{
    // do stuff
    toggle_led();

    // clear the interrupt
    TIM21->SR &= ~(TIM_SR_UIF_Msk);
}