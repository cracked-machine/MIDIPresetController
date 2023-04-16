#ifndef __MAIN_H__
#define __MAIN_H__

extern "C" 
{
    void EXTI0_1_IRQHandler();
    void EXTI4_15_IRQHandler();
    void TIM21_IRQHandler();
    void LPUART1_IRQHandler();
}

#endif // __MAIN_H__