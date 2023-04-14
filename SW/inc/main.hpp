#ifndef __MAIN_H__
#define __MAIN_H__

extern "C" 
{

    void EXTI4_15_IRQHandler(void);
    void TIM21_IRQHandler();
    void LPUART1_IRQHandler();
}

#endif // __MAIN_H__