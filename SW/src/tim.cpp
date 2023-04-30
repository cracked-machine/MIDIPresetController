#include <tim.hpp>
#include <led.hpp>

extern "C" 
{

void TIM21_IRQHandler()
{
    // do stuff
    toggle_led();
    // clear the interrupt
    TIM21->SR &= ~(TIM_SR_UIF_Msk);
}

}   // extern "C" 