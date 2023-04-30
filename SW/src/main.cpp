
#include "main.hpp"

#include "stm32l011xx.h"
#if DEBUG
    #include <SEGGER_RTT.h>
#endif

#include <led.hpp>
#include <rcc.hpp>


#define UART_DIV_LPUART(__PCLK__, __BAUD__)      (((((uint64_t)(__PCLK__)*256U)) + ((__BAUD__)/2U)) / (__BAUD__))


const uint32_t _LPUART_ISR_PRIORITY = 0U;
const uint32_t _TIM21_ISR_PRIORITY = 0U;
const uint32_t _EXTI_ISR_PRIORITY = 0U;

void setup()
{
  
    // HCLK //
    //////////
    RCC->CR |= RCC_CR_HSION_Msk;                                // enable HSI16 clk source
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI;                           // set the PLL source mux to HSI16
    
    RCC->CR &= (uint32_t)(~RCC_CR_PLLON);                       // Disable the PLL    
    while((RCC->CR & RCC_CR_PLLRDY) != 0) { }                   // Wait for PLLRDY to be cleared

    FLASH->ACR |= FLASH_ACR_LATENCY;                            // Set latency to 1 wait state
    RCC->CFGR |= (RCC_CFGR_PLLMUL3 | RCC_CFGR_PLLDIV2);         // Set the PLL to 24MHz (assuming RCC_CFGR_HPRE_DIV1 is set below)

    RCC->CFGR &= ~(RCC_CFGR_HPRE_DIV512);                       // mask write all four bits to zero
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1);                          // 24Mhz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV2);                          // 12MHz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV4);                          // 6Mhz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV8);                          // 3MHz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV16);                         // 1.5MHz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV64);                         // 0.375MHz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV128);                        // 0.1875MHz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV256);                        // 0.09375MHz
    // RCC->CFGR |= (RCC_CFGR_HPRE_DIV512);                        // 0.046875MHz
    
    RCC->CR |= RCC_CR_PLLON;                                    // Enable the PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) { }                  // Wait until PLLRDY is set
    
    RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);                  // Select PLL as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }  // Wait untill PLL is switched on


    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;                                  // enable GPIO bus clock (used by multiple porta pins)
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;                                  // enable GPIO bus clock (used by multiple porta pins)
    
    // LED //
    /////////
    GPIOA->MODER |= (GPIO_MODER_MODE7_0);                               // config PA7 as output (0x01)
    GPIOA->MODER &= ~(GPIO_MODER_MODE7_1);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_7);                               // setup p7 as pushpull, highspeed, no PU/PD
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED7_Msk);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);    

    // EXTI //
    //////////
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk; 
    SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PA);                     // map Port A GPIOs to EXTI9
    SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PA);                    // map Port A GPIOs to EXTI10
                                                                        
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

    // PA4 - EXT_MIDI_CH_BIT1
    GPIOA->MODER &= ~((GPIO_MODER_MODE4_0) | (GPIO_MODER_MODE4_1));     // setup PA4 gpio as input (0x00)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD4_0);                               // set pa4 gpio pullup (0x01)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4_1);
    
    // PC14 - EXT_MIDI_CH_BIT2
    GPIOC->MODER &= ~((GPIO_MODER_MODE14_0) | (GPIO_MODER_MODE14_1));   // setup PC14 gpio as input (0x00)
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPD14_0);                              // set pc14 gpio pullup (0x01)
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD14_1);

    // PC15 - EXT_MIDI_CH_BIT3
    GPIOC->MODER &= ~((GPIO_MODER_MODE15_0) | (GPIO_MODER_MODE15_1));   // setup PC15 gpio as input (0x00)
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPD15_0);                              // set pc15 gpio pullup (0x01)
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD15_1);
    
    // PA0 - EXT_MIDI_CH_BIT4
    GPIOA->MODER &= ~((GPIO_MODER_MODE0_0) | (GPIO_MODER_MODE0_1));     // setup PA0 gpio as input (0x00)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD0_0);                               // set pa0 gpio pullup (0x01)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_1);
    
    NVIC_EnableIRQ(EXTI4_15_IRQn);                                      // configure nvic irq channel and prio
    NVIC_SetPriority(EXTI4_15_IRQn, _EXTI_ISR_PRIORITY); 

    // LPUART //
    ////////////
    GPIOA->MODER |= (GPIO_MODER_MODE1_1);                               // Set PA1 as alt func (0x10) 
    GPIOA->MODER &= ~(GPIO_MODER_MODE1_0);
    volatile uint32_t GPIOA_AF_LPUART = 0x6;
    GPIOA->AFR[0] |= (GPIOA_AF_LPUART << GPIO_AFRL_AFSEL1_Pos);         // Set alt func on pin pa1 to lpuart1
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED1_Msk;                        // "very fast" speed
    
    RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN_Msk;                          // enable the APB1 bus clock used by LPUART
    RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL_1 | RCC_CCIPR_LPUART1SEL_0);   // Reset. APB/PCLK (HSI16 via PLL and AHB PSC) selected as LPUART source
    // RCC->CCIPR |= (RCC_CCIPR_LPUART1SEL_0);                             // SystemClock (HSI16 via PLL) selected as LPUART source
    // RCC->CCIPR |= (RCC_CCIPR_LPUART1SEL_1);                             // direct HSI16 (No Multipliers or PSCs) selected as LPUART source
    
    LPUART1->CR1 &= ~((USART_CR1_M0_Msk) | (USART_CR1_M1_Msk));          // 1 start bit, 8 data bits, n stop bits
    uint32_t pclk = get_clk_freq();                                     // baud rate
    uint32_t baudrate = 31250U;    
    LPUART1->BRR = (uint32_t)UART_DIV_LPUART(pclk, baudrate);                               
    LPUART1->CR2 &= ~(USART_CR2_STOP_Msk);                               // 1 stop bit
    LPUART1->CR1 &= ~(USART_CR1_PCE_Msk);                                // no parity for MIDI
    LPUART1->CR2 &= ~(USART_CR2_MSBFIRST_Msk);                           // LSB first
    // LPUART1->CR1 |=  ( USART_CR1_TXEIE_Msk );                              // Enable TXE (tx data reg transferred) interrupts
                    // | USART_CR1_TCIE_Msk);                                // Enable TC (transmission complete) interrupts
                    // | USART_CR3_EIE_Msk );                              // Enable error interrupts
    LPUART1->CR1 |= (USART_CR1_UE_Msk);                                  // enable the LPUART
    LPUART1->CR1 |= (USART_CR1_TE_Msk);                                  // send idle frame as first transmission
    NVIC_SetPriority(LPUART1_IRQn, _LPUART_ISR_PRIORITY);
    NVIC_EnableIRQ(LPUART1_IRQn);

    // TIMER21 //
    /////////////
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN_Msk;                            // enable the APB2 bus clock used by TIM21
    TIM21->CR1 |= (TIM_CR1_ARPE_Msk);                                   // preload buffer
    TIM21->PSC = 0xFFFF;                                                    // prescaler
    TIM21->ARR = 0xFFFFFFFE;                                            // auto-reload value
    // TIM21->PSC = 0x1;                                                    // prescaler
    // TIM21->ARR = 0xF;       

    // TIM21->DIER |= (TIM_DIER_UIE_Msk);                                  // enable interrupts
    // NVIC_EnableIRQ(TIM21_IRQn); 
    // NVIC_SetPriority(TIM21_IRQn,_TIM21_ISR_PRIORITY);    
    
    TIM21->CR1 |= (TIM_CR1_CEN_Msk);                                    // start the timer

}

int main()
{
    #if DEBUG
        SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    #endif
    setup();
    // SEGGER_RTT_printf(0, "%x\r\n", midi_up_preset_msg);
    // SEGGER_RTT_printf(0, "%x\r\n", midi_down_preset_msg);

    // read_midi_channel();
    // SEGGER_RTT_printf(0, "%x\r\n", midi_up_preset_msg);
    // SEGGER_RTT_printf(0, "%x\r\n", midi_down_preset_msg);
    
    // SEGGER_RTT_printf(0, "%x\r\n", midi_down_preset_msg[0]);
    // SEGGER_RTT_printf(0, "%x\r\n", midi_down_preset_msg[1]);
    // SEGGER_RTT_printf(0, "%x\r\n", midi_down_preset_msg[2]);

    [[maybe_unused]] uint32_t tmp = get_clk_freq();
    tmp = get_clk_freq();

    enable_led(false);

    while (1)
    {
        // blinky!
        // toggle_led(0xFFFF);
    }
}


////////////////////////////////
/// Interupt Service Routines
////////////////////////////////

extern "C" 
{

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

}