
#include "main.hpp"
#include "stm32l011xx.h"
#include <array>

#define UART_DIV_LPUART(__PCLK__, __BAUD__)      (((((uint64_t)(__PCLK__)*256U)) + ((__BAUD__)/2U)) / (__BAUD__))

// hacky quick clk adjust
#define _HCLK12MHZ (RCC_CFGR_PLLMUL3 | RCC_CFGR_PLLDIV4)
#define _HCLK16MHZ (RCC_CFGR_PLLMUL3 | RCC_CFGR_PLLDIV3)
#define _HCLK24MHZ (RCC_CFGR_PLLMUL3 | RCC_CFGR_PLLDIV2)
#define _HCLK32MHZ (RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLDIV3)

#define _LPUART_ISR_PRIORITY 0U
#define _TIM21_ISR_PRIORITY 0U
#define _EXTI_ISR_PRIORITY 0U

const volatile uint32_t DEBOUNCE_COOLDOWN{ 100 };
volatile uint32_t prev_cnt = 0;


std::array<uint8_t, 4> msg{0xDE, 0xAD, 0xBE, 0xEF};
// uint8_t msg[4] = {0xDE, 0xAD, 0xBE, 0xAF};
uint16_t msg_tx_count = 0;
uint8_t bytes_sent{0};

uint32_t get_clk_freq()
{
    uint32_t pllmul = 0U, plldiv = 0U, hsivalue = 16000000U, tmp = 0U, msirange = 0U, result_hz = 0U;
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    if (RCC->CR & RCC_CR_HSIDIVF) { hsivalue = hsivalue / 4; }
    
    switch (tmp)
    {
        case 0x00U:  /* MSI used as system clock */
            msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos;
            result_hz = (32768U * (1U << (msirange + 1U)));
            break;
        case 0x04U:  /* HSI used as system clock */
            if ((RCC->CR & RCC_CR_HSIDIVF) != 0U)
            {
                result_hz = hsivalue / 4U;
            }
            else
            {
                result_hz = hsivalue;
            }
            break;
        case 0x08U:  /* HSE used as system clock */
            result_hz = hsivalue;
            break;
        default:  /* PLL used as system clock */
        
            pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
            plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
            pllmul = PLLMulTable[(pllmul >> RCC_CFGR_PLLMUL_Pos)];
            plldiv = (plldiv >> RCC_CFGR_PLLDIV_Pos) + 1U;
            
            /* HSI oscillator clock selected as PLL clock entry */
            if ((RCC->CR & RCC_CR_HSIDIVF) != 0U)
            {
                result_hz = (((hsivalue / 4U) * pllmul) / plldiv);
            }
            else
            {
                result_hz = (((hsivalue) * pllmul) / plldiv);
            }
    }
    return result_hz;
}

static inline bool delay(uint32_t delay_us)
{
    // if (delay_us > 0xFFFE) { delay_us = 0xFFFE; }
    while (TIM21->CNT < delay_us);
    return true;
}

void setup()
{

    // HCLK //
    //////////
    RCC->CR |= RCC_CR_HSION_Msk;                                // enable HSI16 clk source
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI;                           // set the PLL source mux to HSI16
    
    RCC->CR &= (uint32_t)(~RCC_CR_PLLON);                       // Disable the PLL    
    while((RCC->CR & RCC_CR_PLLRDY) != 0) { }                   // Wait for PLLRDY to be cleared

    FLASH->ACR |= FLASH_ACR_LATENCY;                            // Set latency to 1 wait state
    RCC->CFGR |= _HCLK32MHZ;                                    // Set the PLL multiplier to 4 and divider by 2 (32MHz)
    
    RCC->CR |= RCC_CR_PLLON;                                    // Enable the PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) { }                  // Wait until PLLRDY is set
    
    RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);                  // Select PLL as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }  // Wait untill PLL is switched on


    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;                                  // enable GPIO bus clock (used by multiple porta pins)
    
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
    NVIC_SetPriority(EXTI4_15_IRQn, _EXTI_ISR_PRIORITY); 

    // LPUART //
    ///////////
    GPIOA->MODER |= (GPIO_MODER_MODE1_1);                               // Set PA1 as alt func (0x10) 
    GPIOA->MODER &= ~(GPIO_MODER_MODE1_0);
    volatile uint32_t GPIOA_AF_LPUART = 0x6;
    GPIOA->AFR[0] |= (GPIOA_AF_LPUART << GPIO_AFRL_AFSEL1_Pos);         // Set alt func on pin pa1 to lpuart1
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED1_Msk;                        // "very fast" speed
    
    RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN_Msk;                          // enable the APB1 bus clock used by LPUART
    RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL_Msk);                           // APB clock selected as LPUART source
    
    LPUART1->CR1 &= ~((USART_CR1_M0_Msk) | (USART_CR1_M1_Msk));          // 1 start bit, 8 data bits, n stop bits
    uint32_t pclk = get_clk_freq();                                     // baud rate
    uint32_t baudrate = 31250U;    
    LPUART1->BRR = (uint32_t)UART_DIV_LPUART(pclk, baudrate);                               
    LPUART1->CR2 &= ~(USART_CR2_STOP_Msk);                               // 1 stop bit
    LPUART1->CR1 &= ~(USART_CR1_PCE_Msk);                                // no parity for MIDI
    LPUART1->CR2 &= ~(USART_CR2_MSBFIRST_Msk);                           // LSB first
    LPUART1->CR1 |=  ( USART_CR1_TXEIE_Msk );                              // Enable TXE (tx data reg transferred) interrupts
                    // | USART_CR1_TCIE_Msk);                                // Enable TC (transmission complete) interrupts
                    // | USART_CR3_EIE_Msk );                              // Enable error interrupts
    LPUART1->CR1 |= (USART_CR1_UE_Msk);                                  // enable the LPUART
    LPUART1->CR1 |= (USART_CR1_TE_Msk);                                  // send idle frame as first transmission
    // NVIC_SetPriority(LPUART1_IRQn, _LPUART_ISR_PRIORITY);
    // NVIC_EnableIRQ(LPUART1_IRQn);

    // TIMER21 //
    /////////////
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN_Msk;                            // enable the APB2 bus clock used by TIM21
    TIM21->CR1 |= (TIM_CR1_ARPE_Msk);                                   // preload buffer
    TIM21->PSC = 0xFFFF;                                                    // prescaler
    TIM21->ARR = 0xFFFFFFFE;                                            // auto-reload value
    
    // TIM21->DIER |= (TIM_DIER_UIE_Msk);                                  // enable interrupts
    // NVIC_EnableIRQ(TIM21_IRQn); 
    // NVIC_SetPriority(TIM21_IRQn,_TIM21_ISR_PRIORITY);    
    
    TIM21->CR1 |= (TIM_CR1_CEN_Msk);                                    // start the timer

}

void enable_led(bool enable, uint32_t sleep = 0)
{
    if (enable)
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    delay(sleep);
}

void toggle_led(uint32_t sleep = 0)
{
    if (GPIOA->ODR & GPIO_ODR_OD7_Msk)
        GPIOA->BSRR |= GPIO_BSRR_BR_7;
    else
        GPIOA->BSRR |= GPIO_BSRR_BS_7;
    delay(sleep);
}

int main()
{
    [[maybe_unused]] uint32_t tmp = get_clk_freq();
    setup();
    tmp = get_clk_freq();

    while (1)
    {
        // blinky!
        // toggle_led(0xFFFF);
    }
}

void EXTI4_15_IRQHandler()
{
    // do stuff
    uint32_t tmp_cnt = TIM21->CNT;
    if ((tmp_cnt - prev_cnt) > DEBOUNCE_COOLDOWN)
    {
        toggle_led();
    }
    prev_cnt = tmp_cnt;

    // clear the pending bit for the interrupt line
    if (EXTI->PR == EXTI_IMR_IM10_Msk)
        EXTI->PR |= (EXTI_PR_PIF10_Msk);
    if (EXTI->PR == EXTI_IMR_IM9_Msk)
        EXTI->PR |= (EXTI_PR_PIF9_Msk);
}

void TIM21_IRQHandler()
{
    // do stuff
    // toggle_led();
    // clear the interrupt
    TIM21->SR &= ~(TIM_SR_UIF_Msk);
}

void LPUART1_IRQHandler()
{
    // uint32_t errorflags = (LPUART1->ISR & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
    
    // if last byte was transferred from TDR to shift register then we are ready to send next byte
    if ((LPUART1->ISR & USART_ISR_TXE) != 0U)
    {
        if (msg_tx_count == 0)
        {
            LPUART1->ICR |= USART_ICR_TCCF_Msk;
        }
        if (msg_tx_count == msg.size())
        {
            msg_tx_count = 0;
        }
        else
        {
            
            // LPUART1->TDR = (uint8_t)(msg[msg_tx_count] & (uint8_t)0xFF);       
            LPUART1->TDR = (msg[msg_tx_count] & 0xFF);       
            msg_tx_count++;
        }
        return;
    }


}