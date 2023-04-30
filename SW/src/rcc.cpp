#include <rcc.hpp>
#include <stm32l011xx.h>

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


