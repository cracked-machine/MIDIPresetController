#include <midi.hpp>
#include <array>
#include <stdint.h>
#include <stm32l011xx.h>
#if DEBUG
    #include <SEGGER_RTT.h>
#endif
#include <led.hpp>

const volatile uint32_t DIPSW_DEBOUNCE_COOLDOWN{ 200 };
volatile uint32_t dipsw_pa0_prev_cnt = 0;
volatile uint32_t dipsw_pa4_prev_cnt = 0;
volatile uint32_t dipsw_pc14_prev_cnt = 0;
volatile uint32_t dipsw_pc15_prev_cnt = 0;
const volatile uint32_t FOOTSW_DEBOUNCE_COOLDOWN{ 100 };
volatile uint32_t footsw_up_prev_cnt = 0;
volatile uint32_t footsw_down_prev_cnt = 0;

// MIDI CC messages - 0xnc, 0xcc, 0xvv 
// LSB first: starting left to right
//   n is the status type (0xB)
//   c is the MIDI channel
//   cc is the controller number (0-127)
//   vv is the controller value (0-127)
// 0x82 is nemesis preset up cmd 
std::array<uint8_t, 3> midi_up_preset_msg{0xB0, 0x82, 0x00};  

// MIDI CC messages - 0xnc, 0xcc, 0xvv 
// LSB first: starting left to right
//   n is the status type (0xB)
//   c is the MIDI channel
//   cc is the controller number (0-127)
//   vv is the controller value (0-127)
// 0x80 is nemesis preset down cmd
std::array<uint8_t, 3> midi_down_preset_msg{0xB0, 0x80, 0x00}; 

uint8_t midi_channel{0};    // only 4 lsb bits used 
#define midi_chan_mask (midi_channel << 0U)
uint8_t midi_preset_counter{8};

// bytes will only be processed by LPUART ISR when counts are zero, so start at non-zero
uint16_t msg_tx_count{0};


enum preset_enum {
    UP_COMMAND,
    DOWN_COMMAND,
    IDLE
};

preset_enum transmit_command_direction = IDLE;

void increment_midi_preset()
{
    if (midi_preset_counter > 0)
    {
        midi_preset_counter--;
        midi_down_preset_msg[2] = midi_preset_counter;
        #if DEBUG
            SEGGER_RTT_printf(0, "DOWN %d\r\n", midi_preset_counter); 
        #endif
        print_midi_msg();
    }    
    transmit_command_direction = DOWN_COMMAND;
}

void decrement_midi_preset()
{
    if (midi_preset_counter < 0xFE)
    {
        midi_preset_counter++;
        midi_up_preset_msg[2] = midi_preset_counter;
        #if DEBUG
            SEGGER_RTT_printf(0, "UP %d\r\n", midi_preset_counter); 
        #endif
        print_midi_msg();
    }
    transmit_command_direction = UP_COMMAND;    
}

void send_midi_bytes()
{
    if (transmit_command_direction == UP_COMMAND)
    {
        // transmit up preset bytes until count = 0
        if (msg_tx_count == 0)
        {
            LPUART1->ICR |= USART_ICR_TCCF_Msk;
        }
        if (msg_tx_count == midi_down_preset_msg.size())
        {
            msg_tx_count = 0;
            LPUART1->CR1 &=  ~( USART_CR1_TXEIE_Msk );                           
        }
        else
        {
            // transfer next byte     
            LPUART1->TDR = (midi_down_preset_msg[msg_tx_count] & 0xFF);       
            msg_tx_count++;
        }
    }

    if (transmit_command_direction == DOWN_COMMAND)
    {
        if (msg_tx_count == 0)
        {
            LPUART1->ICR |= USART_ICR_TCCF_Msk;
        }
        if (msg_tx_count == midi_down_preset_msg.size())
        {
            msg_tx_count = 0;
            LPUART1->CR1 &=  ~( USART_CR1_TXEIE_Msk );                           
        }
        else
        {
            // transfer next byte     
            LPUART1->TDR = (midi_down_preset_msg[msg_tx_count] & 0xFF);       
            msg_tx_count++;
        }
    }    
}

void print_midi_msg()
{
    #if DEBUG
        SEGGER_RTT_printf(0, "midi_down_preset_msg:");
        SEGGER_RTT_printf(0, "0x%x", midi_down_preset_msg[2]);
        SEGGER_RTT_printf(0, "%x", midi_down_preset_msg[1]);
        SEGGER_RTT_printf(0, "%x\r\n", midi_down_preset_msg[0]); 
        SEGGER_RTT_printf(0, "midi_up_preset_msg:");
        SEGGER_RTT_printf(0, "0x%x", midi_up_preset_msg[2]);
        SEGGER_RTT_printf(0, "%x", midi_up_preset_msg[1]);
        SEGGER_RTT_printf(0, "%x\r\n", midi_up_preset_msg[0]);    
    #endif
}

/// @brief Write the 4 LSB according to the dip switch settings
void read_midi_channel()
{
    // PA4  - EXT_MIDI_CH_BIT1
    if (GPIOA->IDR & (GPIO_IDR_ID4_Msk))
    {
        midi_channel |= (1U << 0U);
    }
    // PC14 - EXT_MIDI_CH_BIT2
    if (GPIOC->IDR & (GPIO_IDR_ID14_Msk))
    {
        midi_channel |= (1U << 1U);
    }
    // PC15 - EXT_MIDI_CH_BIT3
    if (GPIOC->IDR & (GPIO_IDR_ID15_Msk))
    {
        midi_channel |= (1U << 2U);
    }    
    // PA0  - EXT_MIDI_CH_BIT4
    if (GPIOA->IDR & (GPIO_IDR_ID0_Msk))
    {
        midi_channel |= (1U << 3U);
    }    

    // most sig. nibble of `midi_channel` should be 0000, so we can shift the 
    // whole byte in without clobbering the existing MSB status bit.
    midi_up_preset_msg[0] |= midi_channel << 0U;
    midi_down_preset_msg[0] |= midi_channel << 0U;
}

////////////////////////////////
/// Interupt Service Routines
////////////////////////////////

extern "C" {
void EXTI4_15_IRQHandler()
{
    // #if DEBUG
    // SEGGER_RTT_printf(0, "EXTI4_15_IRQHandler\r\n");
    // #endif

    // PA9 EXTI - EXT_DOWN_PRESET_SW
    if (EXTI->PR & EXTI_IMR_IM9_Msk)                
    {
        uint32_t tmp_cnt = TIM21->CNT;
        if ((tmp_cnt - footsw_down_prev_cnt) > FOOTSW_DEBOUNCE_COOLDOWN)
        {
            toggle_led();   // visual debug. This should be removed at some point
            increment_midi_preset();
            
            // Enable TXE (tx data reg transferred) interrupts to start the LPUART transmission
            LPUART1->CR1 |=  ( USART_CR1_TXEIE_Msk );   
        }
        else
        {
            // #if DEBUG
            // SEGGER_RTT_printf(0, "Debounced\r\n");
            // #endif
        }
        EXTI->PR |= (EXTI_PR_PIF9_Msk);
        footsw_down_prev_cnt = tmp_cnt;
    }

    // PA10 EXTI - EXT_UP_PRESET_SW
    else if (EXTI->PR & EXTI_IMR_IM10_Msk)         
    {
        uint32_t tmp_cnt = TIM21->CNT;
        if ((tmp_cnt - footsw_up_prev_cnt) > FOOTSW_DEBOUNCE_COOLDOWN)
        {
            toggle_led();   // visual debug. This should be removed at some point
            decrement_midi_preset();
            // Enable TXE (tx data reg transferred) interrupts to start the LPUART transmission
            LPUART1->CR1 |=  ( USART_CR1_TXEIE_Msk );   
        }
        else
        {
            // #if DEBUG
            // SEGGER_RTT_printf(0, "Debounced\r\n");
            // #endif
        }
        EXTI->PR |= (EXTI_PR_PIF10_Msk);            
        footsw_up_prev_cnt = tmp_cnt;
    }
}

void LPUART1_IRQHandler()
{
    
    // if last byte was transferred from TDR to shift register then we are ready to send next byte
    if ((LPUART1->ISR & USART_ISR_TXE) != 0U)
    {
        send_midi_bytes();
    }
}

}