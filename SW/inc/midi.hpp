#ifndef __MIDI_HPP__
#define __MIDI_HPP__

extern "C" {
void EXTI4_15_IRQHandler();
void LPUART1_IRQHandler();
}

// void increment_midi_preset();
// void decrement_midi_preset();
void print_midi_msg();
// void send_midi_bytes();


#endif // __MIDI_HPP__