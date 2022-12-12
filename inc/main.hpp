#ifndef __MAIN_H__
#define __MAIN_H__

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#endif // __MAIN_H__