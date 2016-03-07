/*
 * tiva_timer.c
 *
 *  tiva c
 *  timer sscheduler
 *  Author: robocupjr 15
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "gyro_f.h"
#include "pwm/pwm.h"
#include "sens_col_ir/sens.h"


extern volatile int procCom, tick;
extern pwm *servo;
extern temperatura * TEMPptr;

extern volatile uint8_t uart1buffer[16], RX_PTR1, READ_PTR1;

///
/// routine di servizio del timer0
void Timer0ISR(void){

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    ///HWREGBITW(&g_ui32Flags, 0) ^= 1;

    //PIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    procCom = 1;
    tick++;
//    if ((tick % 50) == 0){
//		uint8_t valore = 0;
//		uart1buffer[RX_PTR1++] = 'D';
//		valore += 'D';
//		uart1buffer[RX_PTR1++] = 1;
//		valore += 1;
//		valore ^= 0xA9;
//		uart1buffer[RX_PTR1++] = valore;
//		uart1buffer[RX_PTR1++] = '*';
//		RX_PTR1 &= 0xF;
//	 }

    //
    // Use the flags to Toggle the LED for this timer
    //
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, g_ui32Flags << 1);

    //
    // Update the interrupt status on the display.
    //
}

void Timer1ISR (void)
{
	volatile static uint32_t cont_sens_ir=0;
	cont_sens_ir++;

	//faccio girare il motore del sensore di temperatura
	if(cont_sens_ir >= 7)
	{
		cont_sens_ir = 0;
		//controllo il pwm e di conseguenza l'angolo pwm=10->90�
		if(servo->delta_2 == 10)
		{
			servo->delta_2 = 5;
			pwmPowerA7(servo, -90 );
		}
		else
		{
			servo->delta_2 = 10;
			pwmPowerA7(servo, 90 );
		}
	}

	//adesso occorre leggere il sensore, passandogli l'opportuno puntatore
	readTemp(TEMPptr);
	/// occorre confrontare la temperatura con il valore di default letto all'inizio.
	if ((int)(TEMPptr->Temp) > 3 * TEMPptr->T_tar )
	{
		/// c'e' un ferito
		// TODO:
		/// mettere il codice

	}

}




//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************


void initTimer0(int interval, gyro *G, ){
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();
    volatile int tick = 0;

    tick = (ROM_SysCtlClockGet() / 1000) * interval;

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF1 & PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);


    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Configure the two 32-bit periodic timers.
    //
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    /// imposta il time_out
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, tick);
    G->tick = (float) interval / 1000;
    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER0A);
    //ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}



void initTimer1(int interval){
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    volatile int tick = 0;

    tick = (ROM_SysCtlClockGet() / 1000) * interval; //100 sono i ms
    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    /// imposta il time_out
    TimerLoadSet(TIMER1_BASE, TIMER_A, tick);

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER1A);
    //ROM_IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    //ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);


}


