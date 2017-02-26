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
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "gyro_f.h"
#include "pwm/pwm.h"
#include "sens_col_ir/sens.h"


extern volatile int procCom, tick10, tick100;
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

    /// updates timer counters
    procCom = 1;
    tick10++;
    tick100++;

}

///
/// potrebbe non servire perché si può metterenei task schedulati ogni 100 ms!!!

void Timer1ISR (void)
{
	volatile static uint32_t cont_sens_ir=0;
	cont_sens_ir++;

	//faccio girare il motore del sensore di temperatura
	if(cont_sens_ir >= 7)
	{
		cont_sens_ir = 0;
		//controllo il pwm e di conseguenza l'angolo pwm=10->90°
		if(servo->delta_2 == 10)
		{
			servo->delta_2 = 5;
			//pwmPowerA7(servo, -90 );
		}
		else
		{
			servo->delta_2 = 10;
			//pwmPowerA7(servo, 90 );
		}
	}

	//adesso occorre leggere il sensore, passandogli l'opportuno puntatore
	//readTemp(TEMPptr);
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


void initTimer0(int interval){
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
    //G->tick = (float) interval / 1000;
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


///
/// per comandare il motore di rotazione del sensore sembrerebbe superfluo,
/// in quanto può essere chiamato all'interno dello schedulatore dei vari tempi.

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

///
/// Usa il timer 2 per portare la base dei tempo a 100us usando il prescaler.
/// in questo modo si vuole avere un intervallo regolare per l'integrazione del PID
/// leggendo il valore del contatore di timerA a 32 bit
/// Il valore del prescale e' posto a 7999 in modo che il divisore sia 7999+1 = 8000
/// e quindi arrivi al contatore un impulso ogni 100us. Tenendo conto che il PID
/// e' calcolato ogni 10ms si ha un fattore di correzione del tempo di integrazione
/// pari ad 1/100. Se l'errore del timer del PID dovesse essere inferiore si potrebbe
/// abbassare questo timer a 10 us.
/// A 100us il tempo di reset del registro del contatore a 32 bit si resetta ogni 5 giorni circa

void initTimer2(uint32_t BASE_TEMPO){
	/// divide il clock per 8000 e quindi calcola il prescaler a 7999
	uint32_t PRESCALER = (ROM_SysCtlClockGet()  / 1000000) * BASE_TEMPO - 1;
	/// abilita il modulo timer2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
	/// configura il funzionamento periodico
	TimerConfigure(WTIMER2_BASE, TIMER_CFG_PERIODIC | TIMER_CFG_SPLIT_PAIR);
	//TimerLoadSet(TIMER2_BASE, TIMER_A, -1);
	//HWREG(TIMER2_BASE + TIMER_O_PP) = TIMER_PP_SIZE_32;
	//HWREG(WTIMER2_BASE + TIMER_O_TAPR) = PRESCALER;
	/// imposta il prescaler a 7999
	//HWREG(TIMER2_BASE + TIMER_O_TAPR) = PRESCALER;
	TimerPrescaleSet(WTIMER2_BASE, TIMER_A, PRESCALER);
	/// avvia il timer
	TimerEnable(WTIMER2_BASE, TIMER_A);

}

