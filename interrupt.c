/*
 * interrupt.c
 *
 *  Created on: 06/mar/2015
 *      Author: robotics
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "uartp/uartstdio.h"
#include "uartp/uart.h"
#include "init.h"


///
/// routine di servizio della uart1 (usata per comunicare con la tiva)
volatile uint8_t uart1buffer[], RX_PTR1 = 0, READ_PTR1 = 0;

void UART1IntHandler(void)
{
    uint32_t 	ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART1_BASE, ui32Status);
    //PRINTF("INT:\n");
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART1_BASE)){
        //
        // Read the next character from the UART and write it back to the UART.
        //
    	uart1buffer[RX_PTR1] = ROM_UARTCharGetNonBlocking(UART1_BASE);
    	/// echo
    	//ROM_UARTCharPutNonBlocking(UART0_BASE, uart1buffer[RX_PTR1]);
    	RX_PTR1++;
    	RX_PTR1 &= DIM_READ_BUFF - 1;
        //UARTCharPutNonBlocking(UART1_BASE,
        //		dato);
    }
}

///
// utilizzata per la lettura del sensore di colore

volatile int contLightPwm = 0;

void IntGPIOd(void){

	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1);

	/// incrementa il contatore delle transizioni del pwm prodotto dal sensore
	/// di luminosita'
	contLightPwm++;
//
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
}

void IntGpioA(void){

	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
	contLightPwm += 1;
}

extern volatile int procCom4;

///
/// routine di servizio del timer4
/// questa routine scatta e chiude la finestra di campionamento del sensore di colore.

void Timer4ISR(void){

	/// pulisce le interruzione del timer4
	ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	/// disabilita la porta D a generare interruzioni
	GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
	/// disabilita il timer 4
	TimerDisable(TIMER4_BASE, TIMER_A);
	/// segnala lo scadere del timer
	procCom4 = 1;
	/// spento il dispositivo
	HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_2 << 2))) = 0;
}

///
/// servizio al watchdog
void watchDOGisr(void){

	/// questa interrupt e' usata per segnalare che il reset sta per scattare
	/// spegne il led verde
	HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) &= ~GREEN_LED;

	while(1){
		/// blink led rosso + verde per segnalare l'imminente reset
		HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_1 << 2))) ^=  RED_LED;
		HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) ^= GREEN_LED; //~GREEN_LED;
		uint32_t i;
		for (i = 1000000; i > 0; i--);

		PRINTF("WD\n");
	}
}
