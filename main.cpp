/*
 * main.c
 */

/* Progetti di test per il bus I2C
	Created on: 20/apr/2015
 *
 *
 *
 *  Versione che sviluppa i controlli PID e e che si occupa di definire
 *  l'automa dei tasks.
 *  VERSIONE COMPLETA DI TEST: prove sui componenti in modalita' completa
 *  VERSIONE IN C++
 *  Ripresa dalla release precedente
 *  alcune prove
 *  In questa release si testa sia il giroscopio che l'accelerometro.
 *  SCHEDA MASTER che invia i comandi allo slave: servee a provare il protocollo di comunicazione
 */


#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "uartp/uartstdio.h"
#include "uartp/cmdline.h"
#include "I2C/tiva_i2c.h"
#include "gyro_init.h"
#include "accel/accel.h"
#include "gen_def.h"
#include "gyro_f.h"
#include "uartp/uart.h"
#include "init.h"

#include "pid.h"
#include "xbee/xbee.h"
#include "pwm/pwm.h"
#include "adc/adc.h"

#include "pwm/pwm.h"
#include "qei.h"
#include "sens_col_ir/sens1.h"
#include "allineamento/allineamento.h"
#include "distMis.h"
#include "Giroscopio.h"
#include "parse.h"




/// variabili globali
volatile int procCom = 0, tick;
volatile int procCom4 = 0;
volatile int ADCflag = 0;
/// buffer per la seriale che riceve i dati dalla raspPi
extern volatile uint8_t uart1buffer[16], RX_PTR1, READ_PTR1;

volatile distanza *dPtr;
/// puntatore globale per scrivere dentro la classe distMis usando la routine di servizio delle interruzioni
volatile distMis *distMisPtr;
void *servo;
temperatura *TEMPptr;

int main(void) {
	
	volatile uint32_t valore = 0, i, blink = 0, contatore, lampeggio_led;
	volatile int32_t arrot;
	volatile int16_t val1 = 0, x, y, z;
	//distanza DIST;
	distMis  MISURE;
	distMisPtr = &MISURE;
	//--------------------------//
	///definizione strutture/////
	//-------------------------//

	//volatile double d = 1.9845637456;
	gyro G;
	Giroscopio Rot;
	//accelerazione A;
	accelerometro A;
	cinematica CIN;
	/// servono differenti PID, almeno uno per la rotazione ed uno per lo spostamento
	/// per la rotazione sarebbero interessante usarne 2, uno per la ortazione soft ed uno per la rotazione
	/// brusca.
	ControlloPID cPid[3], *pidPtr;
	/// descrittore della sintassi dei comandi
	syn_stat synSTATO;
	/// modulo zigbee per telemetria
	//xbee XB;
	/// pwm servi e motori
	pwm PWM, pwmServi;
	/// struttura del sensore di colore
	colore COL;
	/// sensore di temperatura ad infrarossi
	temperatura TEMP;
	//TEMPER sensIR;
	/// informazioni sul sopravvissuto
	survivor SUR;
	//inizializzazione struttura per qei
	//qei QEI;
	/// oggetto che riallinea il mezzo
	allineamento AL;

	/// disabilita le interruzioni
	DI();
	//pidPtr = CTRL;
	//dPtr = &DIST;


	TEMPptr =  &TEMP;
//	CIN.Aptr = &A;
//	CIN.distPTR = &DIST;
//	CIN.vel = 0.0;

	glb  COLLECTDATA;
	comando CMD;
	//DATA.distPtr = &DIST;
	//passaggio degli indirizzi delle strutture alla struttura generale
	//dati_a_struttura(&G, &DIST, &CIN, &COL, &TEMP, &SUR, &DATA);
	/// l'oggetto COLLECTDATA (glb) e' una struttara che contiene i puntatori alle strutture e classi del progetto
	datiRaccolti(&CIN, &COL, &TEMP, &SUR, &MISURE, &Rot, &COLLECTDATA);

	/// setup di base
	setupMCU();
	/// imposta i parametri del PID
	//setupPID(CTRL);
	/// imposta le UART e setta la PRINTF sulla 1 in modo da trasmettere la telemetria
	//setupUART(1);
	/// imposta le UART e setta la PRINTF sulla 0
	setupUART(0);
	PRINTF("impostata UART0 per debug\n");
	/// messaggio d'inizio
	PRINT_WELCOME();
    //inizializzo l'i2c
	InitI2C0();
	/// messaggio d'inizio
	PRINTF("inizializzato I2C\n");

	//initGyro(&G, Z_AXIS);
	tick = 0;
	/// inizializza il timer 0 e genera un tick da 10 ms
	initTimer0(INT_STEP_10_MS, &G);
	PRINTF("inizializzato TIMER0\n");

	/// inizializza il timer 1
	//initTimer1(100);
	/// inizializza il contatore della persistenza del comando
	synSTATO.tick = 0;


	initAdc(distMisPtr);
	PRINTF("inizializzato ADC\n");
	/// reset dell'automa di analisi della sintassi
	resetAutoma(&synSTATO);
	PRINTF("inizializzato automa comandi\n");



	/// abilita le interruzioni
	EI();
	PRINTF("abilitate interruzioni\n");

	contatore = 0;
	volatile uint8_t numSens, sblocco = 0;
	while(1){

		///extern volatile uint8_t uart1buffer[16], RX_PTR1, READ_PTR1;
		/*    *****     */
		// controllo di messaggio sulla seriale 1 (ricevuto comando da rasp
		if (READ_PTR1 != RX_PTR1){
			/// analizza il comando e imposta il valore dell'oggetto CMD (comando)
			 parse(&synSTATO, &CMD);
			 /// aggiorna il buffer
			 READ_PTR1++;
			 READ_PTR1 &= DIM_READ_BUFF1 - 1;
		}
//		if (synSTATO.valid == VALIDO && synSTATO.token != ERRORE){
//			/// il comandoche e' stato analizzato ha prodotto un risultat adeguato
//			rispondiComando(&synSTATO, &COLLECTDATA);
//			/// avendo terminato la risposta, la validità dell'automa
//			/// va rimossa.
//			synSTATO.valid = NON_VALIDO;
//		}

		if (procCom == 1){
			/// ciclo da 10 ms
			procCom = 0;

			if (sblocco == 1 && numSens < 6){

				numSens++;
				uint8_t buff[8], check = 0;
				buff[0] = 'D';
				buff[1] = numSens;
				check ^= buff[0];
				check ^= buff[1];
				check ^= CHECK_SUM;
				buff[2] = check;
				for (uint8_t i= 0; i < 3; i++)
					ROM_UARTCharPut(UART1_BASE, buff[i]);

				/// infine invia il terminatore di stringa '*'
				ROM_UARTCharPut(UART1_BASE, '*');

			}
			else
				sblocco = 0;
		}
		/// lampeggio ed interrogazione dei 5 sensori.
		if (tick >= 100){
			PRINTF("\n----\n");
			///
			/// accende il pin PB5
			HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) |=  GPIO_PIN_5;
			tick = 0;
			HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) ^=  GPIO_PIN_3;
			/// invia i bytes di richiesta dei sensori di distanza 1,2,3,4,5
			/// invia tutti i caratteri nella stringa
			numSens = 0;
			sblocco = 1;

		}
	}
}



uint8_t printFloat(double number, uint8_t digits){

  //size_t n = 0;

  // Handle negative numbers
  if (number < 0.0)
  {
     PRINTF("-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  PRINTF("%d", int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    PRINTF(".");
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    PRINTF("%d", toPrint);
    remainder -= toPrint;
  }


  return 0;
}
