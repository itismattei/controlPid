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
 *  Si testano anche i sensori di distanza (5 sensori)
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
	/// inizializza il giroscopio con banda a 190Hz invece cha a 95Hz
	Rot.initGyro(ODR_190 | Z_AXIS);
	//initGyro(&G, Z_AXIS);
	tick = 0;
	/// inizializza il timer 0 e genera un tick da 10 ms
	initTimer0(INT_STEP_10_MS, &G);
	PRINTF("inizializzato TIMER0\n");
	/// imposta il passo di integrazione per il calcolo dell'angolo
	Rot.tick = (INT_STEP_10_MS / 1000.0) ;
	/// inizializza il timer 1
	//initTimer1(100);
	/// inizializza il contatore della persistenza del comando
	synSTATO.tick = 0;
	/// inizializza il pwm
	//pwmMotInit(&PWM);
	// TODO: //pwmServoInit (&pwmServi);
	/// inizializza l'adc e lo prepara a funzionare ad interruzioni.

	initAdc(distMisPtr);
	PRINTF("inizializzato ADC\n");
	/// reset dell'automa di analisi della sintassi
	resetAutoma(&synSTATO);
	PRINTF("inizializzato automa comandi\n");

	//servo = (pwm *) &pwmServi;
	/// inizializzazione accelerometro
	A.testAccel();
	if (A.isPresent == true)
		/// imposta l'accelerometro
		A.impostaAccel();
	/// iniziailizzazione del lettore encoder
	//qei_init(&QEI);

	/// abilita le interruzioni
	EI();
	PRINTF("abilitate interruzioni\n");
	/// attende che il sensore vada a regime
	//if (G.IsPresent == OK){
	if (Rot.IsPresent == OK){
		PRINTF("\nAzzeramento assi giroscopio\n");
		while (blink < 70){
			if (procCom == 1){
				procCom = 0;
				blink++;
			}
		}
		blink = 0;
		/// azzeramento degli assi
		//azzeraAssi(&G);
		Rot.azzeraAssi();
#ifdef _DEBUG_
		PRINTF("media: ");
		printFloat(Rot.media, 4);
		PRINTF("\nm: ");
		printFloat(Rot.m, 4);
		PRINTF("\nq: ");
		printFloat(Rot.q, 4);
		PRINTF("\n");
#endif
	}

	/// test della presenza del modulo zig-bee
	/// il modulo zig-be si attiva con al sequnza '+++' e risponde con 'OK' (maiuscolo)
//	if (testXbee() == 0){
//		// ok;
//		XB.present = 1;
//		PRINTF("Modulo xbee presente.\n");
//	}
//	else{
//		XB.present = 0;
//		PRINTF("Modulo xbee non presente.\n");
//	}
//
//	pwm_power(&PWM);
	contatore = 0;

	//// inizializza l'accelrometro
	//stato =  writeI2CByte(CTRL_REG1_A, ODR1 + ODR0 + ZaxEN + YaxEN + XaxEN);
	// scrivo nel registro 0x20 il valore 0x0F, cioe' banda minima, modulo on e assi on
	/// sintassi: indirizzo slave, num parm, indirizzo reg, valore da scrivere
	//I2CSend(ACCEL_ADDR, 2, CTRL_REG1_A, ODR1 + ODR0 + ZaxEN + YaxEN + XaxEN);
//	A.isPresent = testAccel();
//	if (A.isPresent)
//		impostaAccel(&A);
//
//	/// taratura sul sensore di luminosita'
//	whiteBal(&COL);
//	/// taratura del sensore di temepratura
//	taraturaTemp(&TEMP);
//
//	///
//	qei_test(&QEI);
	/// task principale
	int tempCont = 0;
	while(1){

		if (HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_0 << 2))) != GPIO_PIN_0){
			PRINTF("Azzeramento assi \n");
			Rot.azzeraAssi();
			PRINTF("media:\t ");
			printFloat(Rot.media, 4);
			PRINTF("\nm:\t ");
			printFloat(Rot.m, 4);
			PRINTF("\nq:\t ");
			printFloat(Rot.q, 4);
			PRINTF("\n");
			//Rot.yawF = 0.0;
			tempCont = 0;
		}
		///extern volatile uint8_t uart1buffer[16], RX_PTR1, READ_PTR1;
		/*    *****     */
		// controllo di messaggio sulla seriale 1 (ricevuto comando da rasp
		if (READ_PTR1 != RX_PTR1){
			/// analizza il comando e imposta il valore dell'oggetto CMD (comando)
			 parse(&synSTATO, &CMD);
			 /// aggiorna il buffer
			 READ_PTR1++;
			 READ_PTR1 &= 0xF;
		}
		if (synSTATO.valid == VALIDO && synSTATO.token != ERRORE){
			/// il comandoche e' stato analizzato ha prodotto un risultat adeguato
			rispondiComando(&synSTATO, &COLLECTDATA);
			/// avendo terminato la risposta, la validità dell'automa
			/// va rimossa.
			synSTATO.valid = NON_VALIDO;
		}
		/// invia la risposta per i comandi di rotazione, quando sono stati eseguiti
//		if(pidPtr->rispondi == TRUE){
//			rispostaRotazione(pidPtr, &synSTATO);
//			pidPtr->rispondi = FALSE;
//		}
//
		/// aggiorna il PID ogni tick del timer che sono 10ms
		if (procCom == 1 ){
			//UARTCharPutNonBlocking(UART1_BASE, 'c');
			procCom = 0;
			contatore++;
			lampeggio_led++;
		}
//
//			if(lampeggio_led >= 50)
//			{
//				lampeggio_led = 0;
//
//				 if(DATA.surPtr->isSurvivor == TRUE )
//				{
//					if(HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GREEN_LED << 2))) != GREEN_LED )
//						HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (RED_LED << 2))) = 0;
//
//					HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GREEN_LED | RED_LED << 2))) ^=  GREEN_LED | RED_LED;
//
//
//				}
//
//				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GREEN_LED << 2))) ^=  GREEN_LED;
//			}
//
//
//			/*  LETTURA DEL COMANDO */
//
//			/// restituisce l'indirizzo del PID da utilizzare nel successivo processo di calcolo
//			pidPtr =  leggiComando(&synSTATO, CTRL, pidPtr, &DATA);

			/* LETTURA SENSORI  */


			/// effettua i calcoli solo se il giroscopio e' presente
			/// TODO: il PID viene calcolato ongi 10ms oppure ogni 20ms? Come è meglio?


			/* misura gli encoder e calcola spostamenti e velocità */
			/* misura i sensori di distanza */
			if (tick >= 100){

				/// TODO controllare se riesce a funzionare mentre legge le accelerazioni su I2C
				ROM_ADCProcessorTrigger(ADC0_BASE, 0);
				/// accende il pin PB5
				HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) |=  GPIO_PIN_5;
				tick = 0;
				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) ^=  GPIO_PIN_3;
			}

			if(ADCflag == 1){
				/// arrivata una nuova conversione AD
				ADCflag = 0;
				/// i dati grezzi vongono copiati nella classe distMis.
				/// verificato il funzionamento del puntatore in adc.cpp
				/// le due righe successive possono essere tolte.
				//for (int i = 0; i < 6; i++)
				//	MISURE.dI[i] = DIST.dI[i];
#ifdef _DEBUG_
				for(int i = 1; i < 6; i++){

					PRINTF("val%d: %d \t", i, MISURE.dI[i]);

				}
				PRINTF("\n");

#endif
				/// converte la misure grezza in mm
				MISURE.rawTomm();
#ifdef _DEBUG_
				/// ricopia nella struttare DIST:
				for(int attesa = 1; attesa < 6; attesa++){
					//if (attesa == 3)
					//	continue;
					PRINTF("mm(%d): %d \t", attesa, MISURE.d_mm[attesa]);
				}
				PRINTF("\n");
				//PRINTF("Temperatura %d\n", Rot.getTemp());
#endif
			}



			/// misura i dati forniti dall'accelerometro se disponibili
//			if(A.isPresent)
//				misuraAccelerazioni(&A);
			/// le misure del giroscopio invece sono effettuate solo dall'apposito pid
			if (procCom == 1 ){
				contatore++;
				procCom = 0;
				if (Rot.IsPresent == OK){
					/// aggiorna l'angolo di yaw
					Rot.misuraAngoli();
#ifdef _DEBUG_
					if(contatore >= 100){
						contatore = 0;
						PRINTF("%d\tasse z: %d\t",tempCont++, Rot.yaw);
						printFloat(Rot.yawF, 4);
						PRINTF("\t");
						printFloat(Rot.yawF0, 4);
						if (A.isPresent == true){
							PRINTF("\t");
							A.misuraAccelerazioni();
						}
						PRINTF("\n");
					}
#endif
				}
			}
			/*if(G.IsPresent == OK)
				if( contatore == 1){
					/// ogni 10 ms effettua il calcolo del PID
					contatore = 0;
					HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_0 << 2))) |=  GPIO_PIN_0;
					PID(&G, pidPtr, &PWM, &CIN);
					setXPWM(&CTRL[1], &PWM);
					procCom = 0;
					HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_0 << 2))) &=  ~GPIO_PIN_0;
					blink++;
					/// lampeggio del led con periodo di 2 s.
					if (blink >= 100){
						HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + ((GPIO_PIN_2 | GPIO_PIN_1) << 2))) = 0;
						HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) ^= GPIO_PIN_3;
						blink = 0;
					}
				///provvede ad integrare la misura della velcita' angolare ogni 10 ms
				//misuraAngoli(&G);
				//PRINTF("asse x: %d\t", G.pitch);
				//PRINTF("\tasse y: %d\t", G.roll);
				//PRINTF("\tasse z: %d\n", G.yaw);
				//PRINTF("uscita PID: %d\n", C.uscita);
			}*/

			/* RISPOSTA AL COMANDO */
			//inviaSensore(&synSTATO, &DATA);

		//}
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
