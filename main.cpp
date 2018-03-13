/*
 * main.c
 */

/*  Test sul sensore di umidita'
	Created on: 20/sett/2017
 *
 *	Si realizzano alcuni testi di movimento, allineamento e lettura sensori
 *  NOTE: Provata la comunicazione con raspberry: invio di 3 comandi , F, B, S.
 *        risultato ok.
 *        Provata la comunicazione con raspberry dei sensori di distanza 1 e 3.
 *        risultato dipendente dallo sleep della raspberry. Comunicazione ok,
 *        al piu' con un ritardo di un ciclo.
 *
 *  RIMOSSA LA STRUCT GLB e SOSTITUITA CON LA CLASSE ALLSTRUCT CHE PERMETTE DI AVERE TUTTE LE STRUCT
 *  DISPNIBILE NEL PROGRAMMA.
 *
 *  CORREZIONE DEL JITTER SULL'INTERVALLO DI INTEGRAZIONE
 *
 *  AGGIUNTO VETTORE DI COEFFICIENTI PER IL PID
 *
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
#include "I2C/tiva_i2c.h"
#include "gyro_init.h"
#include "gen_def.h"
#include "uartp/uart.h"
#include "init.h"
#include "I2C/i2cTiva.h"
#include "pwm/motpwm.h"
#include "power.h"
#include "pid.h"

#include "sensHum/HIH87Hum.h"


/// variabili globali
volatile int procCom = 0, tick10, tick100, millis10 = 0;
volatile int procCom4 = 0;
volatile int ADCDataReadyFlag = 0;
/// buffer per la seriale che riceve i dati dalla raspPi
extern volatile uint8_t uart1buffer[16], RX_PTR1, READ_PTR1;

volatile distanza *dPtr;
volatile PWM_SERVI *sensPtr;
/// puntatore globale per scrivere dentro la classe distMis usando la routine di servizio delle interruzioni
volatile distMis *distMisPtr;
void *servo;
temperatura *TEMPptr;



int main(void) {

	/// disabilita le interruzioni
	DI();

	/// setup di base
	setupMCU();

	/// imposta le UART e setta la PRINTF sulla 1 in modo da trasmettere la telemetria
	//setupUART(1);
	/// imposta la UART1 a ricevere da raspberry (PB0 = RX; PB1 = TX)
	//setupUART(1);
	/// imposta le UART e setta la PRINTF sulla 0
	setupUART(0);
	PRINTF("\n######\nImpostata UART0 per debug\n");
	/// messaggio d'inizio
	PRINT_WELCOME();

	/// MODULO COMUNICAZIONE BUS I2C ///
	//I2C TEST(I2C0_BASE);
	//I2C BUS_COMM(I2C1_BASE);
	I2C BUS_COMM[3] = {I2C1_BASE, I2C1_BASE, I2C1_BASE};
	/// messaggio d'inizio
	PRINTF("inizializzato I2C\n");

	/// imposta il canale I2C sul numero 1 che uso per il test del sensore di umidita'
	I2C TEST_HUM_SENS(I2C1_BASE);
	HIH8_7Hum HIH8;
	//L'indirizzo base del sensore e' 0x27
	HIH8.attachI2C(&TEST_HUM_SENS, 0x27);
	//Manda solo byte su SDA e conclude con lo STOP
	HIH8.newData();

	tick10 = tick100 = 0;
	/// inizializza il timer 0 e genera un tick da 10 ms
	initTimer0(INT_STEP_10_MS);
	PRINTF("inizializzato TIMER0\n");
	/// inizializza il timer 2 a contare con prescaler ogni 100us
	initTimer2(_100_US);
	PRINTF("inizializzato TIMER2\n");
//	/// imposta il passo di integrazione per il calcolo dell'angolo
//	Rot.tick = (INT_STEP_10_MS / 1000.0) ;
//	/// inizializza il timer 1
//	//initTimer1(100);

	/// abilita le interruzioni
	EI();
	PRINTF("abilitate interruzioni\n");

	/////////////////////////////////////////////////////////
	///
	///      TASK PRINCIPALE
	///
	/////////////////////////////////////////////////////////

	while(1){

/************************************************************/
/*  			ATTIVITA' SVOLTE AD OGNI CICLO				*/
/************************************************************/

		/// AZIONI DA COMPIERE OGNI 1s ///
		if (tick100 >= 100){
			/// converte il dato letto al ciclo precedente
			HIH8.readRaw();
			HIH8.convertRaw();
			/// registra la classe che gestisce i dati del sensore
			if(HIH8.dataValid && ! HIH8.dataOld){
				PRINTF("Dato valido \n");
				PRINTF("Um: %s\n", HIH8.humidity);
				PRINTF("T: %s\n", HIH8.temperature);
			}
			if (HIH8.dataOld)
				PRINTF("Dato vecchio\n");

				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) ^=  GREEN_LED;
				/// si assicura che il led rosso sia spento
				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_1 << 2))) &=  ~GPIO_PIN_1;
//			}
//
//			else{
//				/// segnala che la batteria sta finendo, facendo lampeggiare il rosso
//				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_1 << 2))) ^=  GPIO_PIN_1;
//				/// spegne il led verde
//				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) &=  ~GREEN_LED;
//			}

			//Campiona il dato per il prossimo ciclo
			HIH8.newData();
			//// reset del contatore
			tick100 = 0;

		}
	}
}


