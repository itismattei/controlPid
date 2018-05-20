/*
 * main.cpp
 * aprile 2018
 *
 * VERSIONE per ROMECUP 2018
 * USO: test dei sensori e sviluppo di correzione nelle misure
 *
 * taratura ed interpolazione dei sensori di distanza
 * gennaio 2018: prova di avanzamento del rover e di allinemamento dei cingoli.
 * si usano le letture degli encoder e si cerca di porre un controllo in retroazione
 * sui valori letti.
 */


#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "uartp/uartstdio.h"
#include "I2C/tiva_i2c.h"
#include "gyro_init.h"
#include "accel/accel.h"
#include "gen_def.h"
#include "gyro_f.h"
#include "uartp/uart.h"
#include "init.h"
#include "Jitter/Jitter.h"

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
#include "encQuad.h"
#include "I2C/i2cTiva.h"
#include "pwm/motpwm.h"
#include "power.h"
#include "pid.h"
#include "maze/maze.h"


/// variabili globali
volatile int procCom = 0, tick10, tick20, tick100, millis10 = 0;
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
	
	volatile uint32_t valore = 0, i, blink = 0, contatore, lampeggio_led, misuraJitter = 0, mJ = 0;
	volatile int32_t arrot;
	volatile int16_t val1 = 0, x, y, z;
	syntaxStatus synStat;

	//--------------------------//
	///definizione strutture/////
	//-------------------------//


	/// MODULO PER LA MISURA DEI SENSORI DI DISTANZA
	//distanza DIST;
	distMis  MISURE;
	distMisPtr = &MISURE;

	/// MODULO DI CONTROLLO DELLA BATTERIA ///
	/// imposta il livello di soglia della batteria a 2600 che corrisponde a:
	/// 2930/7.52*6.9
	power BATT(2500);

	/// MODULO PWM PER MOTORI SERVO ///
	/// disabilitati per olimpiade di robotica
	//PWM_SERVI KIT, MOT_SENS;
	//sensPtr = &MOT_SENS;

	/// MODULI ENCODER ///
	encQuad ENC0(QEI0_BASE);
	encQuad ENC1(QEI1_BASE);
//	encQuad ENC0, ENC1;
//	/// imposta gli indirizzi dei due moduli
//	ENC0.setAddr(QEI0_BASE);
//	ENC1.setAddr(QEI1_BASE);
//	/// imposta i coefficienti di conversione letture->mm
//	ENC0.setCoeff(0.225);
//	ENC1.setCoeff(0.231);

	maze M;
	cell C(1,2);
	M.carica(C);

	//volatile double d = 1.9845637456;
	//gyro G;
	Jitter JIT;
	/// MODULO INERZIALE  ///
	Giroscopio Rot;
	//accelerazione A;
	accelerometro A;
	//cinematica CIN;
	/// servono differenti PID, almeno uno per la rotazione ed uno per lo spostamento
	/// per la rotazione sarebbero interessante usarne 2, uno per la rotazione soft ed uno per la rotazione
	/// brusca.
	digPID cPid[3];
	cPid[0].setupPID(1);
	cPid[1].setupPID(2);
	cPid[2].setupPID(3);
	/// descrittore della sintassi dei comandi
	syn_stat synSTATO;
	/// modulo zigbee per telemetria
	//xbee XB;
	xBee XB;

	/// MODULO SENSORE DI COLORE  ///
	COLORE CL;
	/// PIASTRELLA
	TILE PST[50];


	/// MODULO SENSORE DI TEMPERATURA (PIROMETRO) ///
	TEMPER sensIR;
	/// informazioni sul sopravvissuto
	survivor SUR;

	/// oggetto che riallinea il mezzo
	allineamento AL;

	/// disabilita le interruzioni
	DI();
	//pidPtr = CTRL;
	//dPtr = &DIST;

	/// questa struttura raccoglie il comando da eseguire e interagisce con il relativo PID
	comando CMD1;
	/// i motori vanno inizializzati da queste parti, altrimenti l'impostazione delle periferiche precoce, prima che l'unita'
	/// sia avviata provoca un fault e si entra nella interruzione di servizio per periferica non pronta.
	PWM_MOTORI M0, M1;
	//DATA.distPtr = &DIST;
	//passaggio degli indirizzi delle strutture alla struttura generale
	//dati_a_struttura(&G, &DIST, &CIN, &COL, &TEMP, &SUR, &DATA);

	/// l'utilita' e' che puo' essere usato come unico puntatore per distribuire le strutture nei vari punti
	/// del programma.
	ALLSTRUCT allDATA;

	allDATA.setup(&A, &ENC0, &ENC1, &sensIR, &CL, &PST[0], &SUR, &MISURE, &Rot);

	/// setup di base
	setupMCU();
	/// imposta i parametri del PID
	//setupPID(CTRL);
	/// MODULO PWM PER MOTORI DI SPOSTAMENTO ///
	//

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


	/// INIZIALIZZAZIONI MODULI E COLLEGAMENTI AI CANALI DI COMUNICAZIONE

	/// inizializza il giroscopio con banda a 190Hz invece cha a 95Hz
	Rot.attachI2C(&BUS_COMM[0], GYRO_ADDR);
	Rot.initGyro(ODR_190 | Z_AXIS);
	/// collegato sensore di temperatura al bus I2C
	sensIR.attachI2C(&BUS_COMM[1], TEMP_ADDR);
	sensIR.taraturaTemp();

	tick10 = tick100 = 0;
	/// inizializza il timer 0 e genera un tick da 10 ms
	initTimer0(INT_STEP_10_MS);
	PRINTF("inizializzato TIMER0\n");
	/// inizializza il timer 2 a contare con prescaler ogni 100us
	initTimer2(_100_US);
	PRINTF("inizializzato TIMER2\n");
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
	/// inizializzati i moduli encoder
	ENC0.qeiInit();
	ENC1.qeiInit();
	//servo = (pwm *) &pwmServi;
	/// inizializzazione accelerometro
	A.attach(&BUS_COMM[2], ACCEL_ADDR);
	A.testAccel();
	if (A.isPresent == true)
		/// imposta l'accelerometro
		A.impostaAccel();

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
	//qei_test(&QEI);
	/// task principale
	int tempCont = 0;

	//KIT.Init();
	//MOT_SENS.Init();
	///
	/// imposta il perno dello sgancio delmattoncino nella posizione di blocco
	//KIT.MotorGo(75);
	///
	/// impsta il motore di rotazione del sensore di temperatura in posizione di zero.
	//MOT_SENS.MotorGo(0);

	CL.Init();
	/// calibra il bianco del sensore di colore
	CL.WhiteBalance();
	//initLightSens1();
	//whiteBal(&COL);
	//XB.test();
	int dir = 1, gradi = 0;

	/////////////////////////////////////////////////////////
	///
	///      TASK PRINCIPALE
	///
	/////////////////////////////////////////////////////////
	//setupUART(1);
	//XB.sendString("Ciao\n", 5);
	//PRINTF("Telemetria\n");


	//***********************************
	/** primo campionamento **/
	ADCProcessorTrigger(ADC0_BASE, 0);
	//***********************************

	///
	/// CICLO PRINCIPALE
	///
	while(1){


/************************************************************/
/*  			ATTIVITA' SVOLTE AD OGNI CICLO				*/
/************************************************************/

		/// ESEGUIT AD OGNI CICLO
		// controllo di messaggio sulla seriale 1 (ricevuto comando da rasp)
		if (READ_PTR1 != RX_PTR1){
			/// analizza il comando e imposta il valore dell'oggetto CMD (comando)
			/// ATTENZIONE: synStat NON E' ANCORA USATO
			 parse(&synSTATO, &CMD1, &synStat);
			 /// aggiorna il buffer
			 READ_PTR1++;
			 READ_PTR1 &= DIM_READ_BUFF - 1;
		}
		if (synSTATO.valid == VALIDO && synSTATO.token != ERRORE){
			/// il comandoche e' stato analizzato ha prodotto un risultato adeguato
			rispondiComando(&synSTATO, &allDATA);
			/// avendo terminato la risposta, la validit� dell'automa
			/// va rimossa e viene quindi resettato
			resetAutoma(&synSTATO);
			//PRINTF("comando ricevuto: %c\n", synSTATO.cmd[0]);
		}

		/*********************/
		/* AZIONI CADENZATE  */
		/*********************/

		/////////////////////////////////////
		/// AZIONI DA COMPIERE OGNI 10 ms ///
		/// aggiorna il PID ogni tick del timer che sono 10ms
		if (procCom == 1 ){
			//UARTCharPutNonBlocking(UART1_BASE, 'c');
			procCom = 0;
			contatore++;
			//millis10++;
			//lampeggio_led++;
			if (Rot.IsPresent == OK){
				/// aggiorna l'angolo di yaw
				Rot.misuraAngoli(&JIT);
			}

			/// e' eseguito il movimento sulla classe comando
			/// viene richiesto il pid di riferimento, lo stato del comando (in modo da continuare se il comando e' valido),
			/// i  pwm per i motori, il valore degli encoder e del giroscopio.
			CMD1.RUN(cPid, &synSTATO, &M0, &M1, &ENC0, &ENC1, &Rot, &JIT);
			/// le misure del giroscopio invece sono effettuate solo dall'apposito pid
			/// PF0 in modalita' blink, per misurare se il tempo sono realmente 10ms
			HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_0 << 2))) ^=  GPIO_PIN_0;
		}
		/// effettua i calcoli solo se il giroscopio e' presente
		/// TODO: il PID viene calcolato ongi 10ms oppure ogni 20ms? Come � meglio?

		/////////////////////////////////////
		/// AZIONI DA COMPIERE OGNI 100ms ///
		if (tick10 >= 10){
			tick10 = 0;
			/// legge la posizione segnalata dagli endcoder
			ENC0.readPos();
			ENC1.readPos();
		}

		/// TODO: 20 ms o 10 ms?
		if (tick20 >= 20){
			tick20 = 0;
			if (ADCDataReadyFlag == 1){
			/// c'e' un dato campionato pronto, ad esempio la batteria, e viene copiato
			ADCDataReadyFlag = 0;
			/// converte la misure grezza, letta dalla routine di interruzione in mm
			/// la lettura del dato sei sensori e' esattamente questo dato.
			MISURE.rawTomm();
			/// memorizza anche il livello della batteria della logica
			BATT.battLevel = MISURE.dI[5];

			//***********************************
			/** (RI)AVVIA IL CAMPIONAMENTO DI ADC **/
			ADCProcessorTrigger(ADC0_BASE, 0);
			//***********************************
			}
		}


		//////////////////////////////////
		/// AZIONI DA COMPIERE OGNI 1s ///

		if (tick100 >= 100){

#ifdef _DEBUG_
			PRINTF("Liv batteria: %d\n", BATT.battLevel);

#endif
#ifdef _DEBUG_
            PRINTF("asse z: %d\n", Rot.yaw);
#endif

#ifdef _ROME_DEB_
			/// legge il valore di tensione del sensore di gas
			/// prova per romecup2018
			PRINTF("Sens. gas %d\n", MISURE.dI[7]);
			PRINTF("Nota 4khz %d\n", MISURE.dI[6]);
#endif


#ifdef _DEBUG_ENC_
			M0.delta = M1.delta = 50;
			M0.direction = 1;
			M1.direction = 1;
			M0.MotorGo();
			M1.MotorGo();
			//M0.MotorStop();
			//M1.MotorStop();
			//for (volatile int i = 5000000; i > 0; i--);
			ENC0.readPos();
			ENC1.readPos();
			PRINTF("\n");
			PRINTF("POS ENC0: %d\t%d\t", ENC0.dist_mm, ENC0.readDir());
			PRINTF("POS ENC1: %d\t%d\n", ENC1.dist_mm, ENC1.readDir());
			PRINTF("\n");
#endif

			uint32_t micros = TimerValueGet(WTIMER2_BASE, TIMER_A);
			mJ = misuraJitter - micros;
			mJ /= 100;
#ifdef _DEBUG_JITTER
			PRINTF("micros: %u delta (0.1 ms) %u\t", micros, mJ);
#endif
			misuraJitter = micros;

	//		PRINTF("ang_rot %d \t", Rot.yaw);
			printFloat(Rot.tick, 4);
			//PRINTF("\t");
			//printFloat(Rot.corr, 7);
			A.misuraAccelerazioni();
//			PRINTF("\tAz: %d\n", A.aInt[2]);
			//PRINTF("\n");

			/// controlla il colore della piastrella sottostante e lo paragona la bianco memorizzato in fase di setup
			/// bisogna anche impostare il numero della piastrella e le sue coordinate.
			CL.Run(&PST[0]);
#ifdef _DEBUG_COL
			/// il colore NERO e' circa il valore del BIANCO diviso 2,6 - 2,7
			PRINTF("Col: %d\t W: %d\n", CL.get(), CL.getWhite());
#endif
			/// legge la temperatura del pirometro
			if(sensIR.readTemp() > sensIR.T_tar + 10.0)
				/// ha torvato un ferito
				sensIR.isSurvivor = IS_SURVIVOR;
			else
				sensIR.isSurvivor = NO_SURVIVOR;
			/// TODO controllare se riesce a funzionare mentre legge le accelerazioni su I2C
			/// avvia il campionamento degli ADC. I dati vengono posti nell'oggetto MISURE dalla routine di servizio
			/// dell'interruzione AD.
			/// Ricordarsi: il dato n.6 e'lo stato della batteria
			/*		   ***			*/
			/// TODO *** ATTENZIONE VERIFICARE SE PUO' ESSERE MESSO NEL CICLO DA 100ms oppure se pu� essere messo il solo
			///      *** START OF COnvERSION */
			if (ADCDataReadyFlag == 1){
				/// c'e' un dato campionato pronto, ad esempio la batteria, e viene copiato
				ADCDataReadyFlag = 0;
				/// converte la misure grezza, letta dalla routine di interruzione in mm
				/// la lettura del dato sei sensori e' esattamente questo dato.
				MISURE.rawTomm();
				/// memorizza anche il livello della batteria della logica
				BATT.battLevel = MISURE.dI[5];

#ifdef _DEBUG_
				PRINTF("Liv batteria: %d\n", BATT.battLevel);
				/// legge il valore di tensione del sensore di gas
				/// prova per romecup2018
				PRINTF("Sens. gas %d\t", MISURE.dI[7]);
				PRINTF("Nota 4khz %d\n", MISURE.dI[6]);


#endif
				//***********************************
				/** (RI)AVVIA IL CAMPIONAMENTO DI ADC **/
				ADCProcessorTrigger(ADC0_BASE, 0);
				//***********************************
			}

			//HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) |=  GPIO_PIN_5;
			if (BATT.battLevel > BATT.safeLevel){
				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) ^=  GREEN_LED;
				/// si assicura che il led rosso sia spento
				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_1 << 2))) &=  ~GPIO_PIN_1;
			}

			else{
				/// segnala che la batteria sta finendo, facendo lampeggiare il rosso
				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_1 << 2))) ^=  GPIO_PIN_1;
				/// spegne il led verde
				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3 << 2))) &=  ~GREEN_LED;
			}

			////
			//// VIENE ESEGUITA QUANDO IL COMANDO E' RILASCIO KIT (comando 'P' da raspberry)
			///  E QUANDO il COMANDO e' SULLO STATO AVVIA. RILASCIATO IL PACK il COMANDO
			///  PONE avvia = false e NON lo RIESEGUIRA' pi� finch� token sar� di nuovo
			///  RILASCIO_PACK e CMD con avvia = true. QUESTO ACCADE IN convertToToken,
			///  nel file parse.cpp
//#ifdef _DEBUG_
//			if (CMD1.avvia == true && synSTATO.token == RILASCIO_PACK){
//				/// rilascio del kit
//				KIT.scarico();
//				CMD1.avvia = false;
//				CMD1.isRun = false;
//			}
//#endif

			/// MISURA IL SENSORE DI ACCELERAZIONE
			A.misuraAccelerazioni();
/// stampe dei valori dei sensori di distanza.
#ifdef _DEBUG_
			for(int i = 0; i < 5; i++){

				PRINTF("val%d: %d \t", i, MISURE.dI[i]);
			}
			PRINTF("\n");
//
#endif

/// stampe le letture degli encoder
#ifdef _DEBUG_

			PRINTF("POS ENC0: %d\t%d\t", ENC0.dist_mm, ENC0.readDir());
			PRINTF("POS ENC1: %d\t%d\n", ENC1.dist_mm, ENC1.readDir());
//
#endif
			PRINTF("#######\n\n");

#ifndef _DEBUG_
//				/// ricopia nella struttare DIST:
			for(int attesa = 0; attesa < 5; attesa++){
//					if (attesa == 3)
//						continue;
				PRINTF("mm(%d): %d \t", attesa, MISURE.d_mm[attesa]);
			}
#endif

#ifndef _DEBUG_

			contatore = 0;
			PRINTF("%d\tasse z: %d\t",tempCont++, Rot.yaw);
			printFloat(Rot.yawF, 4);
			PRINTF("\t");
			printFloat(Rot.yawF0, 4);
//					if (A.isPresent == true){
//						PRINTF("\t");
//						A.misuraAccelerazioni();
//					}
			PRINTF("\n");

#endif

			/// aggiornamento delle strutture dati del labirinto
			// todo da aggiungere
			//M.run(allDATA);
			//// reset del contatore
			tick100 = 0;

		} /// fine delle azioni da compiere ogni 1s

	}
}


