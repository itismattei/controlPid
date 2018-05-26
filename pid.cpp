/*
 * pid.c
 *
 *  Created on: 05/feb/2015
 *  PID
 *  Author: robocupjr 15
 */

#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "pwm\pwm.h"
#include "init.h"		///serve per le costanti dei token
#include "uartp/uartstdio.h"



void rispostaRotazione(/*pid * ,*/ syn_stat *);

/// impostazioni dei PID presenti
void digPID::setupPID(int type){
	///
	e[0] = e[1] = 0.0;
	/// inizializza i coeficienti del pid
	switch(type){
	/// il PID n.1 che e' relativo all'avanzamento, ha come parametro regolato la velocita' del rover
	case 1:
		setKpid(30.0, 0.5, 25.0);
	break;

	default:
		setKpid(1.0, 0.0, 1.0);
	break;

	}
	/// impostazione del tipo di PID
	tipo = type;
}

///
/// imposta i coefficienti del PID su valori standard
void digPID::setKpid(float p, float d, float i){
	kp[0] = p;
	kd[0] = d;
	ki[0] = i;
	/// imposta anche i valori inziali della derivata ed integrale
	I1 = 0.0;
	d = 0.0;
}

///
/// effettua l'integrazione numerica
void digPID::calcola(float tick, Jitter *J){

	float D, P, I, corr = 0.0;
	///
	/// qui effettua la correzione sulla jitter dell'intervallo di integrazione
	///
	int32_t diff;
	J->setActualPID();
	if (J->prevValuePID != 0){
		/// effettua l'aggiornamento
		diff = J->prevValuePID - J->jitter_timerPID;
		if (diff < 0)
			diff = -diff;
		/// se sono passati esattamente 10ms allora diff = 100, altrimenti la differenza sara' il jitter;
		/// supponendo che diff - 100 = 1, allora corr = 1e-4 => 100 us
		corr = (float)(diff - 100) / 10000.0;
		tick += corr;
	}
	J->prevValueGyro = J->jitter_timerGyro;

	/// derivativo
	D = kd[0] * (e[1] - e[0]) / tick;
	/// proporzionale
	P = kp[0] * e[1];
	/// integrale (trapezoidale)
	I = I1 + ki[0] * tick * (e[1] + e[0]);
	I *= 0.50;
	I1 = I;
	uscita = D + P + I;
	/// dispositivo con saturazione
	if (uscita > 100.0)
		uscita = 100.0;
		else if (uscita < -100.0)
			uscita = -100.0;
	//aggiornamento dell'errore
	e[0] = e[1];
}


///
/// funzione che legge il sensore e calcola il nuovo valore
/// dell'errore dopo l'azione del PID
/// il PID deve distinguere tra rotazione e movimento lineare e
/// per questo riceve un vettore di struct di tipo PID
int digPID::Run(Giroscopio *G, PWM_MOTORI *PWM1, PWM_MOTORI * PWM2, distMis *DISTANZA, encQuad * ENC1, encQuad * ENC2){

	float soglia = 0.05;
	/// controllare se arriva un puntatore nullo per il pid, generato da una condizione di time out
	/// si ricorda che l'errore nel comando  non annulla un comando in esecuzione.
//	if (C == NULL) {
//		C->attivo = false;
//		(C + 1)->attivo= false;
//		(C + 2)->attivo= false;
//		/// stop al pwm
//		PWM->delta_1 = PWM->delta_2 = 0;
//		pwm_power(PWM);
//		/// spento il PWM esce con codice di errore
//		return -1;
//	}


	return 0;
}

///


///
/// COSTRUTTORE CLASSE COMANDO
comando::comando(){
	azione = false; isRun = false; finished = false; numPid = -1; token = -1; tick = 0; avvia = 0;
	/// soglia in gradi del raggiungimento del valore finale, durante le rotazioni del mezzo
	sogliaAlfa = 2;
	/// soglia in cm/s del raggiungimento della velocita'
	sogliaVel = 1;
	/// valore finale
	valFin = 0.0;
	/// inizializzazione dei coefficienti degli intervalli di correzione del delta pwm per cingoli che hanno differenti attriti
	cCor[0] = 100;
	cCor[1] = 10;
	cCor[2] = -100;
	cCor[3] = -10;
	distanza = NULL;
}

/// costruttore con parametri
comando::comando(distMis *d){
	comando();
	distanza = d;
}

///
/// impostazione dei puntatori ai trasduttori
void comando::setUptrasducers(Giroscopio *G, pwm *p, distMis *dist){
	gPtr 		= G;
	PWM 		= p;
	distanza 	= dist;
}

volatile static int statoDebugComando = 0;
///
/// esegue il pid selezionato
int comando::RUN(digPID *p, syn_stat *s, PWM_MOTORI *PWM1, PWM_MOTORI *PWM2, encQuad * ENC1, encQuad * ENC2,
		Giroscopio *G, Jitter *J){
	/// controlla il time out del comando e se scaduto si ferma
	/// Siccome il metdo RUN viene chiamato ogni 10ms lo scatto del timeout avviene
	/// dopo 1.5s
	///
//#ifdef _DEBUG_
//	/// forse l'esecuzione del pid poiche' tick < TIMEOUT_CMD
//	if (statoDebugComando == 0){
//		tick = 0;
//		numPid = RUOTA_SINISTRA;
//		valFin = -90.0;
//		s->token = INDIETRO;
//		statoDebugComando = 1;
//	}
//	////
//#endif

	///
	/// CONTROLLO DEL TIMEOUT OPPURE SELEZIONE DEL PID
	/// controlla anche la distaza raw misurata dal sensore 0, quello anteriore.
	bool prossimoOstacolo = false;
//	if (distanza != NULL){
//		/// distanza compresa tra 150 e 200 mm
//		if (distanza->dI[0] > 3300)
//			prossimoOstacolo = true;
//		else
//			prossimoOstacolo = false;
//	}

	if (tick > TIMEOUT_CMD || prossimoOstacolo){
		/// in caso di timeout nella persistenza del comando si deve fermare
		/// il contaotre di timeout (tick) viene resettato in convertToToken(syn_stat *STATO, comando *cmdPtr)
		/// quale era o erano i pid attivo/i?
		s->token = STOP;
		s->valid = NON_VALIDO;

		/// deve anche mettere i pid in stato disattivo (.attivo = false)
		if (numPid >= 0 && numPid < 3)
			(p + numPid)->attivo = false;
		/// se il comando va in timeout, isRun diventa falso
		isRun = false;
		PWM1->MotorStop();
		PWM2->MotorStop();
	}
	else{
		/// agggiorna il contatore di persistenza.
		tick++;
		float soglia = 0.05;
		/// controllare se arriva un puntatore nullo per il pid, generato da una condizione di time out
		/// si ricorda che l'errore nel comando  non annulla un comando in esecuzione.
	//	if (C == NULL) {
	//		C->attivo = false;
	//		(C + 1)->attivo= false;
	//		(C + 2)->attivo= false;
	//		/// stop al pwm
	//		PWM->delta_1 = PWM->delta_2 = 0;
	//		pwm_power(PWM);
	//		/// spento il PWM esce con codice di errore
	//		return -1;
	//	}

		int val;
		/// seleziona il tipo di PID. QUESTA E' LA FORMA SEMPLIFICATA
		/// ATTENZIONE: AVANZA ED INDIETRO HANNO LO STESSO NUMERO DI PID
		if (s->token == INDIETRO){
			/// sicome il pid per avanti ed indietro e' lo stesso, la direzione viene impostata leggendo il token del messaggio
			PWM1->direction = -1;
			PWM2->direction = -1;
		}
		if(s->token == AVANTI){
			PWM1->direction = 1;
			PWM2->direction = 1;
		}
		switch(numPid){
		case AVANZA:
			/// l'avanzamento inizia con un delta = 65 ed arriva fino ad un delta = 80
			if (PWM1->delta <= 50)
				PWM1->delta = 50;
			else
				if (PWM1->delta >= 80)
					PWM1->delta = 80;
				else
					PWM1->delta += 1;

			if (PWM2->delta <= 50)
				PWM2->delta = 50;
			else
				if (PWM2->delta >= 80)
					PWM2->delta = 80;
				else
					PWM2->delta += 1;
//			//provvede a misurare la velocita'
//			//misuraVelocità()
//			float veloc = (ENC1->readVel() - ENC2->readVel()) /  2;
//			p->e[1] = (p->valFin - veloc);
////			/// se l'errore e' minore di una soglia, vuoil dire che e' a regime e
////			/// quindi inutile integrare ulteriormente.
//			if (abs(p->e[1]) > soglia  ){
////				/// calcola l'integrale numerico del PID
//				p->calcola(INT_STEP_10_MS / 1000.0, J);
////				/// avanti oppure indietro
//				if(p->e[1] > 0.0)
//					/// avanti
//					PWM->dir_1 = PWM->dir_2 = 1;
//				else
//					/// indietro
//					PWM->dir_1 = PWM->dir_2 = 2;
////				/// impostazione del PWM ed invio del comando
////				//setXPWM(C, PWM);
//			}
//			else
//				p->attivo = false;

//			setFpwm(PWM1, PWM2, p, numPid);
//			PWM1->delta = 75;
//			PWM2->delta = 75;
			PWM1->MotorGo();
			PWM2->MotorGo();
			/// serve una correzione del PWM qualora si avessero letture differenti degli encoder.
			//correctPwm(ENC1, ENC2, PWM1, PWM2);

		break;

		case RUOTA_DESTRA:
			///provvede ad integrare la misura della velcita' angolare
			/// prestare attenzione al segnale d'errore che poi andra' rimosso
			/// dal PWM perche' i motori, a differenza della regolazione della velocita' dovranno
			/// fermarsi.

			val =  G->yaw - (int)valFin;
			/// calcola il valore assoluto della differenza tra il valore da raggiungere ed i valore attuale
			if (val < 0)
				val = -val;
			/// considera +-2° come angolo raggiunto
			if (val > sogliaAlfa){
				// ruota il rover
				PWM1->delta = PWM2->delta = 80;
				PWM1->direction = 1;
				PWM2->direction = -1;
				PWM1->MotorGo();
				PWM2->MotorGo();
				/// blocca gli aggiornamenti del giroscopio
				G->IsRotating = true;
				PRINTF("Z= %d\n", val);
			}
			else{
				/// termina la rotazione
				PWM1->MotorStop();
				PWM2->MotorStop();
				/// attiv gli aggiornamenti del giroscopio
				G->IsRotating = false;
				/// chiede al giroscopio, appena puo' di effettuare un offset
				G->offsetRequest = 1;
				/// e di effettuare anche un offset ritardato
				G->offsetDelayed = 1;
				/// termina il comando
				s->token = STOP;
				/// ed attiva il timeout
				tick = 151;
			}

//			p->e[1] = (float) (p->valFin - gPtr->yaw);
//			/// calcola l'integrale numerico del PID
//			p->integra(gPtr->tick);


		break;

		case RUOTA_SINISTRA:

			val = G->yaw - valFin;
			/// calcola il valore assoluto della differenza tra il valore da raggiungere ed i valore attuale
			if (val < 0)
				val = -val;
			/// considera +-2° come angolo raggiunto
			if (val > sogliaAlfa){
				G->IsRotating = true;
				// ruota il rover
				PWM1->delta = PWM2->delta = 80;
				PWM1->direction = -1;
				PWM2->direction = 1;
				PWM1->MotorGo();
				PWM2->MotorGo();
			}
			else{
				/// termina la rotazione
				PWM1->MotorStop();
				PWM2->MotorStop();
				G->IsRotating = false;
				/// chiede al giroscopio, appena puo' di effettuare un offset
				G->offsetRequest = 1;
				/// e di effettuare anche un offset ritardato
				G->offsetDelayed = 1;
				/// termina il comando
				s->token = STOP;
				/// ed attiva il timeout
				tick = 151;
			}
		break;

		case ARRESTA:
		default:
			isRun = false;
			PWM1->MotorStop();
			PWM2->MotorStop();;
		break;
		}
	}
	return 0;
}



///
/// coverte l'uscita del pid nel giusto valore del pwm
///
void comando::setFpwm(PWM_MOTORI *pwm1, PWM_MOTORI *pwm2, digPID *p, int  numPid){
	switch(numPid){
	case 1:
		/// qui la curva e' lineare, ma si puo' scegliere anche una differente curva
		pwm1->delta = pwm2->delta = p->uscita * 0.45 + 55.0;

	break;

	case 2:
	case 3:
	default:
	break;
	}

}


///
/// metodo che provvede a correggere il pwm durante l'avanzmanto
void comando::correctPwm(encQuad * ENC1, encQuad * ENC2, PWM_MOTORI *PWM1, PWM_MOTORI *PWM2){

	int diffR = ENC2->readPos() - ENC1->readPos();
	int diff = diffR - diffEnc;
	/// relazione tra incremento del delta e diff tra lettura:
	/// kCor *= (1 + diff * 1e-5) se diff e' in 10-100
	/// kCor *= 1.005 se diff > 100
	/// kCor *= 1.0 se diff in 0 - 10
//	cCor[0] = 100;
//	cCor[1] = 10;
//	cCor[2] = -100;
//	cCor[3] = -10;
	if (diff > cCor[0])
		PWM1->kCor *= 1.005;
	else
		if (diff > cCor[1])
			PWM1->kCor *= (1 + diff * 1e-5);
		else
			if (diff < cCor[2])
				PWM1->kCor *= 0.995;
			else
				if (diff < cCor[3])
					PWM1->kCor *= (1 + diff * 1e-5);
				/// se nessuno dei rami precedenti si e' attivato vuol dire che lo scartamento
				/// e' inferiore alle 10 unità e non viene modificato kCor
	///aggiornamento della differenza per il passo successivo
	diffEnc = diffR;
}
