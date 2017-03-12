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
#
#include "init.h"		///serve per le costanti dei token



void rispostaRotazione(/*pid * ,*/ syn_stat *);

/// impostazioni dei PID presenti
void ControlloPID::setupPID(int type){
	///
	/// inizializza i coeficienti del pid
	switch(type){
	case 1:
		setKpid(30.0, 0.5, 25.0);
	break;

	default:
		setKpid(1.0, 0.0, 1.0);
	break;
	/// impostazione del tipo di PID
	tipo = type;
	}
}

///
/// imposta i coefficienti del PID su valori standard
void ControlloPID::setKpid(float p, float d, float i){
	kp[0] = p;
	kd[0] = d;
	ki[0] = i;
	/// imposta anche i valori inziali della derivata ed integrale
	I1 = 0.0;
	d = 0.0;
}

///
/// effettua l'integrazione numerica
void ControlloPID::calcola(float tick){

	float D, P, I;
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
int ControlloPID::Run(Giroscopio *G, PWM_MOTORI *PWM1, PWM_MOTORI * PWM2, distMis *DISTANZA, encQuad * ENC1, encQuad * ENC2){

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
/// imposta il pwm a seguito del valore clacolato dal PID
/// occorre definire la funzione che mappa i valori di uscita su
/// intervalli di funzionamento ammessi dal pwm
/// A 11.4V si va dal 70% al 95%.
//void setXPWM(pid *C, pwm *PWM){
//
//	/// funzioncina lineare al momento
//	/// 10 -> 70; 95 -> 95
//	/// m = (95 - 70)/(95 - 10) = 0,294
//	/// q = 67,07
//	float valore;
//	valore = 0.294 * C->uscita + 67.07;
//	if (C->tipo == AVANZA){
//		PWM->delta_1 = (uint32_t)valore;
//		PWM->delta_2 = (uint32_t)valore;
//	}
//	/// invia i valori al registro del PWM
//	//pwm_power(PWM);
//
//}


///
/// effettua l'integrazione numerica
//void integra(pid *C, float tick){
//
//	float D, P, I;
//	/// derivativo
//	D = C->kd * (C->e[1] - C->e[0]) / tick;
//	/// proporzionale
//	P = C->kp * C->e[1];
//	/// integrale
//	I = C->I + C->ki * tick * (C->e[1] + C->e[0]);
//	I *= (float)0.50;
//	C->I = I;
//	C->uscita = D + P + I;
//	/// dispositivo con saturazione
//	if (C->uscita > 100.0)
//		C->uscita = 100.0;
//		else if (C->uscita < -100.0)
//			C->uscita = -100.0;
//	//aggiornamento dell'errore
//	C->e[0] = C->e[1];
//}

///
/// COSTRUTTORE CLASSE COMANDO
comando::comando(){
	azione = false; isRun = false; finished = false; numPid = -1; token = -1; tick = 0; avvia = 0;
	/// soglia in gradi del raggiungimento
	sogliaAlfa = 2;
	/// soglia in cm/s del raggiungimento della velocita'
	sogliaVel = 1;
}

///
/// impsotazione dei puntatori ai trasduttori
void comando::setUptrasducers(Giroscopio *G, pwm *p, distMis *dist){
	gPtr 		= G;
	PWM 		= p;
	distanza 	= dist;
}

///
/// esegue il pid selezionato
int comando::RUN(ControlloPID *p, syn_stat *s, PWM_MOTORI *PWM1, PWM_MOTORI *PWM2, encQuad * ENC1, encQuad * ENC2, Giroscopio *G){
	/// controlla il time out del comando e se scaduto si ferma
	if (tick > TIMEOUT_CMD){
		/// in caso di timeout nella persistenza del comando si deve fermare
		/// quale era o erano i pid attivo/i?
		s->token = STOP;
		s->valid = NON_VALIDO;

		/// deve anche mettere i pid in stato disattivo (.attivo = false)
		if (numPid > 0 && numPid < 3)
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
//			//provvede a misurare la velocita'
//			//misuraVelocità()
//			p->e[1] = (float) ((float)p->valFin - distanza->vel);
//			/// se l'errore e' minore di una soglia, vuoil dire che e' a regime e
//			/// quindi inutile integrare ulteriormente.
//			if (abs(p->e[1]) > soglia  ){
//				/// calcola l'integrale numerico del PID
//				p->integra(gPtr->tick);
//				/// avanti oppure indietro
//				if(p->e[1] > 0.0)
//					/// avanti
//					PWM->dir_1 = PWM->dir_2 = 1;
//				else
//					/// indietro
//					PWM->dir_1 = PWM->dir_2 = 2;
//				/// impostazione del PWM ed invio del comando
//				//setXPWM(C, PWM);
//			}
//			else
//				p->attivo = false;

			setFpwm(PWM1, PWM2, p, numPid);
			PWM1->MotorGo();
			PWM2->MotorGo();

		break;

		case RUOTA_DESTRA:
			///provvede ad integrare la misura della velcita' angolare
			/// prestare attenzione al segnale d'errore che poi andra' rimosso
			/// dal PWM perche' i motori, a differenza della regolazione della velocita' dovranno
			/// fermarsi.

			val = G->yaw - valFin;
			/// calcola il valore assoluto della differenza tra il valore da raggiungere ed i valore attuale
			if (val < 0)
				val = -val;
			/// considera +-2° come angolo raggiunto
			if (val > sogliaAlfa){
				// ruota il rover
				PWM1->delta = PWM2->delta = 50;
				PWM1->direction = 1;
				PWM2->direction = -1;
				PWM1->MotorGo();
				PWM2->MotorGo();
				/// blocca gli aggiornamenti del giroscopio
				G->IsRotating = true;
			}
			else{
				/// termina la rotazione
				PWM1->MotorStop();
				PWM2->MotorStop();
				/// attiv gli aggiornamenti del giroscopio
				G->IsRotating = false;
				/// chiede al giroscopio, appena puo' di effettuare un offset
				G->offsetRequest = 1;
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
			if (val > 2){
				G->IsRotating = true;
				// ruota il rover
				PWM1->delta = PWM2->delta = 50;
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
/// cpnverte l'uscita del pid nel giusto valore del pwm
///
void comando::setFpwm(PWM_MOTORI *pwm1, PWM_MOTORI *pwm2, ControlloPID *p, int  numPid){
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

