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
#include "init.h"


void rispostaRotazione(pid *, syn_stat *);

/// impostazioni dei PID presenti
void ControlloPID::setupPID(int type){
	///
	/// inizializza i coeficienti del pid
	int i;
	for (i = 0; i < 3; i++)
		setKpid(1.1, 2.1, 0.5);
	/// impostazione del tipo di PID
	tipo = type;
}

///
/// imposta i coefficienti del PID su valori standard
void ControlloPID::setKpid(float kp, float kd, float ki){
	kp = kp;
	kd = kd;
	ki = ki;
	/// imposta anche i valori inziali della derivata ed integrale
	I1 = 0.0;
	d = 0.0;
}

///
/// effettua l'integrazione numerica
void ControlloPID::integra(float tick){

	float D, P, I;
	/// derivativo
	D = kd * (e[1] - e[0]) / tick;
	/// proporzionale
	P = kp * e[1];
	/// integrale
	I = I1 + ki * tick * (e[1] + e[0]);
	I *= (float)0.50;
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

/// impostazioni dei PID presenti
//void setupPID(pid C[]){
//	///
//	/// inizializza i coeficienti del pid
//	int i;
//	for (i = 0; i < 3; i++)
//		setKpid(&C[i], 1.1, 2.1, 0.5);
//	/// impostazione del tipo di PID
//
//	C[0].tipo = AVANZA;
//	C[1].tipo = RUOTA;
//	C[2].tipo = RUOTA_SU_ASSE;
//}

///
/// imposta i coefficienti del PID su valori standard
//void setKpid(pid *C, float kp, float kd, float ki){
//	C->kp = kp;
//	C->kd = kd;
//	C->ki = ki;
//	/// imposta anche i valori inziali della derivata ed integrale
//	C->I = 0.0;
//	C->d = 0.0;
//}


///
/// funzione che legge il sensore e calcola il nuovo valore
/// dell'errore dopo l'azione del PID
/// il PID deve distinguere tra rotazione e movimento lineare e
/// per questo riceve un vettore di struct di tipo PID
int ControlloPID::Run(Giroscopio *G, pwm *PWM, distMis *DISTANZA){

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

	/// seleziona il tipo di PID
	switch(tipo){
	case AVANZA:
		//provvede a misurare la velocita'
		//misuraVelocità()
		e[1] = (float) ((float)valFin - DISTANZA->vel);
		/// se l'errore e' minore di una soglia, vuoil dire che e' a regime e
		/// quindi inutile integrare ulteriormente.
		if (abs(e[1]) > soglia  ){
			/// calcola l'integrale numerico del PID
			integra(G->tick);
			/// avanti oppure indietro
			if(e[1] > 0.0)
				/// avanti
				PWM->dir_1 = PWM->dir_2 = 1;
			else
				/// indietro
				PWM->dir_1 = PWM->dir_2 = 2;
			/// impostazione del PWM ed invio del comando
			//setXPWM(C, PWM);
		}
		else
			attivo = false;

	break;

	case RUOTA:
		///provvede ad integrare la misura della velcita' angolare
		/// prestare attenzione al segnale d'errore che poi andra' rimosso
		/// dal PWM perche' i motori, a differenza della regolazione della velocita' dovranno
		/// fermarsi.
		G->misuraAngoli();
		e[1] = (float) (valFin - G->yaw);
		/// calcola l'integrale numerico del PID
		integra(G->tick);
		//TODO: adesso si deve mandare il comando al PWM

	break;

	case RUOTA_SU_ASSE:
		G->misuraAngoli();
		e[1] = (float) (valFin - G->yaw);
		/// calcola l'integrale numerico del PID
		integra(G->tick);
		if (e[1] > 0.0){
			///ruota in senso antiorario
			PWM->dir_1 = 1;
			PWM->dir_2 = 2;
		}
		else{
			/// in senso orario
			PWM->dir_1 = 2;
			PWM->dir_2 = 1;
		}
		///
		if(e[1] > -1.0 && e[1] < 1.0){
			/// si può pensare che il comando sia stato eseguito e completato e quindi si puo' comunicare
			/// questo evento.
			rispondi = TRUE;
		}
		/// impostazione del PWM ed invio del comando
		//setXPWM(C, PWM);
	break;
	}
	return 0;
}

///
/// imposta il pwm a seguito del valore clacolato dal PID
/// occorre definire la funzione che mappa i valori di uscita su
/// intervalli di funzionamento ammessi dal pwm
/// A 11.4V si va dal 70% al 95%.
void setXPWM(pid *C, pwm *PWM){

	/// funzioncina lineare al momento
	/// 10 -> 70; 95 -> 95
	/// m = (95 - 70)/(95 - 10) = 0,294
	/// q = 67,07
	float valore;
	valore = 0.294 * C->uscita + 67.07;
	if (C->tipo == AVANZA){
		PWM->delta_1 = (uint32_t)valore;
		PWM->delta_2 = (uint32_t)valore;
	}
	/// invia i valori al registro del PWM
	pwm_power(PWM);

}


///
/// effettua l'integrazione numerica
void integra(pid *C, float tick){

	float D, P, I;
	/// derivativo
	D = C->kd * (C->e[1] - C->e[0]) / tick;
	/// proporzionale
	P = C->kp * C->e[1];
	/// integrale
	I = C->I + C->ki * tick * (C->e[1] + C->e[0]);
	I *= (float)0.50;
	C->I = I;
	C->uscita = D + P + I;
	/// dispositivo con saturazione
	if (C->uscita > 100.0)
		C->uscita = 100.0;
		else if (C->uscita < -100.0)
			C->uscita = -100.0;
	//aggiornamento dell'errore
	C->e[0] = C->e[1];
}
