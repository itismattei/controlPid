/*
 * pid.h
 *
 *  Created on: 05/feb/2015
 *  PID
 *  Author: robocupjr 15
 */

#ifndef PID_H_
#define PID_H_

#include "gyro_f.h"
#include "pwm/pwm.h"
#include "pwm/motpwm.h"
#include "adc/adc.h"
#include <stdint.h>
#include "Giroscopio.h"
#include "distMis.h"
#include "init.h"
#include "encQuad.h"

#define			AVANZA			0
#define			RUOTA_DESTRA	1
#define			RUOTA_SINISTRA	2
#define			ARRESTA			-1

class ControlloPID;

///
/// la classe comando serve a trasformare i  dati del PID nei valori di PWM dei motori
class comando{
public:
	comando();

	int RUN(ControlloPID *, syn_stat *,PWM_MOTORI *, PWM_MOTORI *, encQuad * ENC1, encQuad * ENC2, Giroscopio *G);
	void setUptrasducers(Giroscopio	*gPtr, pwm	*PWM, distMis *distanza);
	void setFpwm(PWM_MOTORI *, PWM_MOTORI *, ControlloPID *, int);

	bool azione;			//indica se e' un comando di azione
	bool isRun;				// indica se il comando sta andando
	bool finished;			// indica se il comando e' giunto al termine

	int token;				// numero del comando
	int avvia;				// indica che il comando va eseguito
	uint32_t tick;			// contatore dei cicli di esecuzione del comando

	Giroscopio 	*gPtr;
	pwm			*PWM;
	distMis		*distanza;

	//private:
	int 	numPid;				// numero del PID attivo. VIENE ASSEGNATO DENTRO convertToToken
	int 	sogliaAlfa;			// soglia di angolo raggiunto
	int 	sogliaVel;			// soglia di velocita' raggiunte
	float 	valFin;			// imposta il valore finale a cui deve giungere il comando. Puo' essere una velocita', una rotazione, una distanza,...

};

///
/// la clsse seguente contiene i parametri dello specifico PID. Potrebbe essere utile avere parametri diversi per la rotazione
/// e per il movimento lineare.

class ControlloPID{
public:
	ControlloPID(){;}
	///
	/// impostazione valore finale
	void setPoint(float valore) { valFin = valore; }

	void setupPID(int type);
	void setKpid(float, float, float);
	void calcola(float tick);
	int Run(Giroscopio *G, PWM_MOTORI *PWM1, PWM_MOTORI *PWM2, distMis *distanza, encQuad * ENC1, encQuad * ENC2);

	float 		kp[4];
	float 		kd[4];
	float 		ki[4];
	float 		e[2];		/// errori all'istante attuale e precedente
	float 		I1;			///valore integrale
	float 		d;			/// valore della derivata
	float 		uscita;		/// valore dell'uscita
	uint32_t 	tipo;		/// tipo di movimento: avenzamento, rotazione, rotazione su asse centrale
	int			valFin;		/// valore finale da raggiungere
	bool 		attivo;		/// indica se il pid agisce o e' disattivato.
	uint8_t		rispondi;	/// flag che indica che occorre fornire risposta, per i comandi di tipo rotazione
};


//typedef struct _pid{
//	float 		kp;
//	float 		kd;
//	float 		ki;
//	float 		e[2];		/// errori all'istante attuale e precedente
//	float 		I;			///valore integrale
//	float 		d;			/// valore della derivata
//	float 		uscita;		/// valore dell'uscita
//	uint32_t 	tipo;		/// tipo di movimento: avenzamento, rotazione, rotazione su asse centrale
//	int			valFin;		/// valore finale da raggiungere
//	bool 		attivo;		/// indica se il pid agisce o e' disattivato.
//	uint8_t		rispondi;	/// flag che indica che occorre fornire risposta, per i comandi di tipo rotazione
//} pid;


#ifdef __cplusplus
extern "C" {
#endif

/// calcola l'aggiornamento del PID digital
//int PID(gyro *G, pid * C, pwm *PWM, cinematica *CIN);
/// imposta i coefficienti del PID
//void setKpid(pid *C, float kp, float kd, float ki);
/// impostazione base del PID
//void setupPID(pid C[]);
/// calcola il valore da fornire al PWM
//void setXPWM(pid *C, pwm *P);
///
//void integra(pid *C, float);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
