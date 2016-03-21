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
#include "adc/adc.h"
#include <stdint.h>
#include "Giroscopio.h"
#include "distMis.h"

#define			AVANZA			0
#define			RUOTA			1
#define			RUOTA_SU_ASSE	2

class comando{
public:
	comando(){azione = false; isRun = false; finished = false;}

	bool azione;			//indica se e' un comando di azione
	bool isRun;				// indica se il comando sta andando
	bool finished;			// indica se il comando e' giunto al termine
	int numPid;				// numero del PID attivo
};

class ControlloPID{
public:
	ControlloPID(){;}

	void setupPID(int type);
	void setKpid(float, float, float);
	void integra(float tick);
	int Run(Giroscopio *G, pwm *PWM, distMis *distanza);

	float 		kp;
	float 		kd;
	float 		ki;
	float 		e[2];		/// errori all'istante attuale e precedente
	float 		I1;			///valore integrale
	float 		d;			/// valore della derivata
	float 		uscita;		/// valore dell'uscita
	uint32_t 	tipo;		/// tipo di movimento: avenzamento, rotazione, rotazione su asse centrale
	int			valFin;		/// valore finale da raggiungere
	bool 		attivo;		/// indica se il pid agisce o e' disattivato.
	uint8_t		rispondi;	/// flag che indica che occorre fornire risposta, per i comandi di tipo rotazione
};


typedef struct _pid{
	float 		kp;
	float 		kd;
	float 		ki;
	float 		e[2];		/// errori all'istante attuale e precedente
	float 		I;			///valore integrale
	float 		d;			/// valore della derivata
	float 		uscita;		/// valore dell'uscita
	uint32_t 	tipo;		/// tipo di movimento: avenzamento, rotazione, rotazione su asse centrale
	int			valFin;		/// valore finale da raggiungere
	bool 		attivo;		/// indica se il pid agisce o e' disattivato.
	uint8_t		rispondi;	/// flag che indica che occorre fornire risposta, per i comandi di tipo rotazione
} pid;

#ifdef __cplusplus
extern "C" {
#endif

/// calcola l'aggiornamento del PID digital
int PID(gyro *G, pid * C, pwm *PWM, cinematica *CIN);
/// imposta i coefficienti del PID
void setKpid(pid *C, float kp, float kd, float ki);
/// impostazione base del PID
void setupPID(pid C[]);
/// calcola il valore da fornire al PWM
void setXPWM(pid *C, pwm *P);
///
void integra(pid *C, float);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
