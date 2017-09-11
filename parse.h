/*
 * parse.h
 *
 *  Created on: 07/mar/2016
 *      Author: itismattei
 */

#ifndef PARSE_H_
#define PARSE_H_

#include <stdint.h>
#include "pid.h"
#include "adc/adc.h"
#include "sens_col_ir/sens.h"
#include "qei.h"
#include "distMis.h"
#include "sens_col_ir/sens1.h"

#include "Giroscopio.h"
#include "encQuad.h"
#include "accel/accel.h"

//typedef struct _glb{
//	//gyro 		*gPtr;
//	TEMPER			*temperat;
//	temperatura 	*tempPtr;
//	colore 			*colPtr;
//	COLORE			*colorClass;
//	//distanza 	*distPtr;
//	cinematica 		*cinPtr;
//	qei 			*qeiPtr;
//	survivor 		*surPtr;
//
//	/* classe */
//	distMis 		*DSTptr;
//	Giroscopio  	*gyro;
//	encQuad			*encoder;
//	accelerometro 	*acc;
//} glb;

/////
/// questa classe ha lo scopo di raccogliere le varie strutture dati create nello sviluppo del software
/// in un'unica organizzazione e rendere quindi più accessibile l'uso dei dati in qualunque parte del main.
////
class ALLSTRUCT{
public:
	ALLSTRUCT(){}
	void DefineGeneric(void * valorePtr){generic = valorePtr;}

	void setup(accelerometro *ACC, encQuad *ENC, TEMPER *TEMP, COLORE *color, TILE * t, survivor *SUR, distMis *DIS, Giroscopio *GYRO,
			void* undefined){
		surPtr 	= SUR;

		cinPtr 		= 0;
		//GLB->colPtr 	= COL;
		//GLB->distPtr 	= D;
		colorClass 	= color;
		tilePtr 	= t;
		temperat 	= TEMP;
		DSTptr 		= DIS;
		gyro		= GYRO;
		encoder		= ENC;
		acc			= ACC;
		generic 	= undefined;

	}

	///membri
	TEMPER			*temperat;
	temperatura 	*tempPtr;
	//colore 			*colPtr;
	COLORE			*colorClass;
	TILE			*tilePtr;
	//distanza 	*distPtr;
	cinematica 		*cinPtr;
	qei 			*qeiPtr;
	survivor 		*surPtr;

	/* classe */
	distMis 		*DSTptr;
	Giroscopio  	*gyro;
	encQuad			*encoder;
	accelerometro 	*acc;
	void*			generic;
};

/*
 * ///
/// struttura che analizza la frase e ne mantiene il token del comando
typedef struct _syn_stat{
	int ST;
	uint8_t 	cmd[4];			/// lunghezza del comando fissa: 4 bytes
	uint8_t 	l_cmd;			/// lunghezza in bytes del comando
	uint8_t		check;			/// checksum del comando
	uint8_t 	token;			/// valore numerico del comando
	uint8_t 	tick;			/// tempo di persistenza del comando
	uint16_t	valid;			/// stato di validita' del comando
	uint8_t		buff_reply[8];	/// buffer della risposta
	uint8_t		dato_valido;	/// validita' della risposta del sensore
	uint8_t		suspend_reply;	/// usata per i comandi che necessitano di risposta alla fine
} syn_stat;
 * */

class syntaxStatus{
public:
	syntaxStatus(){valore = 0; tick = 0;}

	int valore;
	int ST;
	uint8_t 	cmd[4];			/// lunghezza del comando fissa: 4 bytes
	uint8_t 	l_cmd;			/// lunghezza in bytes del comando
	uint8_t		check;			/// checksum del comando
	uint8_t 	token;			/// valore numerico del comando
	uint8_t 	tick;			/// tempo di persistenza del comando
	uint16_t	valid;			/// stato di validita' del comando
	uint8_t		buff_reply[8];	/// buffer della risposta
	uint8_t		dato_valido;	/// validita' della risposta del sensore
	uint8_t		suspend_reply;	/// usata per i comandi che necessitano di risposta alla fine
};

///
/// classe che gestisce l'analisi sintattica semplificato, molto, dei comandi
class Parse{
public:
	Parse(){}

	/// analizza lo stato del messaggio e carica i risultati nell sintassi, poi agisce sui comando del pid
	void parse(PIDtoPWM *, syntaxStatus *);
	/// trasforma un comando in un token
	void convertToToken(syntaxStatus *, PIDtoPWM * );

	/// proprieta'
	syntaxStatus * syntStat;
};

#ifdef __cplusplus
extern "C" {
#endif

void parse(syn_stat *,  PIDtoPWM *, syntaxStatus *);
void convertToToken(syn_stat *, PIDtoPWM * );
/// reset dell'automa (o inizializzazione)
void resetAutoma(syn_stat * STATO);
void initModule();


void sendReply(syn_stat *, uint8_t numChar);

#ifdef __cplusplus
}
#endif

void rispondiComando(syn_stat *sSTAT, ALLSTRUCT *);
/// invia la lettura di un sensore
void inviaSensore(syn_stat *,  ALLSTRUCT*);

void dati_a_struttura(gyro *, distanza *, cinematica *, colore *, temperatura* ,survivor *, dati *);
//void datiRaccolti(accelerometro *ACC, encQuad *ENC, TEMPER *TEMP, COLORE *CL, survivor *SUR, distMis *DIS, Giroscopio *GYRO, glb *GLB );

//pid * leggiComando(syn_stat *sSTAT, pid CTRL[], pid *p, dati *);
void EseguiPID(syn_stat *sSTAT, digPID *);

#endif /* PARSE_H_ */
