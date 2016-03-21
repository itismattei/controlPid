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

#include "Giroscopio.h"

typedef struct _glb{
	gyro 		*gPtr;
	temperatura *tempPtr;
	colore 		*colPtr;
	distanza 	*distPtr;
	cinematica 	*cinPtr;
	qei 		*qeiPtr;
	survivor 	*surPtr;
	distMis 	*DSTptr;
	Giroscopio  *gyro;
} glb;

class syntaxStatus{
public:
	syntaxStatus(){valore = 0;}

	int valore;
};

#ifdef __cplusplus
extern "C" {
#endif

void parse(syn_stat *,  comando *);
void convertToToken(syn_stat *, comando *);
/// reset dell'automa (o inizializzazione)
void resetAutoma(syn_stat * STATO);
void initModule();


void sendReply(syn_stat *, uint8_t numChar);

#ifdef __cplusplus
}
#endif

void rispondiComando(syn_stat *sSTAT, glb *);
/// invia la lettura di un sensore
void inviaSensore(syn_stat *,  glb*);

void dati_a_struttura(gyro *, distanza *, cinematica *, colore *, temperatura* ,survivor *, dati *);
void datiRaccolti(distanza *D, cinematica *CIN, colore *COL, temperatura *TEMP, survivor *SUR, distMis *DIS, Giroscopio *GYRO, glb *GLB );

pid * leggiComando(syn_stat *sSTAT, pid CTRL[], pid *p, dati *);

#endif /* PARSE_H_ */
