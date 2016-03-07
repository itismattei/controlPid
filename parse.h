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




void rispondiComando(syn_stat *sSTAT, dati *, distMis &);
/// invia la lettura di un sensore
void inviaSensore(syn_stat *, dati *, distMis *);

void dati_a_struttura(gyro *, distanza *, cinematica *, colore *, temperatura* ,survivor *, dati *);

pid * leggiComando(syn_stat *sSTAT, pid CTRL[], pid *p, dati *);

#endif /* PARSE_H_ */
