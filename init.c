/*
 * init.c
 *
 *  Created on: 22/mar/2015
 *      Author: robotics
 */

#include "init.h"

void initModule(){
	;
}

void dati_a_struttura(gyro *G, distanza *D, cinematica *CIN, colore *COL, temperatura *TEMP, survivor *SUR, dati *DATA)
{
	DATA->surPtr = SUR;
	DATA->gPtr = G;
	DATA->cinPtr = CIN;
	DATA->colPtr = COL;
	DATA->distPtr = D;
	DATA->tempPtr = TEMP;
}


