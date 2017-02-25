/*
 * init.c
 *
 *  Created on: 22/mar/2015
 *      Author: robotics
 */

#include "init.h"
#include "distMis.h"
#include "Giroscopio.h"
#include "parse.h"
#include "sens_col_ir/sens1.h"



void initModule(){
	;
}

void dati_a_struttura(gyro *G, distanza *D, cinematica *CIN, colore *COL, temperatura *TEMP, survivor *SUR, dati *DATA)
{
	DATA->surPtr = SUR;
	DATA->gPtr = G;
	//DATA->cinPtr = CIN;
	DATA->colPtr = COL;
	//DATA->distPtr = D;
	DATA->tempPtr = TEMP;
}


//void datiRaccolti(accelerometro *ACC, encQuad *ENC, TEMPER *TEMP, COLORE * color, survivor *SUR, distMis *DIS, Giroscopio *GYRO, glb *GLB ){
//
//	GLB->surPtr 	= SUR;
//
//	GLB->cinPtr 	= 0;
//	//GLB->colPtr 	= COL;
//	//GLB->distPtr 	= D;
//	GLB->colorClass = color;
//	GLB->temperat 	= TEMP;
//	GLB->DSTptr 	= DIS;
//	GLB->gyro		= GYRO;
//	GLB->encoder	= ENC;
//	GLB->acc		= ACC;
//}
