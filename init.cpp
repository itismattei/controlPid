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

/// date:	09-04-2018
/// Release del software
volatile uint8_t vers[] = "1.1 <- oratorio\0";

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

#include <iomanip> // setprecision
#include <sstream> // stringstream

using namespace std;
///
/// converte un float o un double a vettore di uint8_t
//void float2string(float f, uint8_t valore[]){
//	stringstream s1;
//	string txt;
//	/// carica il float nello stream
//	s1 << f;
//	/// converte lo stream in stringa
//	txt = s1.str();
//	uint8_t i;
//	for (i = 0; i < txt.length(); i++){
//		valore[i] = txt[i];
//	}
//	/// inserisco il terminatore di stringa
//	valore[i] = '\0';

//}
