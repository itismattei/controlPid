/*
 * allineamento.c
 *
 *  Created on: 28/mar/2015
 *      Author: Nico
 */


#include "allineamento.h"

allineamento::allineamento(){
	valore 	= 0;

}

allineamento::~allineamento(){
	;
}
//funzione per gestire l'allineamento col corridoio tramite adc
//andrebbe chiamata a intervalli regolari

//N.B. ancora è molto rude...

void allineamento::adc_allinea(distMis &DIST, pwm *PWM /*, pid *C*/)
{
	//variabili
	float delta = 0.0, d[4];
	float angolo_minimo = 5.0; 	//cambiare il valore del'angolo minimo per decidere quando far scattare l'algoritmo di allineamento

	float angolo;	 		//angolo tra la parete e il robot
	float angolo_pid; 		// l'angolo da dare al pid

	angolo = 0.0;
	//leggi il sensore di distanza destro e sinistro

	d[0]= DIST.d_mm[0];
	d[1]= DIST.d_mm[1];

	distAssi = 150.0;
	//determino se la differenza delle letture dei due sensori è significativa
	delta = d[0] - d[1];
	distMDX = (d[0] + d[1]) / 2.0;

	angoloCalc1 = atan(delta / distAssi);
	delta = (DIST.d_mm[4] - DIST.d_mm[5]);
	distMSX = (DIST.d_mm[4] + DIST.d_mm[5]) / 2.0;
	angoloCalc2 = atan( delta / distAssi);

	angolo_pid = (float) - angolo;

	/// adesso vuole capire se ruota sul baricentro o si sposta.
	/// il criterio potrebbe essere: se sono verso il centro ruoto sul baricentro,
	/// se invece sono vicino ad un lato provo a muovermi
	if (distMSX > distMDX)
		dx = true;

	else
		dx = false;

	//se l'angolo è consistente avvio il programma di correzzione della traiettoria
	if(angolo > angolo_minimo){
		///setto il pid dandogli come valore finale l'inverso dell'angolo

//		//preso dal codice pid.c il case RUOTA
//		C->e[1] = (float) (0 - angolo_pid);
//		/// calcola l'integrale numerico del PID
//		integra(C, angolo_pid);
//		PWM->dir_1 = PWM->dir_2 = 1;
//		setXPWM(C, PWM);
	}

 }

//gestisce l'allineamento tramite giroscopio
void allineamento::gyro_allinea()
{

	//leggere il giroscopio
	//se l'angolo è troppo grande avviare la routine di allineamento
	//nella routine di allineamento va dato l'inverso dell'angolo come valore finale del pid





}

///
/// allinea secondo il parametro dx
void allineamento::allinea(void){

}

