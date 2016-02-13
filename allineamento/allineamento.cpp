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

void allineamento::adc_allinea(distanza *DIST, pwm *PWM, pid *C)
{
	//variabili
	double delta=0, d[2];
	int angolo_minimo=5; //cambiare il valore del'angolo minimo per decidere quando far scattare l'algoritmo di allineamento
	double dist_sens=0; //distanza tra i due sensori nella basetta, è fissa
	double angolo; //angolo tra la parete e il robot
	float angolo_pid; // l'angolo da dare al pid

	//leggi il sensore di distanza destro e sinistro

	d[0]= DIST->d_mm[0];
	d[1]= DIST->d_mm[1];

	//determino se la differenza delle letture dei due sensori è significativa
	delta = d[0] - d[1];

	angolo = atan(delta / dist_sens);
	angolo_pid = (float) - angolo;


	//se l'angolo è consistente avvio il programma di correzzione della traiettoria
	if(angolo > angolo_minimo)
	{
		///setto il pid dandogli come valore finale l'inverso dell'angolo

		//preso dal codice pid.c il case RUOTA
		C->e[1] = (float) (0 - angolo_pid);
		/// calcola l'integrale numerico del PID
		integra(C, angolo_pid);
		PWM->dir_1 = PWM->dir_2 = 1;
		setXPWM(C, PWM);
	}

 }

//gestisce l'allineamento tramite giroscopio
void allineamento::gyro_allinea()
{

	//leggere il giroscopio
	//se l'angolo è troppo grande avviare la routine di allineamento
	//nella routine di allineamento va dato l'inverso dell'angolo come valore finale del pid





}



