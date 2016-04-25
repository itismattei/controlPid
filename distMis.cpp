/*
 * distMis.cpp
 *
 *  Created on: 21/feb/2016
 *      Author: massimo
 *
 *      modifica: giulio 21/apr/2016
			(prova approx in rawTomm con dist = k*pow(sens,m)
 */

/// in questa revisione sistemo le misure di distanza secondo quanto previsto dal protocollo di comunicazione.


#include "distMis.h"
#include "math.h"

distMis::distMis() {
	// TODO Auto-generated constructor stub
	for (int i = 0; i < 6; i++){
		misSens[i] = 0.0;
		d_mm[i] = 0;
		dI[i] = 0;
		run = false;
		kf = 1.0;
	}

}

distMis::~distMis() {
	// TODO Auto-generated destructor stub
}


///
/// estrae dal dato grezzo la misura
void distMis::rawTomm(){
	// implementazione sintetica, con uso di powf.

	//valori "esatti" solo per i primi quattro sensori
	//ricavati per regress lineare su logaritmi delle misure
	//di Bricca e compagni 22 aprile 2016
	//
	//          3
	//       +--!--+
	//     2 <     > ?
	//       |     |
	//     1 <     > 0
	//
	float k[5]={19389.0,   24330.0,   21930.0,  305162.2871,     19389.0};
	float m[5]={-1.0331,   -1.0742,   -1.05414,   -1.201951,    -1.05414};
    float d;
	/// conversione del dato grezzo
	// per ogni sensore i:
	for (int i = 0; i < 5; i++){


		 //formula:
		 //  ln(y) = m ln(x) + q   (x: dato grezzo dI, y: dist in cm).
		 // quindi y = e^(m ln(x) + q)
		 // quindi y = e^q * x^m
		 //    pongo e^q = k:
		 // y = k*pow(x,m)


		d = dI[i];
		if( d > 3600) d = 3600.0;
		if( d < 650 ) d = 650.0;
		d_mm[i] = (int) ((k[i]*powf(d,m[i]))*10.0);
	}
}
