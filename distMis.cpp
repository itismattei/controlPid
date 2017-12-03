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

	//   NUMERAZIONE DEI SENSORI DI DISTANZA
	//          1
	//       +--!--+
	//     1 <     > 3
	//       |     |
	//     2 <     > 4
	//


	/*
	 * f: inverso della distanza
	 * V: tensione di ingresso
	 * dI: tensione convertita in digitale V / 3.3 * 4095
	 * f = k * dI + q
	 * distanza = 1/f
	 *
	 * k1 = (0,06935 - 0,03287)/(0,925 - 0,43) =  (0,06935 - 0,03287)/(1144 - 532) = 59.61e-6
	 * q1 = f - k1 * dI = 0,925 - k1 * 1144 = 1.159e-3
	 *
	 * k2 = (0,2 - 0,142)/(2,35 - 1,78) =  (0,2 - 0,142)/(2916 - 2209) = 8.204e-5
	 * q2 = f - k2 * dI = 0,142 - k2 * 2209 = -3.92e-2
	 *
	 * k3 = (0,2857 - 0,2)/(3 - 2,35) =  (0,2857 - 0,2)/(3723 - 2916) = 1.067e-4
	 * q3 = f - k3 * dI = 0,2 - k3 * 2916 = -0.1111372
	 * */


	/// conversione del dato grezzo in cm
	for (int i = 0; i < 5; i++){

		d_mm[i] = dI[i];
	}
}


