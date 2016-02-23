/*
 * distMis.cpp
 *
 *  Created on: 21/feb/2016
 *      Author: massimo
 */

#include "distMis.h"

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

	/// per adesso facciamo un amisura approssimata:
	/// 3V = 3800 -> 1/3= 0,33cm 0.3V = 380 -> 1/30cm = 0,033
	/// 1/d = (0.3 - 0,033) / (3800 - 380)

	/*
	 * f: inverso della distanza
	 * V: tensione di ingresso
	 * dI: tensione convertita in digitale V / 3.3 * 4095
	 * f = k * dI + q
	 * distanza = 1/f
	 *
	 * k1 = (0,142 - 0,025)/(1,78-0,35) =  (0,142 - 0,025)/(2209 - 434)
	 * q1 = f - k1 * dI = 0,142 - k1 * 2209 = 0,142 - 6.592e-5 * 2209 = -3.617e-3
	 *
	 * k2 = (0,2 - 0,142)/(2,35 - 1,78) =  (0,2 - 0,142)/(2916 - 2209) = 8.204e-5
	 * q2 = f - k2 * dI = 0,142 - k2 * 2209 = -3.92e-2
	 *
	 * k3 = (0,2857 - 0,2)/(3 - 2,35) =  (0,2857 - 0,2)/(3723 - 2916) = 1.067e-4
	 * q3 = f - k3 * dI = 0,2 - k3 * 2916 = -0.1111372
	 * */

	float f, d, k = 0.87 * (0.2857 - 0.03125)/(3723.0 - 496.0);
	float k1, k2, k3, q1, q2, q3;
	k1 = 6.592e-5;
	q1 = -3.617e-3;
	k2 = 8.204e-5;
	q2 = -3.92e-2;
	k3 = 1.067e-4;
	q3 = -0.1111372;
	/// conversione del dato grezzo in cm
	for (int i = 0; i < 6; i++){
		/// distnza superiori a 40cm non vengono calcolate
		if (dI[i] < 400)
			dI[i] = 400;
		if (dI[i] < 2210){
			/// si usa la prima retta interpolante
			f = k1 * dI[i] + q1;
		}
		else if (dI[i] < 2916){
			/// intervallo 5 - 7 cm
			f = k2 * dI[i] + q2;
			}
		else if (dI[i] < 3723){
			/// intervallo 5 - 3.5 cm
			f = k3 * dI[i] + q3;
		}

		if (dI[i] > 3723){
			/// valori minori di 3.5cm non hanno senso
			dI[i] = 3723;
			f =  k3 * 3723.0 + q3;
		}

		/// la distanza (in cm) e' 1 / f
		d = 1 / f * 10;			/// distanza in mm

		d_mm[i] = (int) d;
	}

//	for (int i = 0; i < 6; i++){
//		if (dI[i] < 400)
//			dI[i] = 400;
//		f = k * (float) dI[i];
//		d = 1/f * 10;		/// risultato in millimetri
//		d_mm[i] = (int) d;
//
//	}

}
