/*
 * distMis.cpp
 *
 *  Created on: 21/feb/2016
 *      Author: massimo
 */

/// in questa revisione sistemo le misure di distanza secondo quanto previsto dal protocollo di comunicazione.


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
	 * k1 = (0,06935 - 0,03287)/(0,925 - 0,43) =  (0,06935 - 0,03287)/(1144 - 532) = 59.61e-6
	 * q1 = f - k1 * dI = 0,925 - k1 * 1144 = 1.159e-3
	 *
	 * k2 = (0,2 - 0,142)/(2,35 - 1,78) =  (0,2 - 0,142)/(2916 - 2209) = 8.204e-5
	 * q2 = f - k2 * dI = 0,142 - k2 * 2209 = -3.92e-2
	 *
	 * k3 = (0,2857 - 0,2)/(3 - 2,35) =  (0,2857 - 0,2)/(3723 - 2916) = 1.067e-4
	 * q3 = f - k3 * dI = 0,2 - k3 * 2916 = -0.1111372
	 * */

	//float f, d, k = 0.87 * (0.2857 - 0.03125)/(3723.0 - 496.0);
	float f, d;
	float m[4] = {5.60984E-05, 6.55586E-05, 1.07152e-4, 0.0 };
	float q[4] = {0.01611, 0.005897452, -0.08579, 0.0};
//	m[0] = 5.60984E-05;
//	m[1] = 6.55586E-05;
//	m[2] = 1.07152e-4;
//	m[3] = 8.65196E-05;
//	q[0] = 0.01611;
//	q[1] = 0.005897452;
//	q[2] = -0.08579;
//	q[3] = -0.065974265;

	/// conversione del dato grezzo in cm
	for (int i = 0; i < 6; i++){
		/// distnza superiori a 40cm non vengono calcolate
		if (dI[i] < 400)
			dI[i] = 400;
		if (dI[i] < 970){
			/// si usa la prima retta interpolante
			f = m[0] * dI[i] + q[0];
		}
		else if (dI[i] < 2050){
			/// intervallo 5 - 7 cm
			f = m[1] * dI[i] + q[1];
			}
			else if (dI[i] < 3745){
				/// intervallo 5 - 3.5 cm
				f = m[2] * dI[i] + q[2];
				}
//				else if (dI[i] < 3711){
//					f = m[3] * dI[i] + q[3];
//				}

		if (dI[i] >= 3745){
			/// valori minori di 3.5cm non hanno senso
			dI[i] = 3745;
			f =  m[2] * 3711.0 + q[2];
		}

		/// la distanza (in cm) e' 1 / f
		d = 1 / f * 10;			/// distanza in mm

		d_mm[i] = (int) d;
	}
}
