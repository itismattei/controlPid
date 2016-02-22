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
	float f, d, k = 0.87 * (0.2857 - 0.03125)/(3723.0 - 496.0);

	for (int i = 0; i < 6; i++){
		if (dI[i] < 400)
			dI[i] = 400;
		f = k * (float) dI[i];
		d = 1/f * 10;		/// risultato in millimetri
		d_mm[i] = (int) d;

	}

}
