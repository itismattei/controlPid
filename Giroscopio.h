/*
 * Giroscopio.h
 *
 *  Created on: 02/mar/2016
 *      Author: massimo
 */

#ifndef GIROSCOPIO_H_
#define GIROSCOPIO_H_

#include <stdint.h>

class Giroscopio {
public:
	Giroscopio();
	virtual ~Giroscopio();

	void initGyro(char);
	void setupAssi(char stato);
	void misuraAngoli();
	void azzeraAssi(void);
	void printAsseZ(int);

	int16_t buffValori[512];
	int16_t buffX[512];
	float  media;
	float m, q;

//private:
	char IsPresent;
	char IsOn;
	/// tiene il setup dell'asse
	int16_t x0, y0, z0;
	/// indica quali assi sono attivi
	char asseOn;
	/// valore degli angoli misurati da integrazione della rotazione
	/// i valori sono in gradi in modo da non dover memorizzare dei float
	int16_t yaw; 	/// azze z
	int16_t roll; 	/// asse y
	int16_t pitch;	/// asse x

	/// valori attuali degli assi
	float yawF;
	float rollF;
	float pitchF;
	/// valori precedenti degli assi
	float yawP, rollP, pitchP;
	/// valori di gradi al secondo dedotti dal manuale del giroscopio
	int16_t gradiSec;
	float FS;
	/// intervallo di integrazione in ms
	float tick;
	/// fattori di scala per ciscun asse
	float kz, ky, kx;
};

#endif /* GIROSCOPIO_H_ */
