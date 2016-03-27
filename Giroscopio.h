/*
 * Giroscopio.h
 *
 *  Created on: 02/mar/2016
 *      Author: massimo
 */

#ifndef GIROSCOPIO_H_
#define GIROSCOPIO_H_

#include <stdint.h>

#define		maxDimBuff		64
#define		numSampleBias	32

class Giroscopio {
public:
	Giroscopio();
	virtual ~Giroscopio();

	void initGyro(char);
	void setupAssi(char stato);
	void misuraAngoli();
	void azzeraAssi(void);
	void printAsseZ(int);
	int getTemp();
	void primoAzzeramento(void);

	int16_t buffValori[maxDimBuff];
	int16_t buffX[maxDimBuff];
	int16_t tempReg;
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
	float yawF, yawF0;
	float rollF;
	float pitchF;
	/// valori precedenti degli assi
	float yawP, rollP, pitchP;
	/// valori di gradi al secondo dedotti dal manuale del giroscopio
	int16_t gradiSec;
	/// fondo scala (�/s)
	float FS;
	/// intervallo di integrazione in ms
	float tick;
	/// fattori di scala per ciscun asse
	float kz, ky, kx;
	/// temepratura
	int temp;
	char IsRotating;
private:
	uint16_t posizione;

};

uint8_t printFloat(double number, uint8_t digits);

#endif /* GIROSCOPIO_H_ */
