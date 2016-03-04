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

	void azzeraAssi(void);
	void printAsseZ(int);

	int16_t buffValori[512];
	int16_t buffX[512];
	float  media;
	float m, q;

};

#endif /* GIROSCOPIO_H_ */
