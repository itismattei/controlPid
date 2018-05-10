/*
 * encQuad.h
 *
 *  Created on: 23/feb/2016
 *      Author: itismattei
 */

#ifndef ENCQUAD_H_
#define ENCQUAD_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "Giroscopio.h"


class encQuad {
public:
	encQuad();
	encQuad(uint32_t add);
	virtual ~encQuad();

	void qeiInit();
	int readPos();
	int readPos(Giroscopio *G);
	float readVel();
	int readDir();
	void setAddr(uint32_t add){address = add;}
	void setCoeff(float c){ kPos = c;}
	void update();

	/// prorpieta'
	float 		vel;
	int 		dir;
	int 		pos;
	int 		vel_period;
	int 		fscala;
	int 		zero_pos;
	uint32_t 	address;
	/// distanza percorsa in mm
	int			dist_mm;
	float 		kPos;
	/// distanza in mm escluse le rotazioni
	int 		dist_mmNR;
	int 		deltaNR_R;

private:
	void ctorIn(void);
};

#endif /* ENCQUAD_H_ */
