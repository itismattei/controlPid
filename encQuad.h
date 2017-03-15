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


class encQuad {
public:
	encQuad();
	virtual ~encQuad();
	void qeiInit();
	void readPos();
	float readVel();
	void readDir();
	void setAddr(uint32_t add){address = add;}
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

};

#endif /* ENCQUAD_H_ */
