/*
 * maze.h
 *
 *  Created on: 02/apr/2016
 *      Author: massimo
 */

#ifndef MAZE_H_
#define MAZE_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <list>
#include "../parse.h"


class cell{
public:
	cell() {x = 0; y = 0; isDiscovered = false;}
	cell(int a, int b){x = a; y = b; isDiscovered = true; rot = 0;}
	/// costruttore di copia
	cell(const cell &c){ x = c.x; y = c.y; isDiscovered = c.isDiscovered;}
	virtual ~cell(){;}

	void setRC(int a, int b){x = a; y = b;}
	int setRot(int r){rot = r;}

	// proprieta'
	int16_t 		x;
	int16_t 		y;
	bool			isDiscovered;
	uint8_t 		w[4];
	/// angolo di rotazione rispetto al sistema di riferimento asoluto
	/// si assume che il versore x sia la direzione alla partenza.
	/// Indica la'ngolo con cui si entra nella cella (che comunque sarebbe
	/// ricavabile dai valori x e y delle coordinate di cella
	int32_t			rot;

};

#include <list>
using namespace std;

class maze {
public:
	maze();
	virtual ~maze();

	maze & carica(cell &c);
	bool blockPresent(int r, int c);
	maze& run(ALLSTRUCT &);


	list<cell> L;
	list<cell>::iterator itC;
	int floor;
};

#endif /* MAZE_H_ */
