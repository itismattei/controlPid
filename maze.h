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


class cell{
public:
	cell() {x = 0; y = 0; isDiscovered = false;}
	cell(int a, int b){x = a; y = b; isDiscovered = true;}
	/// costruttore di copia
	cell(const cell &c){ x = c.x; y = c.y; isDiscovered = c.isDiscovered;}
	virtual ~cell(){;}

	void setRC(int a, int b){x = a; y = b;}

	// proprieta'
	int16_t 		x;
	int16_t 		y;
	bool			isDiscovered;
	uint8_t 		w[4];


};

#include <list>
using namespace std;

class maze {
public:
	maze();
	virtual ~maze();

	maze & carica(cell &c) {L.push_back(c); return *this;}

	list<cell> L;
	int floor;
};

#endif /* MAZE_H_ */
