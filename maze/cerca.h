/*
 * cerca.h
 *
 *  Created on: 02 apr 2018
 *      Author: massimo
 */

#ifndef MAZE_CERCA_H_
#define MAZE_CERCA_H_

#include "maze.h"

struct cercaCella{

public:

	cercaCella(int riga, int colonna) : r(riga), c(colonna) {}
	bool operator()(cell &ce) const {
		if (r == ce.x && c == ce.y)
			return true;
		else
			return false;
	}

	/// proprieta'
	int r, c;
};

#endif /* MAZE_CERCA_H_ */
