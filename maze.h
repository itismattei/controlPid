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

class cell{
public:
	cell();
	virtual ~cell();

	// proprieta'
	int16_t 	x;
	int16_t 	y;
	bool		isDiscovered;

};


class maze {
public:
	maze();
	virtual ~maze();

	cell grid[200];
	int floor;
};

#endif /* MAZE_H_ */
