/*
 * maze.cpp
 *
 *  Created on: 02/apr/2016
 *      Author: massimo
 */

#include "maze.h"
#include "cerca.h"

maze::maze() {
	// TODO Auto-generated constructor stub
	floor = 0;
}

maze::~maze() {
	// TODO Auto-generated destructor stub
}

/// restituisce un booleano per indicare la presenza di una cella.
/// se il risultaot e' vero itC punta alla cella trovata
bool maze::blockPresent(int r, int c){

	/// alloca l'iteratore per scorrere la lista
	itC = L.begin();
	itC = find_if(itC++, L.end(), cercaCella(r, c));
	if (itC != L.end())
		/// ha trovato la cella
		return true;
	else
		return false;

}

maze & maze::carica(cell &c){
	/// controllo su c
	if (c.isDiscovered)
		L.push_back(c);
	return *this;
}

///
/// questo metodo costruisce le celle esplorate e le memorizza nella lista
maze& maze::run(ALLSTRUCT &D){
	cell c(0,0);
	L.push_back(c);
}
