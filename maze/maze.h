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

	void setRC(int a, int b){x = a; y = b; isDiscovered = true;}	/// l'impostazione delle coordinate pone la cella nello stato true
	int setRot(int r){rot = r; return r;}

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
	int				colore;

};

#define		INIZIO			0
#define		AVANTI			1
#define		NUOVA_CELL		2
#define		RUOTA_DX		3
#define		RUOTA_SX 		4
#define		ASPETTA			5
#define		RETRO			6
#define		FERMA			7
#define		CARICA_CELLA	8
#define		RUOTA			9
/// in fase di ulteriore definizione


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
	int distTot;		/// misura della distanza percorsa
	int distPar;		/// variabile per distanza parziale
	int distIn;			/// variabile per distanza iniziale
	int statoMappatura;	/// stato dell'automa per la mappatura
	int r,c;			/// coordinate cella attuale
	int rot;			/// rotazione dell'asse x locale rispetto all'asse X assoluto
};

#endif /* MAZE_H_ */
