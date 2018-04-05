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
	statoMappatura = INIZIO;
	r = c = 1;
	rot = 0;
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
	/*
	 * 1. deve capire se si sta muovendo in una nuova cella
	 * 2. il versore x è quello in direzione del moto
	 * 3. il parametro rot indica il versore x della cella rispetto al versore X del riferimento assoluto (campo di gara)
	 *
	 * */
	int16_t angoloZ;
	cell c1;
	switch(statoMappatura){
	case INIZIO:
		/// assegna la prima cella
		if (L.empty()){
			/// serve il colore
			if (D.colorClass->isValid){
				int col = D.colorClass->get();
				/// pone la cella nello stato valido
				cell c(1,1);
				c.colore = col;
				L.push_back(c);
			}
			/// intanto carica la distanza totale percorsa
			distTot = (D.encoder0->dist_mm + D.encoder1->dist_mm) / 2;
			distPar = distTot;
			distIn = distTot;
		}
		else{
			/// tratta il caso successivo all'inizio e quindi misura la distanza percorsa per capire se
			/// va avanti o altro.
			if ((D.encoder0->dist_mm + D.encoder1->dist_mm) / 2 > distIn){
				if (!D.gyro->IsRotating || !D.gyro->IsPresent)
					/// avanza alla cella successiva
					statoMappatura = AVANTI;
				else
					statoMappatura = RUOTA;
			}
		}
	break;

	case AVANTI:
		/// non si e' certi se si va avanti o si ruota e quindi occorre
		/// interrogare la classe che gestisce il giroscopio
		distTot = (D.encoder0->dist_mm + D.encoder1->dist_mm) / 2;
		distPar = distTot - distIn;
		if (distPar >= 300){
			distIn = distTot;
			/// imposta la direzione
			rot = 0;
			statoMappatura = CARICA_CELLA;
		}
		/// se si accorge che sta ruotando esce dallo stato di avanzamento
		if (D.gyro->IsRotating && D.gyro->IsPresent){
			statoMappatura = RUOTA;
		}

	break;

	case CARICA_CELLA:
//		/// deve calcolare la nuova coordinata di cella, partendo da r,c attuali
//		/// rot == 0 => va aumentata la x
//		/// rot == 90 => aumenta la y
//		/// rot == 180 => diminuisce la x
//		/// rot == 270 oppure == -90 diminuisce la y
		if (rot == 0)
			r++;
		if (rot == 90)
			c++;
		if (rot == 180)
			r--;
		if (rot == 270 || rot == -90)
			c--;

		/// imposta le varie prorpietà della cella: coordinate, direzione di esplorazione, colore di
		/// sonfodo, presenza di muri.
		c1.setRC(r, c);
		c1.setRot(rot);
		c1.colore = D.colorClass->get();
		//TODO lettura muri
		L.push_back(c1);

		statoMappatura = INIZIO;
	break;

	case RUOTA:
		/// in questo stato e nei successivi DX e SX non si controlla la misura
		/// degli encoder ma si interroga il giroscopio, avanzando di stato
		/// quando la rotazione e' terminata. Il valore dell'angolo del
		/// giroscopio indica quanto riportare nella proprietà rot
		/// la rotazione non comporta il caricamento della cella,
		/// perché si suppone che non sia cambiata la cella, ma solo "vista"
		/// con altro s.r.
		angoloZ = D.gyro->yaw;
		if (!D.gyro->IsRotating){
			/// terminata la rotazione provvede a impostare il valore dell'asse
			/// x locale della cella.
			if (angoloZ > 88 && angoloZ < 92){
				rot = 90;
				statoMappatura = INIZIO;
			}
			if (angoloZ > 178 && angoloZ < 182){
				rot = 180;
				statoMappatura = INIZIO;
			}
			if ((angoloZ > 268 && angoloZ < 272) ||
				(angoloZ > -92 && angoloZ < -88)){
				rot = -90;
				statoMappatura = INIZIO;
			}
		}

	break;

	default:
		statoMappatura = INIZIO;
	break;
	}

	return *this;
}
