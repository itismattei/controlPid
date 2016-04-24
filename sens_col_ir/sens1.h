/*
 * sens1.h
 *
 *  Created on: 20/apr/2015
 *      Author: massimo
 */

#ifndef SENS1_H_
#define SENS1_H_

#include "../I2C/i2cTiva.h"
#include <stdint.h>

//typedef struct _temp{
//	float 	Temp;
//	int 	tempRaw;
//	int 	T_tar;
//	int 	Tcase;
//	int		ok;
//} temperatura;



class TEMPER{
public:

	TEMPER(){ok = 0; isSurvivor = 0;}
	~TEMPER(){;}
	void readTemp();
	float getTemp(){ return Temp; }
	void taraturaTemp();
	void attachI2C(I2C *, uint8_t sa);

	float 		Temp;
	int 		tempRaw;
	int 		T_tar;
	int 		Tcase;
	int			ok;
	uint16_t 	isSurvivor;
	I2C* 		i2cPtr;

};


//#ifdef __cplusplus
//extern "C" {
//#endif
//
//
//void readColourSens(colore *);
//void taraturaTemp(temperatura *tempPtr);
//void readTemp(temperatura *tempPtr);
//
//void initLightSens1(void);
//void initTimer4(uint8_t);
//int readCol(void);
//
//#ifdef __cplusplus
//}
//#endif


#define		IS_DARK		1
#define		ISNT_DARK	0

void initLightSens1(void);
int readCol(void);

/// indica le mattonelle scure da evitare
class TILE{
public:
	TILE(){;}

	uint32_t isDark;
};


class COLORE{
public:
	COLORE();

	inline void Init(){ initLightSens1(); initTimer4(10);}
	/// legge il valore di luminanza
	inline int read(){ return readCol(); }
	/// legge il valore di luminanza
	inline int get(void){ return luminanza; }
	/// restituisce il valore del bianco
	inline int getWhite(void){ return bianco; }
	inline void set(int v){ luminanza = v; }
	//Imposta il livello di bianco, ipotizzando che all'inizio la paistrella sia bianca.
	inline void WhiteBalance(){ bianco = read(); }

	void Run();

	/// proprieta'
	int 	rosso;
	int 	verde;
	int 	blu;
	int 	luminanza;
	int 	bianco;
	TILE	piastra;
};



#endif /* SENS1_H_ */
