/*
 * sens1.h
 *
 *  Created on: 20/apr/2015
 *      Author: massimo
 */

#ifndef SENS1_H_
#define SENS1_H_

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

	float 		Temp;
	int 		tempRaw;
	int 		T_tar;
	int 		Tcase;
	int			ok;
	uint16_t 	isSurvivor;

};


#ifdef __cplusplus
extern "C" {
#endif


void readColourSens(colore *);
void taraturaTemp(temperatura *tempPtr);
void readTemp(temperatura *tempPtr);

void initLightSens1(void);
void initTimer4(uint8_t);
int readCol(void);

#ifdef __cplusplus
}
#endif



class Colour{
public:
	Colour(){ luminanza = 0;}

	inline void Init(){ initLightSens1(); initTimer4(10);}
	inline int read(){ return readCol(); }
	inline int get(void){ return luminanza; }
	inline void set(int v){ luminanza = v; }
	inline void WhiteBalance(){ set(read()); }
	int rosso;
	int verde;
	int blu;
	int luminanza;
	int bianco;
};



#endif /* SENS1_H_ */
