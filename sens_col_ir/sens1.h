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

void initLightSens(void);
void initTimer4(uint8_t);
void whiteBal(colore *);

#ifdef __cplusplus
}
#endif


#endif /* SENS1_H_ */
