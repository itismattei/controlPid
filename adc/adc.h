/*
 * adc.h
 *
 *  Created on: 15/mar/2015
 *      Author: robotics
 */

#ifndef ADC_H_
#define ADC_H_

//#include "../accel/accel.h"
#include <stdint.h>
#include "../distMis.h"


///
/// struttura che mantiene la distanza misurata dai 5 sensori
/// e la conversione in millimetri
typedef struct _dist{

	float 		misSens[6];		/// distanza in millimetri con virgola
	float 		kf;				/// coefficiente di trasformazione tensione - distanza
	int 		d_mm[6];		/// distanza in millimetri (intera)
	uint8_t		d_cm[6];		/// misura in cm per comunicazioni con raspberry
	uint32_t	dI[8];
	bool		run;			/// indica se il dispositivo funziona

} distanza;


typedef struct _cinematica{
	//accelerazione	*Aptr;			/// accelerazione fornita dal sensore inerziale
	distanza		*distPTR;		/// distanza fornita dai sensori IR
	float			vel;			/// velocità media
	float			spazio[2];		/// distanza misurata su ciascun cingolo
} cinematica;


#ifdef __cplusplus
extern "C" {
#endif

void initAdc(volatile distMis *);
void initHW_ADC(void);

#ifdef __cplusplus
}
#endif


#endif /* ADC_H_ */
