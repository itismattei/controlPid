/*
 * Giroscopio.cpp
 *
 *  Created on: 02/mar/2016
 *      Author: massimo
 */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "gyro_init.h"
#include "uartp/uartstdio.h"
#include "I2C/tiva_i2c.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "gen_def.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "uartp/uartstdio.h"
#include "uartp/uart.h"

#include "Giroscopio.h"

Giroscopio::Giroscopio() {
	// TODO Auto-generated constructor stub

}

Giroscopio::~Giroscopio() {
	// TODO Auto-generated destructor stub
}

void Giroscopio::azzeraAssi(){
	int conteggio = 0;
	int valore;

	uint8_t buffer[2];

	media = 0;
	while(conteggio < 512){
			/// effettua 32 letture e calcola la media
			valore = I2CReceive(GYRO_ADDR,STATUS_REG);
			//PRINTF("REG_STAT 0x%x\n", valore);
			if (valore != 0){
				/// asse z ON
				I2CReceiveN(GYRO_ADDR,OUT_Z_L | MUL_READ , 2, buffer);
				buffValori[conteggio] = (int16_t)((buffer[1]<< 8) + buffer[0]);
				buffX[conteggio] = conteggio;
				media += buffValori[conteggio];
				//z += (int16_t )((buffer[1]<< 8) + buffer[0]);
				conteggio++;
				for (int i = 0; i < 50000; i++);
			}
		}
	media /= 512.0;
	printAsseZ(512);
	minQuad(buffX, buffValori, 512, &m, &q);

}


///
/// stampa i valori dell'asse z del giroscopio

void Giroscopio::printAsseZ(int numS){
	for (int16_t i = 0; i < numS; i++)
		PRINTF("%d\n", buffValori[i]);
}
