/*
 * accel.c
 *
 *  Created on: 19/mar/2015
 *      Author: robotics
 */


//#include "accel.h"
#include "accel.h"
#include <stdbool.h>
#include <stdint.h>
#include "../gyro_init.h"
#include "../Giroscopio.h"		/// per printfloat
#include "../I2C/tiva_i2c.h"
#include "../uartp/uart.h"
#include "../uartp/uartstdio.h"


	/*          INIT ACCELEROMETER REGISTERS                */
		//initI2C_B1(FDCO, 800000, ACCEL_ADDR);
		//valore = readI2CByteFromAddress(CTRL_REG1_A, &stato);
		//stato =  writeI2CByte(CTRL_REG1_A, ODR1 + ODR0 + ZaxEN + YaxEN + XaxEN);

void accelerometro::testAccel(){

	volatile uint32_t valore;

	I2CSend(ACCEL_ADDR, 2, CTRL_REG1_A, ZaxEN + YaxEN + XaxEN);
	/// legge il registro di controllo 1 che dovrebbe contenere 0x07
	valore = I2CReceive(ACCEL_ADDR, CTRL_REG1_A);
	if ((valore & 0xFF) == 7){
		PRINTF("Accelereometro OK! %d\n", valore);
		isPresent =  true;
		blinkRedLed();
	}
	else
		isPresent = false;
}

///
/// avvia il modulo accelerometro
void accelerometro::impostaAccel(){

	volatile uint32_t valore;
	/// imposta il campionamento a 50 sample / s ed abilita i 3 assi
	I2CSend(ACCEL_ADDR, 2, CTRL_REG1_A, ODR1 + ODR0 + ZaxEN + YaxEN + XaxEN);
	/// attesa dell'accensione del dispositivo
	valore = 1000;
	while(--valore);
	valore = I2CReceive(ACCEL_ADDR, CTRL_REG1_A);
}

///
/// misura i dati forniti dall'accelelrometro se disponibili
/// sono restituiti nella struct passata per indirizzo
void accelerometro::misuraAccelerazioni(){

	volatile int x, y, z;
	uint8_t buffer[8];
	/// legge i valori degli assi
	if((I2CReceive(ACCEL_ADDR, STATUS_REG_A) & 0xF) != 0){
		I2CReceiveN(ACCEL_ADDR, OUT_X_L_A | MUL_READ , 6, buffer);
		x = (int16_t)((buffer[1]<< 8) + buffer[0]);
		y = (int16_t)((buffer[3]<< 8) + buffer[2]);
		z = (int16_t)((buffer[5]<< 8) + buffer[4]);
		a[0] = (float) x * 2 / 32768;
		a[1] = (float) y * 2 / 32768;
		a[2] = (float) z * 2 / 32768;
		PRINTF("acc z:\t");
		printFloat(a[2], 4);
		PRINTF("\n");
		///PRINTF("acc x %d\t acc y %d\t acc z %d\n", x, y, z);
	}
}
