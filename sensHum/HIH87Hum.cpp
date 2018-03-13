/*
 * HIH87Hum.cpp
 *
 *  Created on: 02 set 2017
 *      Author: lenovoMax
 */

#include "HIH87Hum.h"
#include "stdlib.h"
#include "../init.h"		// per float2string
#include <stdio.h>

HIH8_7Hum::HIH8_7Hum() {
	// TODO Auto-generated constructor stub
	i2cPtr = NULL;
	dataValid = false;
	dataOld = true;
}


///
/// collega il sensore alla classe che gestisce la comunicazione I2C
void HIH8_7Hum::attachI2C(I2C *p, uint8_t sa){
	i2cPtr = p;
	i2cPtr->I2CSetSlave_Add(sa);
}


///
/// legge i 4 byte de sensore: umidita' e temepratura.
/// i dati sono su 14 bit: per l'umidita' i primi due bit indicano lo stato
/// della lettura (00 = OK; 01= lettura gia' effettuata);
/// per la temperatura, gli ultimi due bit sono "don't care"

void HIH8_7Hum::readRaw(){
	if (i2cPtr->stato == IMPOSTATA)
		i2cPtr->I2CGetN(4, buff);
	else{
		/// c'e' un errore sul canale. riprova una volta e poi si arrende
		i2cPtr->stato &= ~ERRORE_COM;
		i2cPtr->I2CGetN(4, buff);
	}
}

///
/// per campionare un nuovo dato,occorre una scrittura "dummy" ed attendere circa 36ms
void HIH8_7Hum::newData(){
	i2cPtr->I2CPut(0);
}

///
/// converte il dato raw in dato di umidita' e temperatura
HIH8_7Hum & HIH8_7Hum::convertToHum(){
	/// analizza se il dato e' valido
	if (i2cPtr->stato != IMPOSTATA || (buff[0] & 0x80) != 0){
		/// bit S = 1, oppure connessione con errori: dato  non valido
		dataValid = false;
	}
	else{

		/// dato valido
		dataValid = true;
		if ((buff[0] & 0x40) == 0x40)
			dataOld= true;
		else
			dataOld = false;

		//conversione del dato
		/// cancella i 2 bit più significativ
		uint16_t valC = (buff[0] & 0x3F);
		/// shift di 8 posizioni;
		valC <<= 8;
		/// carica il byte meno significativo
		valC |= buff[1];

		/// valore RH% HumRaw/(2^14-2) x 100
		/// 2^14 -2 = 16382
		hum = valC / 16382.0 * 100.0;
	}
	return *this;
}

///
/// cnverte il dato di temperatura raw in dato di temperatura °C
/// da chiamare dopo la conversione da raw ad umidita'
HIH8_7Hum & HIH8_7Hum::convertToTemp(){

	if(dataValid == true){
		/// compone il dato di temperatura grezzo, lo trasla a destra di 2 posizioni
		uint16_t valT = (buff[2] << 8) | buff[3];
		valT >>= 2;

		///temperatura = temp Raw (2^14 -2) * 165 - 40
		temp = valT / 16382.0 * 165.0 - 40.0;
	}

	return *this;
}

///
/// converte i dati raw in dati di temperatura ed umidita'
void HIH8_7Hum::convertRaw(){
	convertToHum().convertToTemp();
	if (dataValid && !dataOld)
		toString();
}

/// converte i valori presenti in hum e temp in stringhe di uint8_t
/// "costa" circa 90kb di flash
void HIH8_7Hum::toString(){
	/// converte un float o un double a vettore di uint8_t
	//float2string(hum, humidity);
	//float2string(temp, temperature);
	/// "costa" 5kb di flash
	if (hum < 0.0001 || hum > 1000)
		/// stampa in notazione scientifica
		sprintf(humidity, "%8.5e", hum);
	else
		/// stampa in notazione con punto separatore
		sprintf(humidity, "%f", hum);

	if (temp < 0.0001 || temp > 1000)
		sprintf(temperature, "%8.5e", temp);
	else
		sprintf(temperature, "%f", temp);


}
