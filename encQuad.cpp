/*
 * encQuad.cpp
 *
 *  Created on: 23/feb/2016
 *      Author: massimo
 */

#include "encQuad.h"

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"

#include "pwm/pwm.h"
#include "uartp/uartstdio.h"
#include "uartp/uart.h"



void UnlockPD7_01()
{
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) 	 = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR)		|= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) 	&= 0x80;
	//HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) 	&= Pin;
	HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) 	|= 0x60000000;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) 	 = 0;

}

encQuad::encQuad() {
	// TODO Auto-generated constructor stub
	address 	= 0;
	/// DA CONTROLLARE SE 80000 VA BENE
	fscala 		= 80000;
	zero_pos 	= 0;
	/// velocita' calcolata sul un intervallo di 1ms
	vel_period = ROM_SysCtlClockGet() / 80000;
	kPos = 1.0;
}

encQuad::~encQuad() {
	// TODO Auto-generated destructor stub
}

void encQuad::qeiInit(){

	///TODO: tarare i due valori: periodo e fondoscala
	//vel_period = ROM_SysCtlClockGet()/10;
	//fscala = 80000;
	//zero_pos = 0;

	// *************************************************************************************************** Lx4F232H5QD
	//    Initialize right side QEI (Quadrature Encoder Interface) for use by the right side drive.
	//    Uses GPIO Port D bits PD6 & PD7 (There is no Index wire from the encoder, perhaps we could install one on the axle).
	//    Note: Port D pin 7 defaults to a NMI input at reset. That functionality is protected by a mechanism
	//    which must be "unlocked" before it can be made available to another module (like QEI0).
	// ***************************************************************************************************

	if (address == QEI0_BASE || address == QEI1_BASE){

		switch(address){
		case QEI0_BASE:
			// Enable Port D module so we can work with it
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
			// Make pin direction of bits 6 and 7 INPUTS (this may be unnecessary?)
			UnlockPD7_01();
			GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_7 | GPIO_PIN_6, GPIO_DIR_MODE_IN);
			// Enable programming access to QEI Module 0
			SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
			QEIDisable(address);
			// Tell the mux which particular QEI function to connect to specified pin
			GPIOPinConfigure(GPIO_PD6_PHA0);
			GPIOPinConfigure(GPIO_PD7_PHB0);    // now redundant
			// Tell the GPIO module which pins will be QEI type pins
			GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_7 | GPIO_PIN_6);
		break;

		case QEI1_BASE:
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
			GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_DIR_MODE_IN);
			// Enable programming access to QEI Module 1
			SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
			QEIDisable(address);
			// Tell the mux which particular QEI function to connect to specified pin
			GPIOPinConfigure(GPIO_PC5_PHA1);
			GPIOPinConfigure(GPIO_PC6_PHB1);
			// Tell the GPIO module which pins will be QEI type pins
			GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
		break;
		}

		//    // Write this 'key' 0x4C4F434B into Port D's Lock register to enable access to Port D's Commit register
		//
		//    //HWREG(GPIO_0_LOCK) |= GPIO_LOCK_KEY;
		//    GPIO_O_LOCK = GPIO_LOCK_KEY;
		//
		//    // Flip only bit 7 ON to ALLOW Port D bit 7 to be switched from NMI use to QEI use
		//    GPIO_O_CR |= 0x80;
		//
		//
		//    // Switch pin usage
		//    GPIO_O_AFSEL |= 0x80;            // Selects alternative usage for the pin
		//    //GPIO_PORTD_PCTL_R  |= 0x60000000;    // Selects QEI0 PHB0 in particular (pages 722 & 1405 in LM4F232H5QD manual)
		//    GPIO_O_PCTL |= 0x60000000;
		//
		//    // Flip only bit 7 OFF to Re-lock
		//    GPIO_O_CR &= !0x08;

		//
		//! Configures the quadrature encoder.
		//!
		//! \param ui32Base is the base address of the quadrature encoder module.
		//! \param ui32Config is the configuration for the quadrature encoder.  See
		//! below for a description of this parameter.
		//! \param ui32MaxPosition specifies the maximum position value.
		//!
		//! This function configures the operation of the quadrature encoder.  The
		//! \e ui32Config parameter provides the configuration of the encoder and is
		//! the logical OR of several values:
		//!
		//! - \b QEI_CONFIG_CAPTURE_A or \b QEI_CONFIG_CAPTURE_A_B specify if edges
		//!   on channel A or on both channels A and B should be counted by the
		//!   position integrator and velocity accumulator.
		//! - \b QEI_CONFIG_NO_RESET or \b QEI_CONFIG_RESET_IDX specify if the
		//!   position integrator should be reset when the index pulse is detected.
		//! - \b QEI_CONFIG_QUADRATURE or \b QEI_CONFIG_CLOCK_DIR specify if
		//!   quadrature signals are being provided on ChA and ChB, or if a direction
		//!   signal and a clock are being provided instead.
		//! - \b QEI_CONFIG_NO_SWAP or \b QEI_CONFIG_SWAP to specify if the signals
		//!   provided on ChA and ChB should be swapped before being processed.
		//!
		//! \e ui32MaxPosition is the maximum value of the position integrator and is
		//! the value used to reset the position capture when in index reset mode and
		//! moving in the reverse (negative) direction.
		//!
		//! \return None.

		//configurazione qei
		QEIConfigure(address,(QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), fscala);
		//QEIConfigure(QEI1_BASE,(QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), fscala);

		QEIPositionSet(address, zero_pos);
		//QEIPositionSet(QEI0_BASE, zero_pos);

		//configurazione lettore velocità del qei
		QEIVelocityDisable(address);
		//QEIVelocityDisable(QEI1_BASE);

		QEIVelocityConfigure(address, QEI_VELDIV_1, vel_period);  //periferica, divisore, periodo
		//QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, vel_period);  //periferica, divisore, periodo

		QEIVelocityEnable(address);
		//QEIVelocityEnable(QEI1_BASE);


		//configurazione interrupt
	//	QEIIntRegister(QEI0_BASE,*QEI0IntHandler);
	//	QEIIntRegister(QEI1_BASE,*QEI1IntHandler);
	//
	//	///TODO: se si vuole usare l'interrupt del qei vanno decommentate le due righe che seguono
	//	//IntEnable(INT_QEI0);
	//	//IntEnable(INT_QEI1);
	//
	//	//QEIIntEnable(QEI0_BASE, QEI_INTDIR | QEI_INTTIMER); //interruzione abilitata al cambio di direzione e al timer della velocità finito
	//	QEIIntEnable(QEI0_BASE, QEI_INTDIR);
	//	QEIIntEnable(QEI1_BASE, QEI_INTDIR);
		QEIEnable(address);
		//QEIEnable(QEI0_BASE);
	}
}


int encQuad::readPos(){
	float tmp;
	pos = QEIPositionGet(address);
	tmp = pos * kPos;
	dist_mm = (int) tmp;
	return dist_mm;

}

float encQuad::readVel(){
	/// legge la velocità misurata.
	/// in questo caso l'intervallo di lettura sono 0.001s e quindi verrano letti val/0,01 impulsi al secondo
	/// RICORDARSI DI DIVIVERE PER IL RAPPROTO distanza/numImpulsi
	vel = QEIVelocityGet(address) * 1000;
	return vel;

}

int encQuad::readDir(){

	dir = QEIDirectionGet(address);

}

///
/// aggiorna i parametri dell'encoder
void encQuad::update(){

	pos =  QEIPositionGet(address);
	vel =  QEIVelocityGet(address) * 100;
	dir =  QEIDirectionGet(address);

}

