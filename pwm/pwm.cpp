/*
 * pwm.c
 *
 *  Created on: 09/mar/2015
 *      Author: robotics
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "pwm.h"
#include "motpwm.h"


///******************************************
/*				PIN PWM
 *
 * 		Tiva		MOTORE destro				L293(pin)
 *
 * 	PB4(M0PWM2)				delta_1				chip enable 1 (1)
 * 		PB6				In_1				input 1(2)
 * 		PB7				In_2				input 2(9)
 * 		 -				dir_1					-
 *
 *
 * 		Tiva		MOTORE sinistro				L293(pin)
 *
 * 	PB5(M0PWM3)			delta_2				chip enable 2 (11)
 * 		PA3				In_3				input 3(12)
 * 		PA2				In_4				input 4(19)
 * 		 -				dir_2					 -
 *
 *
 *******************************************///

#define IN1 	GPIO_PIN_6		//PB6
#define IN2 	GPIO_PIN_7		//PB7
#define IN3 	GPIO_PIN_3		//PA3
#define IN4 	GPIO_PIN_2		//PA2


///in1, in2 -> pb6, pb7
///in3, in4 -> pa3, pa2

///
/// direzione del mezzo
void pwm_dir(pwm *p){
	/*  in1 = in3 = 1; in2 = in4 = 0  avanti
	 *  in1 = in3 = 0; in2 = in4 = 1  indietro
	 *  in1 = 1, in2 = 0; in3 = 0, in4 = 1 rotazione antioraria
	 *  in1 = 0, in2 = 1; in3 = 1, in4 = 0 rotazione oraria
	 *
	 *  */
	uint8_t direzione = ((p->dir_1 & 0xFF) << 4) | (p->dir_2 & 0xFF);

	// direzione X.X.X.X:Y.Y.Y.Y
	//           in1.in2:in3.in4
	switch(direzione){
	case 0x11:		//avanti: IN1A = 1, IN2A = 1
	case 0x31:		/// effettua una rotazione a grande angolo

		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN4) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN2) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN1) << 2)) |=  IN1;
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN3) << 2)) |=  IN3;
	break;

	case 0x22:		// indietro: IN1B = 1, IN2B = 1
	case 0x32:		/// effettua una rotazione a grande raggio

		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN3) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN1) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN2) << 2)) |=  IN2;
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN4) << 2)) |=  IN4;
	break;

	case 0x21:
		// rotazione oraria: IN1B = 1, IN2B = 1
		/// rotazione su asse centrale
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN3) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN1) << 2)) |=  IN1;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN2) << 2)) =  0;
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN4) << 2)) |=  IN4;
	break;
		//0001:0010
	case 0x12:
		// rotazione antioraria
		/// rotazione su asse centrale
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN4) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN2) << 2)) |=  IN2;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN1) << 2)) =  0;
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN3) << 2)) |=  IN3;

	break;


	case 0x00:
	default:
		// stop
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN4) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN2) << 2)) =  0;
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (IN1) << 2)) =  0;
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (IN3) << 2)) =  0;
	break;
	}
}


//*****************************************************************************
//
// Funzione per mandare avanti i motori
//
//*****************************************************************************

uint32_t convertDegToPwm(pwm *p, int16_t gradi){

	/// -90° -> delta_2 = 5; +90° -> delta_2 = 10
	float rot, delta;
	uint32_t valFin;
	rot = gradi / 36.0 + 7.50;
	delta = (float) p->NCont * rot;
	valFin = (uint32_t) delta;
	return valFin;
}


///
/// PWM per i motori di spostamento
void pwm_power(pwm *p){

	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, p->NCont * p->delta_1 / 100);    //delta_1 è del motore dx, uscita PWM-> M0PWM2
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, p->NCont * p->delta_2 / 100);    //delta_2 è del motore sx, uscita PWM-> M0PWM3

}

///
/// attiva il motore su PA6 lo scarico del kit
void pwmPowerA6(pwm *p, int16_t gradi){

	uint32_t valFin;
	valFin = convertDegToPwm(p, gradi);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, valFin / 100);    //delta_1 e' del servo di movimento dello scrico del rescue kit
}


///
/// attiva il motore su PA7 il sensore di temperatura
void pwmPowerA7(pwm *p, int16_t gradi){

	uint32_t valFin;
	valFin = convertDegToPwm(p, gradi);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, valFin / 100);    //delta_2 e' del pwm del sensore IR
}


///
/// questa funzione inizializza un solo modulo del PWM: per ottenre il controllo  dei motori a 8kHz
void pwmMotInit(pwm *p){

	/// per ottenre una frequenza di 50 Hz necessaria ai servi, partendo da 80MHz di clock si deve dividere pe la costante 1.6e6
	/// che è ottenuta come prodotto tra 50000 (contenuto del registro di fine conteggio a 16 bit della cpu) e 32 (divisore del clock di sistema)
	/// 80000000 / (50000 * 32) = 50. Infatti il tempo di raggiungere 1600000 conteggi  e' di 20 ms
	/// Per ottenre una frequenza di 8kHz occorre invece dividere il clock per la costante 10000 = 32 * 312.5 e quindi il fine conteggio
	/// dovra' essere 312. Ci'o significa che la granularità del PWM e' di 1/312 = 0.32% inferiore a quanto riloevabile dai motori
	/// I valori sono quindi: 80000000 / (8000 * 32) = 80000000 / 256000 e quindi il fine conteggio e' 312.5
    p->NCont = ROM_SysCtlClockGet() / 256000;
    p->delta_1 = 0;
    p->delta_2 = 0;

	/// Abilita le porte usate dal PWM, e per il comando della direzione dei motori,
	/// anche se potrebbero gia' esserlo stato inaltre parti del programma
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	///in1, in2 -> pb6, pb7
	///in3, in4 -> pa3, pa2
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, IN1 | IN2);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN3 | IN4);
	/// azzeramento delle uscite dei pin
	HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_6 | GPIO_PIN_7) << 2)) =  0;
	HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_3 | GPIO_PIN_2) << 2)) =  0;
    //
    // Enable the GPIO port B
    //
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32);  //divisore per 32
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //modulo pwm numero 0

    //Configure PB4, PB5 Pins as PWM
    ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);  //generatore 1
    ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2); // generatore 1
    ///PB4 e' il modulo 1, mentre PB5 e' il modulo 2 del L293E
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);


    //il generatore e' il numero 1
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Imposta il periodo del PWM rispetto al clock
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, p->NCont);

    // Enable the PWM generator
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // Turn on the Output pins
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT|PWM_OUT_2_BIT, true);

    /// spegne i PWM
    p->delta_1 = 0;
    p->delta_2 = 0;
    pwm_power(p);
}

///
/// abilita il modulo 1 del pwm per comandare i servomotori
void pwmServoInit (pwm *p){

	volatile unsigned long perM = 0;

	//configura il divisore del clock di sistema (deve combaciare con la precedente funzione)
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

	// Enable the peripherals used by this program.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	/// la frequenza di ripetizione del segnale e' 50Hz e quindi 80000000 / (50 * 32) = 1600
	p->NCont = ROM_SysCtlClockGet() / 1600;

	//Configura PA6 e PA7 come pin PWM
	ROM_GPIOPinConfigure(GPIO_PA6_M1PWM2);
	ROM_GPIOPinConfigure(GPIO_PA7_M1PWM3);
	ROM_GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

// configura il modo di funzionamento del PWM: contattore su-giu'.
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	// imposta il periodo
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, p->NCont);
    // azzera il pwm
	p->delta_1 = p->delta_2 = 0;
	//lo scarico
	pwmPowerA6(p, 0);
	//il sens di temperatura
	pwmPowerA7(p, 0);

	// Abilita il generatore
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);

	// Turn on the Output pins
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
}

///
/// inizializza il numero di PWM usati ed il numero di servi usati
int PWM_MOTORI::numPwm = 0;
int PWM_SERVI::numServi = 0;

void PWM_MOTORI::Init(){

	if(numMot == 1){
		/// per ottenre una frequenza di 50 Hz necessaria ai servi, partendo da 80MHz di clock si deve dividere pe la costante 1.6e6
		/// che è ottenuta come prodotto tra 50000 (contenuto del registro di fine conteggio a 16 bit della cpu) e 32 (divisore del clock di sistema)
		/// 80000000 / (50000 * 32) = 50. Infatti il tempo di raggiungere 1600000 conteggi  e' di 20 ms
		/// Per ottenre una frequenza di 8kHz occorre invece dividere il clock per la costante 10000 = 32 * 312.5 e quindi il fine conteggio
		/// dovra' essere 312. Ci'o significa che la granularità del PWM e' di 1/312 = 0.32% inferiore a quanto riloevabile dai motori
		/// I valori sono quindi: 80000000 / (8000 * 32) = 80000000 / 256000 e quindi il fine conteggio e' 312.5
	    NCont = ROM_SysCtlClockGet() / 256000;
	    delta = 0;

		/// Abilita le porte usate dal PWM, e per il comando della direzione dei motori,
		/// anche se potrebbero gia' esserlo stato inaltre parti del programma
	    /// il primo pwm motori deve abilitare PE5
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		/// mentre PE4 e' una uscita digitale che combianta con il segnale PWM stabilisce la potenza al motore ed il verso di rotazione
		/// In pratica se PE4 = '0' e PE5 ha un PWM, supponendo che il motore giri in senso orario, allora
		/// quando PE4 = '1' e PE5 ha lo stesso PWM ma con polarita' invertita, il motore ruota in senso orario.

		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

		/// azzeramento delle uscite dei pin
		HWREG(GPIO_PORTE_BASE + (GPIO_O_DATA + (GPIO_PIN_4) << 2)) =  0;
		//HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_3 | GPIO_PIN_2) << 2)) =  0;
//	    //
//	    // Enable the GPIO port B
//	    //
//		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32);  //divisore per 32
	    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //modulo pwm numero 0

	    //Configure PE5 Pins as PWM
	    ROM_GPIOPinConfigure(GPIO_PE5_M0PWM5);  //generatore 2
	    //ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2); // generatore 1
	    ///PE5 e' il modulo 1, mentre PB4 e' il modulo 2 del L298
	    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);


	    //il generatore e' il numero 2
	    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	    //Imposta il periodo del PWM rispetto al clock
	    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, NCont);

	    // Enable the PWM generator
	    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);

	    // Turn on the Output pins (quello di M0PWM5)
	    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
	    /// PWM non invertito
	    ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_5_BIT, false);

	    /// spegne i PWM
	    numPin = PWM_OUT_5;
	    MotorStop();

	}
	else{
		/// per ottenre una frequenza di 50 Hz necessaria ai servi, partendo da 80MHz di clock si deve dividere pe la costante 1.6e6
		/// che è ottenuta come prodotto tra 50000 (contenuto del registro di fine conteggio a 16 bit della cpu) e 32 (divisore del clock di sistema)
		/// 80000000 / (50000 * 32) = 50. Infatti il tempo di raggiungere 1600000 conteggi  e' di 20 ms
		/// Per ottenre una frequenza di 8kHz occorre invece dividere il clock per la costante 10000 = 32 * 312.5 e quindi il fine conteggio
		/// dovra' essere 312. Ci'o significa che la granularità del PWM e' di 1/312 = 0.32% inferiore a quanto riloevabile dai motori
		/// I valori sono quindi: 80000000 / (8000 * 32) = 80000000 / 256000 e quindi il fine conteggio e' 312.5
		NCont = ROM_SysCtlClockGet() / 256000;
		delta = 0;

		/// Abilita le porte usate dal PWM, e per il comando della direzione dei motori,
		/// anche se potrebbero gia' esserlo stato inaltre parti del programma
		/// il secondo pwm motori deve abilitare PB4
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		/// mentre PE4 e' una uscita digitale che combianta con il segnale PWM stabilisce la potenza al motore ed il verso di rotazione
		/// In pratica se PA5 = '0' e PB4 ha un PWM, supponendo che il motore giri in senso orario, allora
		/// quando PA5 = '1' e PB4 ha lo stesso PWM ma con polarita' invertita, il motore ruota in senso orario.

		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

		/// azzeramento delle uscite dei pin
		HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_5) << 2)) =  0;
		//HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_3 | GPIO_PIN_2) << 2)) =  0;

		ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32);  //divisore per 32
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //modulo pwm numero 0

		//Configure PB4 Pins as PWM
		//ROM_GPIOPinConfigure(GPIO_PB4_M0PWM5);  //generatore 1
		ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2); // generatore 1
		///PE5 e' il modulo 1, mentre PB4 e' il modulo 2 del L298
		ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);


		//il generatore e' il numero 1
		ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

		//Imposta il periodo del PWM rispetto al clock
		ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, NCont);

		// Enable the PWM generator
		ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

		// Turn on the Output pins (quello di M0PWM2)
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
		/// VA FATTO L'OR CON L'ALTRO PIN????
		/// PWM non invertito
		ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_2_BIT, false);

		/// spegne i PWM
		numPin = PWM_OUT_2;
		MotorStop();

	}
}


///
/// spegne il motore associato al PWM
void PWM_MOTORI::MotorStop(){
	delta = 0;
	pwmPower();
}


////
//// questa funzione modifica il valore del PWM per l'oggetto associato al motore.
void PWM_MOTORI::pwmPower(){
	PWMPulseWidthSet(PWM0_BASE, numPin, NCont * delta / 100);
}


#include "inc/hw_pwm.h"
///
/// muove il motore
void PWM_MOTORI::MotorGo(){
	switch (numMot){
	case 1:
		if (direction == 1){
			//PWMOutputInvert(PWM0_BASE, PWM_OUT_5_BIT, false);
			/// imposta il pwm in modalita' affermata
			HWREG(PWM0_BASE + PWM_O_INVERT) &= ~(PWM_OUT_5_BIT);
			}
		else{
			//PWMOutputInvert(PWM0_BASE, PWM_OUT_5_BIT, true);
			/// imposta il pwm in modalita' negata
			HWREG(PWM0_BASE + PWM_O_INVERT) |= PWM_OUT_5_BIT;
		}
		PWMPulseWidthSet(PWM0_BASE, numPin, NCont * delta / 100);
	break;
	case 2:
		if (direction == 1){
			//PWMOutputInvert(PWM0_BASE, PWM_OUT_5_BIT, false);
			/// imposta il pwm in modalita' affermata
			HWREG(PWM0_BASE + PWM_O_INVERT) &= ~(PWM_OUT_2_BIT);
			}
		else{
			//PWMOutputInvert(PWM0_BASE, PWM_OUT_5_BIT, true);
			/// imposta il pwm in modalita' negata
			HWREG(PWM0_BASE + PWM_O_INVERT) |= PWM_OUT_2_BIT;
		}
		PWMPulseWidthSet(PWM0_BASE, numPin, NCont * delta / 100);
	break;
	}
}

///
/// inizializzazione per i  servi
void PWM_SERVI::Init(){
	if(numS == 1){
		/// per ottenre una frequenza di 50 Hz necessaria ai servi, partendo da 80MHz di clock si deve dividere pe la costante 1.6e6
		/// che è ottenuta come prodotto tra 50000 (contenuto del registro di fine conteggio a 16 bit della cpu) e 32 (divisore del clock di sistema)
		/// 80000000 / (50000 * 32) = 50. Infatti il tempo di raggiungere 1600000 conteggi  e' di 20 ms
		/// Per ottenre una frequenza di 8kHz occorre invece dividere il clock per la costante 10000 = 32 * 312.5 e quindi il fine conteggio
		/// dovra' essere 312. Ci'o significa che la granularità del PWM e' di 1/312 = 0.32% inferiore a quanto riloevabile dai motori
		/// I valori sono quindi: 80000000 / (8000 * 32) = 80000000 / 256000 e quindi il fine conteggio e' 312.5
	    NCont = ROM_SysCtlClockGet() / 1600000;
	    delta = 0;

		/// Abilita le porte usate dal PWM, e per il comando della direzione dei motori,
		/// anche se potrebbero gia' esserlo stato inaltre parti del programma
	    /// il primo pwm motori deve abilitare PF2
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		/// mentre PE4 e' una uscita digitale che combianta con il segnale PWM stabilisce la potenza al motore ed il verso di rotazione
		/// In pratica se PE4 = '0' e PE5 ha un PWM, supponendo che il motore giri in senso orario, allora
		/// quando PE4 = '1' e PE5 ha lo stesso PWM ma con polarita' invertita, il motore ruota in senso orario.

		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

		/// azzeramento delle uscite dei pin
		HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_2) << 2)) =  0;
		//HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_3 | GPIO_PIN_2) << 2)) =  0;
//	    //
//	    // Enable the GPIO port B
//	    //
//		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32);  //divisore per 32
	    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //modulo pwm numero 1

	    //Configure PE5 Pins as PWM
	    ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);  //generatore 3
	    //ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2); // generatore 1
	    ///PE5 e' il modulo 1, mentre PB4 e' il modulo 2 del L298
	    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);


	    //il generatore e' il numero 3
	    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	    //Imposta il periodo del PWM rispetto al clock
	    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, NCont);

	    // Enable the PWM generator
	    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);

	    // Turn on the Output pins (quello di M1PWM6)
	    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	    /// PWM non invertito
	    ROM_PWMOutputInvert(PWM1_BASE, PWM_OUT_6_BIT, false);

	    /// spegne i PWM
	    numPin = PWM_OUT_2;
	    MotorStop();

	}
	else{
		/// per ottenre una frequenza di 50 Hz necessaria ai servi, partendo da 80MHz di clock si deve dividere pe la costante 1.6e6
		/// che è ottenuta come prodotto tra 50000 (contenuto del registro di fine conteggio a 16 bit della cpu) e 32 (divisore del clock di sistema)
		/// 80000000 / (50000 * 32) = 50. Infatti il tempo di raggiungere 1600000 conteggi  e' di 20 ms
		/// Per ottenre una frequenza di 8kHz occorre invece dividere il clock per la costante 10000 = 32 * 312.5 e quindi il fine conteggio
		/// dovra' essere 312. Ci'o significa che la granularità del PWM e' di 1/312 = 0.32% inferiore a quanto riloevabile dai motori
		/// I valori sono quindi: 80000000 / (8000 * 32) = 80000000 / 256000 e quindi il fine conteggio e' 312.5
		NCont = ROM_SysCtlClockGet() / 1600000;
		delta = 0;

		/// Abilita le porte usate dal PWM, e per il comando della direzione dei motori,
		/// anche se potrebbero gia' esserlo stato inaltre parti del programma
		/// il secondo pwm motori deve abilitare PB4
		//ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		/// mentre PE4 e' una uscita digitale che combianta con il segnale PWM stabilisce la potenza al motore ed il verso di rotazione
		/// In pratica se PA5 = '0' e PB4 ha un PWM, supponendo che il motore giri in senso orario, allora
		/// quando PA5 = '1' e PB4 ha lo stesso PWM ma con polarita' invertita, il motore ruota in senso orario.

		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

		/// azzeramento delle uscite dei pin
		HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + (GPIO_PIN_3) << 2)) =  0;
		//HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + (GPIO_PIN_3 | GPIO_PIN_2) << 2)) =  0;
//	    //
//	    // Enable the GPIO port B
//	    //
//		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

		ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32);  //divisore per 32
		//ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //modulo pwm numero 0

		//Configure PF3 Pins as PWM
		//ROM_GPIOPinConfigure(GPIO_PB4_M0PWM5);  //generatore 1
		ROM_GPIOPinConfigure(GPIO_PF3_M1PWM7); // generatore 1
		///PE5 e' il modulo 1, mentre PB4 e' il modulo 2 del L298
		ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);


		//il generatore e' il numero 3
		//ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

		//Imposta il periodo del PWM rispetto al clock
		//ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, NCont);

		// Enable the PWM generator
		//ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);

		// Turn on the Output pins (quello di M0PWM2)
		ROM_PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
		/// VA FATTO L'OR CON L'ALTRO PIN????
		/// PWM non invertito
		ROM_PWMOutputInvert(PWM1_BASE, PWM_OUT_7_BIT, false);

		/// spegne i PWM
		numPin = PWM_OUT_3;
		MotorStop();

	}
}


void PWM_SERVI::MotorGo(int16_t gradi){
	switch (numServi){
	///
	/// attiva il motore su PA6 lo scarico del kit
	uint32_t valFin;
	case 1:

		valFin = convertDeg2Pwm(gradi);
		ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, valFin / 100);    //delta_1 e' del servo di movimento dello scrico del rescue kit
	break;

	case 2:

		valFin = convertDeg2Pwm(gradi);
		ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, valFin / 100);    //delta_1 e' del servo di movimento dello scrico del rescue kit
	}
}

///
/// funzione che converte i gradi in sewgnale pwm per i servomotori
uint32_t PWM_SERVI::convertDeg2Pwm(int16_t gradi){

	/// -90° -> delta_2 = 5; +90° -> delta_2 = 10
	float rot, delta;
	uint32_t valFin;
	rot = gradi / 36.0 + 7.50;
	delta = (float) NCont * rot;
	valFin = (uint32_t) delta;
	return valFin;
}

