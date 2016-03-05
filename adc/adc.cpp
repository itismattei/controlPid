/*
 * adc.c
 *
 *  Created on: 15/mar/2015
 *      Author: robotics
 */

#include <stdint.h>
#include "adc.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"


#define ADC_SEQ                 (ADC_O_SSMUX0)
#define ADC_SEQ_STEP            (ADC_O_SSMUX1 - ADC_O_SSMUX0)
#define ADC_SSMUX               (ADC_O_SSMUX0 - ADC_O_SSMUX0)
#define ADC_SSEMUX              (ADC_O_SSEMUX0 - ADC_O_SSMUX0)
#define ADC_SSCTL               (ADC_O_SSCTL0 - ADC_O_SSMUX0)
#define ADC_SSFIFO              (ADC_O_SSFIFO0 - ADC_O_SSMUX0)
#define ADC_SSFSTAT             (ADC_O_SSFSTAT0 - ADC_O_SSMUX0)
#define ADC_SSOP                (ADC_O_SSOP0 - ADC_O_SSMUX0)
#define ADC_SSDC                (ADC_O_SSDC0 - ADC_O_SSMUX0)


void initAdc(volatile distanza *d){
	int i;
	for (i = 0; i < 6; i++){
		d->misSens[i] = 0.0;
		d->d_mm[i] = 0;
		d->dI[i] = 0;
	}
	initHW_ADC();
	/// setta ad on al presenza e funzionalita' del sensore
	d->run = true;
}

///
/// estrae i dati  dopo la lettura dei 5 sensori
int32_t ADCSequenceData_Get(uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t pADCBuffer[]){

    uint32_t ui32Count;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Get the offset of the sequence to be read.
    //
    ui32Base += ADC_SEQ + (ADC_SEQ_STEP * ui32SequenceNum);

    //
    // Read samples from the FIFO until it is empty.
    //
    volatile int i = 0;
    ui32Count = 0;
    while(!(HWREG(ui32Base + ADC_SSFSTAT) & ADC_SSFSTAT0_EMPTY) &&
          (ui32Count < 10))
    {

        //
        // Read the FIFO and copy it to the destination.
        //
        *pADCBuffer++ = HWREG(ui32Base + ADC_SSFIFO);
    	//pADCBuffer[i]  = HWREG(ui32Base + ADC_SSFIFO) & 0xFFF;
    	//pADCBuffer++;
    	//pADCBuffer[i] = 5 + i;
    	//risultato[i] = HWREG(ui32Base + ADC_SSFIFO) & 0xFFF;
    	//i++;
        //
        // Increment the count of samples read.
        //
        ui32Count++;
    }

    //
    // Return the number of samples read.
    //
    return(ui32Count);
}


extern  distanza *dPtr;
#include "../distMis.h"
extern  distMis *misPtr;
volatile extern  int ADCflag;
volatile uint32_t numByte;

#ifdef __cplusplus
extern "C" {
#endif



void adcISR(void){
	volatile uint32_t attesa;
	ADCIntClear(ADC0_BASE, 0);
	numByte = ADCSequenceData_Get(ADC0_BASE, 0, dPtr->dI);    // Read ADC Value.
	/// finito di trascrivere i dati, spegne il pin di segnalazione
	HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) &=  ~GPIO_PIN_5;
	/// riavvia il campionamento
	//HWREG(ADC0_BASE + ADC_O_PSSI) |= ((2 & 0xffff0000) | (1 << (2 & 0xf)));
	///riutilizzo della variabile attesa per stampare a video i dati provenienti dal buffer dell'adc
//	for(attesa= 1; attesa < 6; attesa++){
//		PRINTF("val: %d \t", dPtr->dI[attesa]);
//		misPtr->dI[attesa] = dPtr->dI[attesa];
//	}
//	PRINTF("\n");
//	misPtr->dI[0] = dPtr->dI[0];

	ADCflag = 1;
}


#ifdef __cplusplus
}
#endif

/*void Timer0ISR(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, 0);
}*/



void initHW_ADC(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
//SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ADCReferenceSet(ADC0_BASE, ADC_REF_INT); //Set reference to the internal reference ,You can set it to 1V or 3 V
//ADCReferenceSet(ADC1_BASE, ADC_REF_INT);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //Ain0
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); //Ain1
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); //Ain2
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); //Ain3
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3); //Ain4
	/// imposta il sequencer 0, che ha 8 letture
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

	//i sensori vengono numerati da quello davanti in senso antiorario

	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	/// PE.2
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);

	// PE.1
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2 );
	//GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); //Ain3
	// PE.0
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3 );
	/// PD.3
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH4 );
	/// PD.2
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);
	/// abilita il sequencer 0
	ADCSequenceEnable(ADC0_BASE, 0);

	/// abilta l'interruzione del sequencer 0
	ADCIntClear(ADC0_BASE, 0);
    //
    // Enable the ADC interrupt.
    //
    IntEnable(INT_ADC0SS0);
    ADCIntEnable(ADC0_BASE, 0);
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

}
