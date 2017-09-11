/*
 * parse.c
 *
 *  Created on: 05/mar/2015
 *      Author: robotics
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "init.h"
#include "pid.h"

#include "sens_col_ir/sens.h"
#include "distMis.h"
#include "parse.h"
#include "uartp/uartstdio.h"
#include "pid.h"

#include "sensHum/HIH87Hum.h"

extern volatile uint8_t uart1buffer[], RX_PTR1, READ_PTR1;

void resetAutoma(syn_stat * STATO){
	STATO->ST = 0;
	STATO->cmd[0] = STATO->cmd[1] = 0;
	STATO->l_cmd = 0;
	STATO->valid = NON_VALIDO;
	STATO->token = ERRORE;
}

/*
 * 	L'analisi del messaggio avviene in due fasi: il parsing e la risposta al comando
 * 	Il parsing genera un token che e' successivamente usato nella risposta al comando.
 *
 */
///
/// analizza il comando che e' arrivato
///
void parse(syn_stat *STATO, PIDtoPWM *cmdPtr, syntaxStatus *synPtr){


	STATO->cmd[STATO->ST] = uart1buffer[READ_PTR1];

	/// La ricezione di un comando errato non produce il cambio di stato del mezzo.
	/// Infatti STATO->valid cambia a seguito di un comando corretto in questa funzione ma NON a seguito di un comando errato
	/// Lo stato può quindi diventare NON_VALIDO solo a seguito di time out o perche' gia' lo era.
	switch(STATO->ST){
	case 0:
		STATO->check = 0;
		STATO->token = ERRORE;
		if (STATO->cmd[0] >64 && STATO->cmd[0] < 91 ){
			/// una lettera MAIUSCOLA e quindi un comando di azione da raspberry
			STATO->l_cmd = 4;
			STATO->ST = 1;
			STATO->check = STATO->cmd[0];
		}
		//PRINTF("0-%d\n", uart1buffer[READ_PTR1]);
		/*else if(STATO->cmd[0] <= 16){
			/// comando di richiesta dati lungo due caratteri
			STATO->l_cmd = 2;
			STATO->ST = 1;
		}*/
	break;

	case 1:

		STATO->check ^= STATO->cmd[1];
		STATO->ST = 2;
		/// si analizza il checksum e poi si esegue il comando
		/*if (STATO->cmd[0] ^ CHECK_SUM == STATO->cmd[1]){
			// ok
			convertToToken(STATO);
			// resetta l'automa
			resetAutoma(STATO);
				if(STATO->token != ERRORE)
			/// il comando e' ora valido
			STATO->valid = VALIDO;
		}
*/
		//PRINTF("1-%d\n", uart1buffer[READ_PTR1]);
	break;

	case 2:
		STATO->check ^= CHECK_SUM;
		if(STATO->check == STATO->cmd[2]){
			/// ok, il messaggio e' valido
			convertToToken(STATO, cmdPtr);
			STATO->ST = 3;
		}
		else{
			STATO->ST = 0;
			STATO->valid = NON_VALIDO;
		}
		//PRINTF("2-%d\n", uart1buffer[READ_PTR1]);
	break;

	case 3:
		/// l'invio del comando e' fatto di 4 bytes e quindi passa di qui quando e' arrivato il IV byte cioe' quello
		/// del terminatore
		/// il comando e' ora valido
		STATO->valid = VALIDO;
		//PRINTF("3-%d\n", uart1buffer[READ_PTR1]);
		//PRINTF("cmd OK!\n\n");
	break;

	}
}


///
/// converte l'indicatore di un comando in un token
void convertToToken(syn_stat *STATO, PIDtoPWM *cmdPtr){

	switch(STATO->cmd[0]){
	case 'F':
		STATO->token = AVANTI;
		cmdPtr->azione = true;
		cmdPtr->numPid = 0;
		/// imposta il valore finale del PID: 5cm / s
		cmdPtr->valFin = 5.0;
		/// preparazione della risposta secondo il protocollo
		STATO->buff_reply[0] = 'F';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'F' ^ 'T' ^ '0' ^ CHECK_SUM;
	break;
	case 'B':
		STATO->token = INDIETRO;
		cmdPtr->azione = true;
		cmdPtr->numPid = 0;
		/// imposta il valore finale del PID: 5cm / s
		cmdPtr->valFin = 5.0;
		STATO->buff_reply[0] = 'B';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'B' ^ 'T' ^ '0' ^ CHECK_SUM;
	break;
	case 'S':
		STATO->token = STOP;
		cmdPtr->azione = true;
		cmdPtr->numPid = -1;
		STATO->buff_reply[0] = 'S';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'S' ^ 'T' ^ '0' ^ CHECK_SUM;;
	break;
	case 'R':
		STATO->token = DESTRA;
		cmdPtr->azione = true;
		cmdPtr->numPid = 1;
		/// imposta il valore finale del PID
		cmdPtr->valFin -= 90;
		STATO->buff_reply[0] = 'R';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'R' ^ 'T' ^ '0' ^ CHECK_SUM;
	break;
	case 'L':
		STATO->token = SINISTRA;
		cmdPtr->azione = true;
		cmdPtr->numPid = 2;
		/// imposta il valore finale del PID
		cmdPtr->valFin += 90;
		STATO->buff_reply[0] = 'L';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'L' ^ 'T' ^ '0' ^ CHECK_SUM;
	break;
	case 'I':
		STATO->token = GIRA_INDIETRO;
		cmdPtr->azione = true;
		/// potrebbe essere un "DESTRA" con valFIn = -180
		STATO->buff_reply[0] = 'I';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'I' ^ 'T' ^ '0' ^ CHECK_SUM;
	break;
	case 'G':
		//// lettura gradi di rotazione dal giroscopio
		STATO->token = MISURA_GRADI;
		cmdPtr->azione = false;
	break;
	case 'D':
		/// lettura sensore
		STATO->token = LETTURA_SENSORE;
		cmdPtr->azione = false;
	break;
	case 'P':
		//rilascio rescue pack
		STATO->token = RILASCIO_PACK;
		cmdPtr->azione = true;
		cmdPtr->avvia = true;
		STATO->buff_reply[0] = 'P';
		STATO->buff_reply[1] = 'T';
		STATO->buff_reply[2] = '0';
		STATO->buff_reply[3] = 'P' ^ 'T' ^ '0' ^ CHECK_SUM;

	default:
		/// se nessun comando e'giusto produce un errore.
		STATO->token = ERRORE;
		cmdPtr->numPid = -1;
	break;
	}

	/// potrebbe essere un comando nnuerico di richiesta dati
	/*if (STATO->cmd[0] <= 16){
		STATO->token = LETTURA_DATI;
	}*/
	/// azzera il contatore della persistenza del comando
	/// cio' vuol di re che c'e' un nuovo comando da servire
	STATO->tick = 0;
	/// resetta il contatore di timeout per permettere al comando di essere eseguito per almeno
	/// 1.5 s
	cmdPtr->tick = 0;

}



///
/// fornisce dati dai sensori a seguito di richiesta


void rispondiComando(syn_stat *sSTAT, ALLSTRUCT *collectedD){
	/// controlla se la sintazzi e' valida
	sSTAT->check = 0;
	/// controllo ridondante gia' effettuato
	if (sSTAT->valid == VALIDO){
		/// analizza il token e per il momento risposnde alle richieste di dati
		/// i tokens sono: LETTURA_SENSORE (con nuero di sensore in sSTAT.cmd[1])
		switch(sSTAT->token){

		/// le psecifiche prevedono che la misura dei gradi sia richiesta con il comando 'G'
		/// oppure con il comando 'D' seguito dall'intero 6 (lettura del sesto dato o sensore)
		case MISURA_GRADI:
			sSTAT->buff_reply[0] = 'G';
			sSTAT->buff_reply[1] = (collectedD->gyro->yaw  & 0xFF00) >> 8;
			sSTAT->buff_reply[2] = collectedD->gyro->yaw  & 0x00FF;

		break;

		/// fornisce la risposta alla lettura di un sensore
		case LETTURA_SENSORE:
			/// prepara solamente il buffer di risposta
			inviaSensore(sSTAT, collectedD);
		break;
		}

		/// se il dato e' valido viene trasmesso al richiedente (raspberry)
		if (sSTAT->dato_valido == 1){
			sSTAT->check  = 0;
		/// calcolo checksum
			for(int i = 0; i < 3; i++)
				/// calcola il checksum
				sSTAT->check ^= sSTAT->buff_reply[i];

			/// aggiunge la chiave 0xA9
			sSTAT->check ^= CHECK_SUM;
			sSTAT->buff_reply[3] = sSTAT->check;
			sSTAT->buff_reply[4] = '*';
		}
		/// invia i 4 byte su seriale. L'ultimo e' invato dalla funzione sendReply
		/// per la risposta ai comandi, il protocollo prevede:
		/// 'X' 'T/F' '0' oppure valore lettura , check_sum '*', quindi 4 byte
		sendReply(sSTAT, 4);
	}
	/// ripulisce il buffer di risposta
	for (int  i = 0; i < 5; i++)
		sSTAT->buff_reply[i] = 0;
}

///
/// invia i bytes di risposta sulla uart 1
void sendReply(syn_stat *sSTAT, uint8_t numChar){
	uint8_t i;
	/// invia tutti i caratterei nella stringa
	for (i= 0; i < numChar; i++)
		UARTCharPut(UART1_BASE, sSTAT->buff_reply[i]);
	//UARTCharPut(UART1_BASE, 'A' + i);

	/// infine invia il terminatore di stringa '*'
	UARTCharPut(UART1_BASE, '*');
	//UARTCharPut(UART1_BASE, 'A');
}

///
/// risponde alla richiesta di un sensore
void inviaSensore(syn_stat *sSTAT, ALLSTRUCT * collectedD){

	//TODO: ho aggiunto il puntatore alla struttura del colore, va aggiunto anche quello della temperatura?
	uint8_t  datoValido = 1;
	//analizzo il secondo byte, che è quello contentente l'ID del sensore
	//se ci dovesse essere un sensore in più quello va come case (0)
	switch(sSTAT->cmd[1]){
		// i sensori vengono numerati da quello davanti in senso antiorario
		// la misura dfornita e' in millimetri
		// sensore di distanza D1
		case(1):
			//risposta con ID del sensore
			sSTAT->buff_reply[0] = 1;
			sSTAT->buff_reply[1] = (collectedD->DSTptr->d_mm[0]  & 0xFF00) >> 8;
			sSTAT->buff_reply[2] = collectedD->DSTptr->d_mm[0]  & 0x00FF;

			break;

		//sensore di distanza DD2
		case(2):
			sSTAT->buff_reply[0] = 2;
			sSTAT->buff_reply[1] = (collectedD->DSTptr->d_mm[1]  & 0xFF00) >> 8;
			sSTAT->buff_reply[2] = collectedD->DSTptr->d_mm[1]  & 0x00FF;

			break;

		/// sensore di distanza anteriore
		case(3):
			sSTAT->buff_reply[0] = 3;
			sSTAT->buff_reply[1] = (collectedD->DSTptr->d_mm[2]  & 0xFF00) >> 8;
			sSTAT->buff_reply[2] = collectedD->DSTptr->d_mm[2]  & 0x00FF;

			break;

		case(4):
			sSTAT->buff_reply[0] = 4;
			sSTAT->buff_reply[1] = (collectedD->DSTptr->d_mm[3]  & 0xFF00) >> 8;
			sSTAT->buff_reply[2] = collectedD->DSTptr->d_mm[3]  & 0x00FF;

			break;

		case(5):
			sSTAT->buff_reply[0] = 5;
			sSTAT->buff_reply[1] = (collectedD->DSTptr->d_mm[4] & 0xFF00) >> 8;
			sSTAT->buff_reply[2] = collectedD->DSTptr->d_mm[4]  & 0x00FF;

			break;

		//se fosse presente anche il sensore nr 6
			//TODO: allargare il vettore d_mm di 1
//		case(0):
//			sSTAT->buff_reply[0] = 0;
//			sSTAT->buff_reply[1] = D->d_mm[5] ;
//			break;

		//lettura giroscopio (angolo tra 0 e 360°)
		case(6):
				sSTAT->buff_reply[0] = 6;
				sSTAT->buff_reply[1] = (collectedD->gyro->yaw & 0xFF00) >> 8;
				sSTAT->buff_reply[2] = collectedD->gyro->yaw & 0x00FF;
				break;

		//lettura luminosità (valore tra 0 e 255)
		case(7):
					sSTAT->buff_reply[0] = 7;
					sSTAT->buff_reply[1] = (collectedD->colorClass->luminanza & 0xFF00) >> 8;
					sSTAT->buff_reply[2] = collectedD->colorClass->luminanza & 0x00FF;
				break;

		//lettura temperatura (valore tra 20 e 40)
		case(8):
					sSTAT->buff_reply[0] = 8;
					sSTAT->buff_reply[1] = ((int)collectedD->temperat->Tint & 0xFF00) >> 8;
					sSTAT->buff_reply[2] = (int)collectedD->temperat->Tint & 0x00FF;
				break;

		//velocità (cm/s)
		case(9):
				sSTAT->buff_reply[0] = 9;
				//cast necessario, bisogna passare un intero
//				sSTAT->buff_reply[1] = ((int)collectedD->cinPtr->vel  & 0xFF00) >> 8;
//				sSTAT->buff_reply[2] = (int)collectedD->cinPtr->vel  & 0x00FF;
				sSTAT->buff_reply[1] = 1;
				sSTAT->buff_reply[2] = 0;
				break;

		//distanza percorsa (cm)
		case(10):
				sSTAT->buff_reply[0] = 10;
				//cast necessario, bisogna passare un intero
				sSTAT->buff_reply[1] = ((int)collectedD->encoder->dist_mm  & 0xFF00) >> 8;
				sSTAT->buff_reply[2] = (int)collectedD->encoder->dist_mm  & 0x00FF;
				break;

		/// inserire qui il codice per il senspre n. 11: il valore di angolo dell'accelerometro
		case(11):
				sSTAT->buff_reply[0] = 11;
				//cast necessario, bisogna passare un intero. Il valore restituito e' in
				// milli g cioe' in piano l'asse z da' 1.00xx
				// mentre in aInt[2] fornisce 1000
				sSTAT->buff_reply[1] = ((int)collectedD->acc->aInt[2]  & 0xFF00) >> 8;
				sSTAT->buff_reply[2] = (int)collectedD->acc->aInt[2]  & 0x00FF;
		break;


		/// gestisce sensori di tipo differente: permette al sw di gestire sensori arbitrari e non noti al
		/// momento della progettazione del sw
		case(12):
				HIH8_7Hum* humPtr = static_cast<HIH8_7Hum*> (collectedD->generic);
				sSTAT->buff_reply[0] = 11;
				sSTAT->buff_reply[1] = humPtr->buff[0];
				sSTAT->buff_reply[2] = humPtr->buff[1];
		break;

		default:
			sSTAT->buff_reply[0] = 0;
			sSTAT->buff_reply[1] = 0;
			sSTAT->buff_reply[2] = 0;
			datoValido = 0;
		break;
	}

	/// tiene nella struttura della sintassi se il dato richiesto e' valido oppure non lo e'
	if (datoValido == 1)
		sSTAT->dato_valido = 1;
	else
		sSTAT->dato_valido = 0;

}

///
/// stampa un numero float sull'uscita selezionata
uint8_t printFloat(double number, uint8_t digits){

  //size_t n = 0;

  // Handle negative numbers
  if (number < 0.0)
  {
     PRINTF("-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  PRINTF("%d", int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    PRINTF(".");
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    PRINTF("%d", toPrint);
    remainder -= toPrint;
  }


  return 0;
}

////
//// classe Parse

void Parse::parse(PIDtoPWM *p, syntaxStatus *s){

}

void Parse::convertToToken(syntaxStatus *s, PIDtoPWM * p){

}
