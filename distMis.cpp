/*
 * distMis.cpp
 *
 *  Created on: 21/feb/2016
 *      Author: massimo
 *
 *      modifica: giulio 21/apr/2016
			(prova approx in rawTomm con dist = k*pow(sens,m)
 */

/// in questa revisione sistemo le misure di distanza secondo quanto previsto dal protocollo di comunicazione.


#include "distMis.h"
#include "math.h"

distMis::distMis() {

	///imposta i parametri per l'interpolazione delle distanze dei vari sensori
	setM_Q();
	for (int i = 0; i < 6; i++){
		misSens[i] = 0.0;
		d_mm[i] = 0;
		dI[i] = 0;
		run = false;
		kf = 1.0;
	}

}

distMis::~distMis() {
	// TODO Auto-generated destructor stub
}


///
/// estrae dal dato grezzo la misura
/*
dI		mm	1/di		m				q
1600	600	0,000625	-1				2200
1700	500	0,000588235	-0,740740741	1759,259259
1835	400	0,000544959	-0,175438596	721,9298246
2120	350	0,000471698	-0,135135135	636,4864865
2490	300	0,000401606	-0,125			611,25
2890	250	0,000346021	-0,142857143	662,8571429
3240	200	0,000308642	-0,217391304	904,3478261
3470	150	0,000288184	0,064935065		-75,32467532
2700	100	0,00037037
*/

void distMis::rawTomm(){

	//   NUMERAZIONE DEI SENSORI DI DISTANZA
	//          0
	//       +--!--+
	//     1 <     > 3
	//       |     |
	//     2 <     > 4
	//
	/// conversione del dato grezzo in mm

	/// sensore S0
	for (int i = 0; i < 8; i++){
		if (dI[0] < 1600){
			d_mm[0] = 650.0;
			break;
		}
		if (dI[0] >= 1600 && dI[0] < 1700){
			d_mm[0] = M[0][i] * dI[0] + Q[0][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[0] >= 1700 && dI[0] < 1835){
			d_mm[0] = M[0][i] * dI[0] + Q[0][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[0] >= 1835 && dI[0] < 2120){
			d_mm[0] = M[0][i] * dI[0] + Q[0][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[0] >= 2120 && dI[0] < 2490){
			d_mm[0] = M[0][i] * dI[0] + Q[0][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[0] >= 2490 && dI[0] < 3240){
			d_mm[0] = M[0][i] * dI[0] + Q[0][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[0] >= 3240 && dI[0] < 3470){
			d_mm[0] = M[0][i] * dI[0] + Q[0][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[0] >= 3470){
			d_mm[0] = 120.0;
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
	}

	/// sensore S1
	for (int i = 0; i < 7; i++){
		if (dI[1] < 1000){
			d_mm[1] = 150.0;
			break;
		}
		if (dI[1] >= 1000 && dI[1] < 1140){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 1140 && dI[1] < 1400){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 1400 && dI[1] < 1710){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 1710 && dI[1] < 2100){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 2100 && dI[1] < 2690){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 2690 && dI[1] < 3050){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 3050 && dI[1] < 3550){
			d_mm[1] = M[1][i] * dI[1] + Q[1][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[1] >= 3550){
			d_mm[1] = 40.0;
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
	}


	/// sensore S2
	for (int i = 0; i < 7; i++){
		if (dI[2] < 960){
			d_mm[2] = 160.0;
			break;
		}
		if (dI[2] >= 960 && dI[2] < 1120){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 1120 && dI[2] < 1340){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 1340 && dI[2] < 1660){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 1660 && dI[2] < 2050){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 2050 && dI[2] < 2700){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 2700 && dI[2] < 3120){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 3120 && dI[2] < 3500){
			d_mm[2] = M[2][i] * dI[2] + Q[2][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[2] >= 3500){
			d_mm[2] = 40.0;
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
	}

	/// sensore S3
	for (int i = 0; i < 7; i++){
		if (dI[3] < 940){
			d_mm[3] = 160.0;
			break;
		}
		if (dI[3] >= 940 && dI[3] < 1093){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 1093 && dI[3] < 1300){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 1300 && dI[3] < 1575){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 1575 && dI[2] < 2035){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 2035 && dI[3] < 2664){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 2664 && dI[3] < 3150){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 3150 && dI[3] < 3700){
			d_mm[3] = M[3][i] * dI[3] + Q[3][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[3] >= 3700){
			d_mm[3] = 40.0;
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
	}


	/// sensore S4
	for (int i = 0; i < 7; i++){
		if (dI[4] < 944){
			d_mm[4] = 160.0;
			break;
		}
		if (dI[4] >= 944 && dI[4] < 1070){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 1070 && dI[4] < 1195){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 1195 && dI[4] < 1420){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 1420 && dI[4] < 1820){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 1820 && dI[4] < 2450){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 2450 && dI[4] < 2980){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 2980 && dI[4] < 3500){
			d_mm[4] = M[4][i] * dI[4] + Q[4][i];
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
		if (dI[4] >= 3500){
			d_mm[4] = 40.0;
			/// trovato l'intervallo di conversione può uscire dal for
			break;
		}
	}

}

void distMis::setM_Q(){
	/*
	dI		mm	1/di		m				q
	1600	600	0,000625	-1				2200
	1700	500	0,000588235	-0,740740741	1759,259259
	1835	400	0,000544959	-0,175438596	721,9298246
	2120	350	0,000471698	-0,135135135	636,4864865
	2490	300	0,000401606	-0,125			611,25
	2890	250	0,000346021	-0,142857143	662,8571429
	3240	200	0,000308642	-0,217391304	904,3478261
	3470	150	0,000288184	0,064935065		-75,32467532
	2700	100	0,00037037
	*/
	{
/// PARAMETRI PER IL SENSORE 0;
		float a[] = {
				1600,
				1700,
				1835,
				2120,
				2490,
				2890,
				3240,
				3470,
				2700
		};

		float b[] = {
				600,
				500,
				400,
				350,
				300,
				250,
				200,
				150,
				100
		};

		for (int i = 0; i < 8; i++){
			M[0][i] = (b[i+1] - b[i]) / (a[i+1] - a[i]);
			Q[0][i] = b[i] - M[0][i] * a[i];
		}
	}

	{
	/*
	 * PARAMETRI PER IL SENSORE 1
	dI		mm	1/dI		mm
S1	1000	160	0,001		160
	1140	140	0,000877193	140
	1400	120	0,000714286	120
	1710	100	0,000584795	100
	2100	80	0,00047619	80
	2690	60	0,000371747	60
	3050	50	0,000327869	50
	3550	40	0,00028169	40
	 *
	 * */
	/// PARAMETRI PER IL SENSORE 1;
		float a[] = {
				1000,
				1140,
				1400,
				1710,
				2100,
				2690,
				3050,
				3550
		};

		float b[] = {
				160,
				140,
				120,
				100,
				80,
				60,
				50,
				40
		};
		for (int i = 0; i < 7; i++){
			M[1][i] = (b[i+1] - b[i]) / (a[i+1] - a[i]);
			Q[1][i] = b[i] - M[1][i] * a[i];
		}
	}

	{
		/*
		PARAMETRI PER IL SENSORE S2
	dI		mm		1/dI
S2	960		160		0,001041667
	1120	140		0,000892857
	1340	120		0,000746269
	1660	100		0,00060241
	2050	80		0,000487805
	2700	60		0,00037037
	3120	50		0,000320513
	3500	40		0,000285714

		 * */

	/// PARAMETRI PER IL SENSORE 2;
		float a[] = {
				960,
				1120,
				1340,
				1660,
				2050,
				2700,
				3120,
				3500

		};

		float b[] = {
				160,
				140,
				120,
				100,
				80,
				60,
				50,
				40
		};
		for (int i = 0; i < 7; i++){
			M[2][i] = (b[i+1] - b[i]) / (a[i+1] - a[i]);
			Q[2][i] = b[i] - M[2][i] * a[i];
		}

	}


	{
	/*
	PARAMETRI PER IL SENSORE S3
	dI		mm
S3	940		160
	1093	140
	1300	120
	1575	100
	2035	80
	2664	60
	3150	50
	3700	40


	 * */
		float a[] = {
				940,
				1093,
				1300,
				1575,
				2035,
				2664,
				3150,
				3700


		};

		float b[] = {
				160,
				140,
				120,
				100,
				80,
				60,
				50,
				40
		};
		for (int i = 0; i < 7; i++){
			M[3][i] = (b[i+1] - b[i]) / (a[i+1] - a[i]);
			Q[3][i] = b[i] - M[3][i] * a[i];
		}
	}

	{
	/*
	 PARAMETRI PER IL SENSORE 4
	dI		mm
S4	944		160
	1070	140
	1195	120
	1420	100
	1820	80
	2450	60
	2980	50
	3500	40
	3400	30


	 * */
		float a[] = {
				944,
				1070,
				1195,
				1420,
				1820,
				2450,
				2980,
				3500,
				3400


		};

		float b[] = {
				160,
				140,
				120,
				100,
				80,
				60,
				50,
				40
		};
		for (int i = 0; i < 7; i++){
			M[4][i] = (b[i+1] - b[i]) / (a[i+1] - a[i]);
			Q[4][i] = b[i] - M[4][i] * a[i];
		}

	}
}
