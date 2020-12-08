/* Programm zum Lesen und Vearbeiten der Gyroskop und Beschleunigungssensor Werte*/


#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include "sensor.cpp" //  Funktionen zum Lesen der I2c Schnittstelle
#include <iostream>
#include <iomanip>
#include "LSM9DS0.h" //Einbinden der RegisterAdressen



using namespace std;


#define DT 0.02         // [s/loop] Zeit eines Loops: 20 ms
#define AA 0.95         // Filter Konstante des Komplementärfilters
//#define BB 0.8

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846
#define ME 12 // Parameter zum Erkennen eines Bewegungsendes


	int z=0;
	int sprung=0;
	int freqtest=0;
	int countx=0;
	int dynfilterrev=0; // Variablen die für dynamischen Filteranpassung verwendet wurden
	int dynfilterfor=0;
	int dynfilterreset=0;
	float offsetTolerance=0.0; // Zum Umrechnen in M/s²
	float unfilteredACCx=0.0;
	float BB=0.98;

	float act_acc_x = 0.0;
	float act_acc_y = 0.0;
	float act_acc_z = 0.0;

	float vel_acc_x = 0.0; // Variablen um die Beschleunigungen
	float vel_acc_y = 0.0; // in der Kalibrierungsfunktion aufzuaddieren
	float vel_acc_z = 0.0;

	float offset_acc_x =0.0; //Offset der durch die Kalibrierungsfunktion bestimmt wurde.
	float offset_acc_y =0.0;
	float offset_acc_z =0.0;

	float spinrate_x=0.0; // Variablen um die Drehraten
	float spinrate_y=0.0; // in der Kalibrierungsfunktion des Gyroskopes aufzuaddieren
	float spinrate_z=0.0;

	float gyro_offset_x=0.0; // Gyro Offset der durch die Kalibrierungsfunktion bestimmt wurde
	float gyro_offset_y=0.0;
	float gyro_offset_z=0.0;

	float RangleX=0.0;	// RotationsWinkel X-Achse (Roll)
	float RangleY=0.0;	// rotationswinkel Y-Achse (Pitch)
	float RangleZ=0.0;  // Rotationswinkel Z-Achse (Yaw)

	float OutputAcc_x=0.0; // Speichervariablen zur Verarbeitung der Beschleunigungen
	float OutputAcc_y=0.0;
	float OutputAcc_z=0.0;

	//Gravitationsvektor::
	float globalgrav_x = 0.0; //Gravitationsvektor im globalen Koordinatensystem
	float globalgrav_y=0.0;
	float globalgrav_z=0.0;

	float debugVAR1 =0.0;  // DEBUG
	float debugVAR2 =0.0;
	float debugVAR3 =0.0;

	float globalAcc_x=0.0;	// Reale Beschleunigung ohne Gravitationsvektor
	float globalAcc_y=0.0;
	float globalAcc_z=0.0;

	float globalAcc_buff_x=0.0; // Buffer für Approximation 1. Ordnung
	float globalAcc_buff_y=0.0;
	float globalAcc_buff_z=0.0;
	float globalSpeed_buff_x=0.0;
	float globalSpeed_buff_y=0.0;
	float globalSpeed_buff_z=0.0;

	//buffer für moving average lp filter
	float emaLPbuff_x=0.0;
	float emaLPbuff_y=0.0;
	float emaLPbuff_z=0.0;

	// EMA gefilterte Beschleunigungswerte
	float emaAcc_x=0.0;
	float emaAcc_y=0.0;
	float emaAcc_z=0.0;
	// Buffer für ema filter Beschleunigungssensor
	float emaAccBuff_x=0.0;
	float emaAccBuff_y=0.0;
	float emaAccBuff_z=0.0;
	// Winkel mit Atan2 EMA filter
	float emaAccAngleX=0.0;
	float emaAccAngleY=0.0;


	float CF2angleX=0.0;
	float CF2angleY=0.0;

	float rotacc_x=0.0;
	float rotacc_y=0.0;
	float rotacc_z=0.0;

	float rueckacc_x=0.0;
	float rueckacc_y=0.0;
	float rueckacc_z=0.0;



	// Funktion zum Kalibrieren des BEschleunigungssensors:
	// die Werte der einzelnen Achsen werden 1024 mal aufgezeichnet und gemittelt.
	// dieser wert bestimmt den Offset

	void gyrocalibration(float ox1, float ox2, float ox3){
		spinrate_x += ox1;
		spinrate_y += ox2;
		spinrate_z += ox3;
	}
	void calibrateAcc(float ox1,float ox2,float ox3){
			vel_acc_x= vel_acc_x + ox1;
			vel_acc_y= vel_acc_y + ox2;
			vel_acc_z= vel_acc_z + ox3;
	}




void  INThandler(int sig) //Interrupt Handling
{
        signal(sig, SIG_IGN);
        exit(0);
}

int mymillis() // Funktionen zur Zeitbestimmung
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;
    return (diff<0);
}

// Funktion um gegebene Beschleunigungen in ein globales Koordinatensystem zu übertragen
// Hierzu werden die Roll, Pitch und Yaw Winkel und Beschleunigungen angenommen
void rotieren(float RangleX,float RangleY,float RangleZ, float outputX,float outputY, float outputZ){
	//rotieren um X
	RangleX=RangleX*M_PI/180;
	float temp = outputY;
	outputY= outputY *cos(RangleX)-outputZ*sin(RangleX);
	outputZ= temp*sin(RangleX)+ outputZ*cos(RangleX);
	//rotieren um y
	RangleY=RangleY*M_PI/180;
	temp=outputZ;
	outputZ=outputZ*cos(RangleY)-outputX*sin(RangleY);
	outputX= temp*sin(RangleY)+outputX*cos(RangleY);
	//rotieren um Z
	RangleZ=RangleZ*M_PI/180;
	temp =outputX;
	outputX= outputX*cos(RangleZ)-outputY*sin(RangleZ);
	outputY= temp*sin(RangleZ)+ outputY*cos(RangleZ);

	//Gesamte Beschleunigung in Globales System:
	OutputAcc_x=outputX;
	OutputAcc_y=outputY;
	OutputAcc_z=outputZ;
/*
	rotacc_x=outputX;
	rotacc_y=outputY;
	rotacc_z=outputZ;
*/
		outputX=0;
		outputY=0;
		outputZ=0;
}
// Diese Funktion wurde getestet und dreht einen Vektor in ein mobiles System
// zu drehen. Für das Programm wurde die rotieren() Funktion verwendet
void globalgrav(float RangleX,float RangleY,float RangleZ,float outputX,float outputY,float outputZ){
	//rotieren um X
	RangleX=RangleX*M_PI/180;
	float temp = outputY;
	outputY= outputY *cos(RangleX)-outputZ*sin(RangleX);
	outputZ= temp*sin(RangleX)+ outputZ*cos(RangleX);
	//rotieren um y
	RangleY=RangleY*M_PI/180;
	temp=outputZ;
	outputZ=outputZ*cos(RangleY)-outputX*sin(RangleY);
	outputX= temp*sin(RangleY)+outputX*cos(RangleY);
	//rotieren um Z/*
	RangleZ=RangleZ*M_PI/180;
	temp =outputX;
	outputX= outputX*cos(RangleZ)-outputY*sin(RangleZ);
	outputY= temp*sin(RangleZ)+ outputY*cos(RangleZ);

	globalgrav_x = outputX;
	globalgrav_y=  outputY;
	globalgrav_z=  outputZ;


	outputX=0;
	outputY=0;
	outputZ=0;

}
// andere Konvention wurde getestet- nicht relevant.
void ruecktrans(float RangleX,float RangleY,float RangleZ,float outputX,float outputY,float outputZ){
	float temp;
	//rotieren um Z/*
	RangleZ=RangleZ*M_PI/180;
	temp =outputX;
	outputX= outputX*cos(RangleZ)+outputY*sin(RangleZ);
	outputY= temp*-sin(RangleZ)+ outputY*cos(RangleZ);
	//rotieren um y
	RangleY=RangleY*M_PI/180;
	temp=outputZ;
	outputZ=outputZ*cos(RangleY)+outputX*sin(RangleY);
	outputX= temp*-sin(RangleY)+outputX*cos(RangleY);
	//rotieren um X
	RangleX=RangleX*M_PI/180;
	temp=outputY;
	outputY= outputY *cos(RangleX)+outputZ*sin(RangleX);
	outputZ= temp*-sin(RangleX)+ outputZ*cos(RangleX);

	//OutputAcc_x=outputX;
	//OutputAcc_y=outputY;
	//OutputAcc_z=outputZ;

	globalgrav_x = outputX;
	globalgrav_y=  outputY;
	globalgrav_z=  outputZ;
}



int main(int argc, char* argv[])
{

	float rate_gyr_y = 0.0;   // [deg/s]  DrehRate Gyroskop
	float rate_gyr_x = 0.0;    // [deg/s]
	float rate_gyr_z = 0.0;     // [deg/s]

	int  accRaw[3]; // Variablen zum Einlesen der Rohwerte
	int  magRaw[3];
	int  gyrRaw[3];

	float gyroXangle = 0.0;	//Winkel des Gyroskops
	float gyroYangle = 0.0;
	float gyroZangle = 0.0;
	float AccYangle = 0.0;	// Winkel errechnet aus Beschleunigungssensor
	float AccXangle = 0.0;
	float CFangleX = 0.0;	// Winkel nach Komplementärfilter
	float CFangleY = 0.0;

	float runtime=0.0; //Laufzeit des Programms


	float acc_buff_x=0.0; //zwischenspeicher für 1. order approx
	float acc_buff_y=0.0;
	float acc_buff_z=0.0;
	float speed_buff_x=0.0;
	float speed_buff_y=0.0;
	float speed_buff_z=0.0;


	float speed_acc_x=0.0; //Erechnete Geschwindigkeit durch Integration der Beschleunigung
	float speed_acc_y=0.0;
	float speed_acc_z=0.0;

	float positionx=0.0; //Errechnete Position durch Integration der Geschwindigkeit
	float positiony=0.0;
	float positionz=0.0;

	int i=0;
	int count2=0;
	int j=0,k=0,l=0; //movement end


//SCHRITTERKENNUNG
	int stepcountA=0;
	int realstep=0;
	int steptimedifference=0; //Zeit zwischen zwei möglichen Schritten
	float stepdistance=0; //Durch SChritte zurückgelegte Strecke
//////////////////////////////////////


	int startInt  = mymillis();
	struct  timeval tvBegin, tvEnd,tvDiff;
    signal(SIGINT, INThandler);

	enableIMU();  //Initialisieren der IMU, siehe Sensor.cpp

	gettimeofday(&tvBegin, NULL);



	cout << "** Kalibriere **" << endl;

	readACC(accRaw);
	readGYR(gyrRaw);
	//Calibrate(accRaw[0],accRaw[1],accRaw[2]); //Zum Berechnen des Offsets Beschleunigungssensor

	//KALIBRIERUNG des Beschleunigungssensors
	do{
		readACC(accRaw);
		calibrateAcc(accRaw[0],accRaw[1],accRaw[2]);
		countx++;
	}while(countx<=1024);
	offset_acc_x= vel_acc_x/1024;
	offset_acc_y= vel_acc_y/1024;
	offset_acc_z = vel_acc_z/1024;
	offsetTolerance=(offset_acc_z/9.81);
	countx=0;
	//Kalibrierung des Gyroskops:
	do{
		readGYR(gyrRaw);
		gyrocalibration(gyrRaw[0],gyrRaw[1],gyrRaw[2]);
		countx++;
	}while(countx<=1024);
	gyro_offset_x = spinrate_x/1024;
	gyro_offset_y = spinrate_y/1024;
	gyro_offset_z = spinrate_z/1024;

	sleep(0.01);

	while(1)
	{
	startInt = mymillis();

	//read ACC and GYR data
	readACC(accRaw);
	readGYR(gyrRaw);

	//Gyroskop Rohwerte werden zu Grad/sekunde konvertiert.
	// Umrechnungsfaktor entsprechend der Auflösung
	rate_gyr_x = (float) (gyrRaw[0]-gyro_offset_x) * G_GAIN;
	rate_gyr_y = (float) (gyrRaw[1]-gyro_offset_y)  * G_GAIN;
	rate_gyr_z = (float) (gyrRaw[2]-gyro_offset_z)  * G_GAIN;

/*	// aktuelle Kräfte BEschleunigungssensor ungefiltert
	act_acc_x = (float) accRaw[0];//-offset_acc_x;
	act_acc_y = (float) accRaw[1];//-offset_acc_y;
	act_acc_z = (float) accRaw[2];//-offset_acc_z;
*/

	// Low pass filter ///******************************************

	// * Typ: gleitender Mittelwert(Simple moving average)
	// * Berechnet den durchschnittswert der Beschleunigung von einer konstanten Anzahl an Werten
/*
	do{
		readACC(accRaw);
		act_acc_x=(float)(act_acc_x+accRaw[0])-offset_acc_x;
		act_acc_y=(float)(act_acc_y+accRaw[1])-offset_acc_y;
		act_acc_z=(float)(act_acc_z+accRaw[2])-offset_acc_z;
		i++;
	}while(i<8);
	act_acc_x=act_acc_x/2;
	act_acc_y=act_acc_y/2;
	act_acc_z=act_acc_z/2;
	i=0;
*/
	// Test zum Vergleich mit ungefilterten Werten:
	unfilteredACCx=accRaw[0];

	//alternativer LP Filter ohne offset  **letzte**

	do{
		readACC(accRaw);
		act_acc_x=(float)(act_acc_x+accRaw[0]);
		act_acc_y=(float)(act_acc_y+accRaw[1]);
		act_acc_z=(float)(act_acc_z+accRaw[2]);
		i++;
	}while(i<25);
	act_acc_x=act_acc_x/25;
	act_acc_y=act_acc_y/25;
	act_acc_z=act_acc_z/25;
	i=0;

	/*
	 * Exponential Moving Average Filter für Beschleunigung:
	 * Der Output ist die gewichtete Summe von neugelesenen Sensorwerten
	 * und dem alten Output des Filters.
	 */
	//float alpha=0.98;


/*	//MIT OFFSET
	act_acc_x=(float)(1-alpha)*act_acc_x+alpha*(accRaw[0]-offset_acc_x);
	act_acc_y=(float)(1-alpha)*act_acc_y+alpha*(accRaw[1]-offset_acc_y);
	act_acc_z=(float)(1-alpha)*act_acc_z+alpha*(accRaw[2]-offset_acc_z);
*/
	// OHNE OFFSET
	/*
	act_acc_x=(float)(1-alpha)*act_acc_x+alpha*accRaw[0];
	act_acc_y=(float)(1-alpha)*act_acc_y+alpha*accRaw[1];
	act_acc_z=(float)(1-alpha)*act_acc_z+alpha*accRaw[2];
*/
	// Beschleunigungssensorrohwerte werden gefiltert mit Exponential Moving Average für CFilter
			/*float alpha=0.01;

			emaAcc_x=(float)(1-alpha)*emaAcc_x+alpha*(accRaw[0]);
			emaAcc_y=(float)(1-alpha)*emaAcc_y+alpha*(accRaw[1]);
			emaAcc_z=(float)(1-alpha)*emaAcc_z+alpha*(accRaw[2]);
*/
			//LP Filter, sma ema kombination:
/*
			do{
					emaAcc_x=(float)(emaAcc_x+accRaw[0]);
					emaAcc_y=(float)(emaAcc_y+accRaw[1]);
					emaAcc_z=(float)(emaAcc_z+accRaw[2]);
					i++;
				}while(i<512);
				emaAcc_x=emaAcc_x/512;
				emaAcc_y=emaAcc_y/512;
				emaAcc_z=emaAcc_z/512;
				i=0;
*/


/*	// berechne geschwindigkeit im Beschleinugungssensor
	vel_acc_x+=(act_acc_x)*DT;
	vel_acc_y+=(act_acc_y)*DT;
	vel_acc_z+=(act_acc_z)*DT;
*/


	//Winkel/Sekunde des Gyros werden Integriert um den aktuellen Winkel zu bestimmen.
	gyroXangle+=rate_gyr_x*DT;
	gyroYangle+=rate_gyr_y*DT;
	gyroZangle+=rate_gyr_z*DT;

	//Rohwerte des Beschleunigungssensors werden verwendet
	//um mittels Atan2 die Neigung des Sensors zu bestimmen.
	// Umrechnung in grad
	AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
	AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;

	emaAccAngleX = (float) (atan2(emaAcc_y,emaAcc_z))*RAD_TO_DEG;
	emaAccAngleY = (float) (atan2(emaAcc_z,emaAcc_x)-M_PI/2)*RAD_TO_DEG;

    // Wenn die IMU andersherum montiert wird kann diese Umrechnung verwendet werden:
	/*
        if (AccXangle >180)
                AccXangle -= (float)360.0;

        AccYangle-=90;
        if (AccYangle >180)
                AccYangle -= (float)360.0;
	*/

        //Bei korrekter montierung:
        AccXangle -= (float)180.0;
	if (AccYangle > 90)
	        AccYangle -= (float)270;
	else
		AccYangle += (float)90;


	//Komplementärfilter um die Werte des Gyroskops und Beschleunigungssensor
	//zur Neigungsbestimmung zu kombinieren.
	CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
	CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;

	CF2angleX=BB*(CF2angleX+rate_gyr_x*DT) +(1 - BB) * emaAccAngleX;
	CF2angleY=BB*(CF2angleY+rate_gyr_y*DT) +(1 - BB) * emaAccAngleY;




	//Komplementärfilter mit ExponentialMovingAverage im Beschleunigungssensor

	// Loopslänge festgelegt auf 20 ms, daher Laufzeit:
	runtime=runtime+0.02;
	freqtest++;

	//Die LP gefilterten Werte (act_acc_) werden zu m/s² umgerechnet. Offsettolerance
	//bestimmt den umrechnungsfaktor. Dieser ist abhängig des Offsets(referenz :z offset = 9.81 m/s²)
	OutputAcc_x=act_acc_x/offsetTolerance;
	OutputAcc_y=act_acc_y/offsetTolerance;
	OutputAcc_z=act_acc_z/offsetTolerance;  //~1655

/*  // Filter test: SMA fragt 128 mal nach dem Wert. Es sollte herausgefunden werden
 * wie viele verschiedene Werte enthalten sind.
	if(freqtest==50){
		do{
				readACC(accRaw);
				act_acc_x=(float)(act_acc_x+accRaw[0])-offset_acc_x;
				act_acc_y=(float)(act_acc_y+accRaw[1])-offset_acc_y;
				act_acc_z=(float)(act_acc_z+accRaw[2])-offset_acc_z;
				i++;
				cout << accRaw[0] <<" " <<accRaw[1]<< " " << accRaw[2] << " " <<i<< endl;
			}while(i<128);
			act_acc_x=act_acc_x/128;
			act_acc_y=act_acc_y/128;
			act_acc_z=act_acc_z/128;
			i=0;

	}
*/

	debugVAR1=OutputAcc_x; // zum debuggen
	debugVAR2=OutputAcc_y;
	debugVAR3=OutputAcc_z;

	// ROTATION IN GLOBALES KOORDINATENSYSTEM :::::::::::::::::::::::::
	//Beschleunigungen in globales Koordinatensystem

	//globaler gravitationsvektor:
	//ruecktrans(CFangleX,CFangleY,0,OutputAcc_x,OutputAcc_y,OutputAcc_z);
	//ruecktrans(70,15,60,0,0,9.81);
	//globalgrav(CFangleX,CFangleY,0,0,0,9.81);

//	OutputAcc_z-=offset_acc_z/offsetTolerance;
/*
	OutputAcc_x=OutputAcc_x-globalgrav_x;
	OutputAcc_y=OutputAcc_y-globalgrav_y;
	OutputAcc_z=OutputAcc_z-globalgrav_z;
*/
	//rotieren(CFangleX,CFangleY,0,OutputAcc_x,OutputAcc_y,OutputAcc_z);

	//OutputAcc_z=OutputAcc_z-9.81;
/*
	globalAcc_x=OutputAcc_x+globalgrav_x;
	globalAcc_y=OutputAcc_y+globalgrav_y;
	globalAcc_z=-OutputAcc_z+globalgrav_z;
*/

	/*
globalAcc_x=OutputAcc_x-globalgrav_x;
globalAcc_y=OutputAcc_y-globalgrav_y;
globalAcc_z=OutputAcc_z-globalgrav_z;
*/
/*
globalAcc_x=rotacc_x+globalgrav_x;
globalAcc_y=rotacc_y+globalgrav_y;
globalAcc_z=rotacc_z-globalgrav_z;
*/
	/*//GLOBAL ACC WINDOW//
	if((globalAcc_z<=0.25)&&(globalAcc_z>=-0.25)){
			globalAcc_z=0.0;
		}
		if((globalAcc_x<=0.09)&&(globalAcc_x>=-0.09)){
			globalAcc_x=0.0;
			}
		if((globalAcc_y<=0.2)&&(globalAcc_y>=-0.2)){
			globalAcc_y=0.0;
			}
	//////////////////
*/
	//Dynamische Anpassung der Filterkonstanten
	// Bremsbewegung wird erkannt und damit die Filterkonstante geändert
	if(OutputAcc_x<=-0.2){
		dynfilterrev++;
	}
	if (dynfilterrev==8){
		BB=0.998;
	}
	if(dynfilterrev==14){
		BB=0.5;
		dynfilterrev=0;
	}
	if(OutputAcc_x>=0.3){
		dynfilterfor++;
	}
	if(dynfilterfor==5){
		 BB=0.95;
		 dynfilterfor=0;
		 dynfilterrev=0;
	}
	if(BB==0.5){
		dynfilterreset++;
		if (dynfilterreset==10){
			BB=0.998;
			dynfilterreset=0;
		}
	}


	///////////window ///////////////////
/*
 * durch den Random Walk erzeugtes Rauschen bei stillstand wird als geschwindigkeit interpretiert
 * sofern es nicht durch ein mechanisches Fenster gefiltert wird.
 * Werte unter einem Grenzwert werden zu 0 gesetzt.
 * */

	if((OutputAcc_z<=0.3)&&(OutputAcc_z>=-0.3)){
		OutputAcc_z=0.0;
	}
	if((OutputAcc_x<=0.21)&&(OutputAcc_x>=-0.21)){
			OutputAcc_x=0.0;
		}
	if((OutputAcc_y<=0.2)&&(OutputAcc_y>=-0.2)){
			OutputAcc_y=0.0;
		}
	/////////////// window ////////////


  //schritterkennung
	/*Zur Schritterkennung wird die Beschleunigung in Z-Achse gemessen
	 * Wenn zwischen dem Erkennen eines möglichen Schritts und dem nächsten
	 * mindestens eine gewisse zeit(0.5 s) liegen wird dies als wirklicher Schritt erkannt
	 *
	 */

	if(OutputAcc_z>=2.3){
		stepcountA++;
		int zero=0;
		steptimedifference=runtime; //Bei Erkennen von z>=2.1 wird zeit festgehalten
		if(runtime-steptimedifference>=0.5){
			sprung=1;
			}
		}

	if(sprung==1){
		float secsteptimedifference=runtime;
		if(OutputAcc_z<=0.5&&OutputAcc_z>=-0.5){
			if (runtime-secsteptimedifference<=0.5){
				realstep++;
				sprung=0;
			}
		}
		stepdistance=realstep*0.8;
	}

/*	Durch Integration mit 20ms zeitkonstante entstehen Fehler
 * Diese können mit einer Interpolation reduziert werden
 *
*/
	//geschwindigkeit nach 1. order approx
	speed_acc_x+=(acc_buff_x+((OutputAcc_x-acc_buff_x)/2))*DT;
	speed_acc_y+=(acc_buff_y+((OutputAcc_y-acc_buff_y)/2))*DT;
	speed_acc_z+=(acc_buff_z+((OutputAcc_z-acc_buff_z)/2))*DT;

	// position mit 1. order approx
	positionx+=(speed_buff_x+((speed_acc_x-speed_buff_x)/2))*DT;
	positiony+=(speed_buff_y+((speed_acc_y-speed_buff_y)/2))*DT;
	positionz+=(speed_buff_z+((speed_acc_z-speed_buff_z)/2))*DT;

///////// POSITIONSBESTIMMUNG FÜR ROTIERTEN VEKTOR //////////////
	/*
	//geschwindigkeit nach 1. order approx
	speed_acc_x+=(globalAcc_buff_x+((globalAcc_x-globalAcc_buff_x)/2))*DT;
	speed_acc_y+=(globalAcc_buff_y+((globalAcc_y-globalAcc_buff_y)/2))*DT;
	speed_acc_z+=(globalAcc_buff_z+((globalAcc_z-globalAcc_buff_z)/2))*DT;

	// position mit 1. order approx
	positionx+=(speed_buff_x+((speed_acc_x-speed_buff_x)/2))*DT;
	positiony+=(speed_buff_y+((speed_acc_y-speed_buff_y)/2))*DT;
	positionz+=(speed_buff_z+((speed_acc_z-speed_buff_z)/2))*DT;
	*/
////////////////////////***********************////////////////////////

	/*
	 * Berechnungen ohne Interpolation..
	 *
	//Geschwindigkeit
	speed_acc_x+=OutputAcc_x*DT;
	speed_acc_y+=OutputAcc_y*DT;
	speed_acc_z+=OutputAcc_z*DT;

	//zurückgelegte Strecke:
	positionx+= speed_acc_x*DT;
	positiony+= speed_acc_y*DT;
	positionz+= speed_acc_z*DT;
*/
	// movement end: ********************************
/*
 * Die Geschwindigkeit ist nach Ende einer Bewegung fehlerbehaftet und nicht gleich 0
 * Damit diese nicht weiter Integriert wird und einen Fehler in der Position erzeugt
 * muss das Ende einer Bewegung erkannt werden. Dies funktioniert in dem
 * die Geschwindigkeit zu 0 gesetzt wird, wenn eine bestimmte Anzahl an Beschleunigungen
 * als 0 erkannt wurden.
 *
 **/
		if(OutputAcc_x==0.0){
			j++;
		}
		else{
			j=0;
		}
		if(j>=ME){
			speed_acc_x=0;
		}
		if(OutputAcc_y==0.0){
			k++;
		}
		else{
			k=0;
		}
		if(k>=ME){
			speed_acc_y=0;
		}
		if(OutputAcc_z==0.0){
			l++;
		}
		else{
			l=0;
		}
		if(l>=ME){
			speed_acc_z=0;
		}

// ***************************** bewegungsende *******************
// BEWGUNGSENDE BEI GLOBALER BESCHLEUNIGUNG /////////////************
/*
	if(globalAcc_x==0.0){
			j++;
		}
		else{
			j=0;
		}
		if(j>=ME){
			speed_acc_x=0;
		}
		if(globalAcc_y==0.0){
			k++;
		}
		else{
			k=0;
		}
		if(k>=ME){
			speed_acc_y=0;
		}
		if(globalAcc_z==0.0){
			l++;
		}
		else{
			l=0;
		}
		if(l>=ME){
			speed_acc_z=0;
		}

*/
	// ***************************** bewegungsende *******************

/*
 * einige Output funktionen ///////////////////////////////////////////////////////////////////////////
 */

//	printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m\n",gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY);
//	cout << "RAW: X,y,z?: " << accRaw[0]<< " " << accRaw[1]<< " " << accRaw[2] <<" " << "\t" <<  "Winkel X: " << AccXangle << "\t" << "Winkel Y: "<< AccYangle << endl;
//	cout <<"runtime: " << runtime << "\t" << "RAW X,Y;Z -OFFSET:\t" << act_acc_x << "\t" << act_acc_y << "\t" << act_acc_z << "\t " << "offset "<< offset_acc_x <<" "<< offset_acc_y<<" " << offset_acc_z << endl;
	cout << setprecision(4);
	cout << fixed	;
//	usleep(100000);
//	cout << "Gyro X,Y,Z " << gyrRaw[0] << "\t" << gyrRaw[1] << "\t" << gyrRaw[2] << "\t" << "Angle xyz: " << gyroXangle << " " << gyroYangle << " " << gyroZangle << endl;
//	cout << "Gyro X,Y,Z " << rate_gyr_x << "\t" << rate_gyr_y << "\t" << rate_gyr_z << "\t" << "Angle xyz: " << gyroXangle << " " << gyroYangle << " " << gyroZangle << endl;
//	cout << gyro_offset_x << "\t" << gyro_offset_y << "\t" << gyro_offset_z <<"\t" << endl;

//Aktueller COUT
	cout <<"runtime: " << runtime <<" " << BB << " " << "m/s²: " << OutputAcc_x << "\t" << OutputAcc_y << "\t" << OutputAcc_z <<"\t" << "Speed m/s: \t" << speed_acc_x << "\t"<< speed_acc_y <<"\t"<< speed_acc_z << "\t"<< "Position:\t"<<positionx << "\t"<<positiony << "\t"<<positionz <<endl;
	//cout <<"runtime: " << runtime << " Z Rate" << rate_gyr_z << "\t" << "X rate "<< rate_gyr_x << "\t" << "Y Rate " << rate_gyr_y << "GYRO Z ANGLE: " << gyroZangle << endl;
	//cout << offsetTolerance << "\t"<< offset_acc_z << endl;
  	//cout << "Gravitation rotiert: "<<  globalgrav_x << "\t"<< globalgrav_y <<" "<< globalgrav_z <<" Gesamt nach rot: "<< OutputAcc_x << " " << OutputAcc_y << " " <<OutputAcc_z << " " << globalAcc_x << " " << globalAcc_y << " "<< globalAcc_z << endl;
	//ALT cout << "Laufzeit:\t"<< runtime << " \t" << "X WERT:\t" << OutputX << "  \t" <<"Y WERT:\t" << OutputY << endl;
 // 	cout <<"runtime: " << runtime << " " << "m/s²: " << globalAcc_x << "\t" << globalAcc_y << "\t" << globalAcc_z <<"\t" << "Speed m/s: \t" << speed_acc_x << "\t"<< speed_acc_y <<"\t"<< speed_acc_z << "\t"<< "Position:\t"<<positionx << "\t"<<positiony << "\t"<<positionz <<endl;

//	cout << "Gravitation rotiert: "<<  globalgrav_x << "\t"<< globalgrav_y <<" "<< globalgrav_z << " " << gyroXangle << " " << gyroYangle << " " << gyroZangle << " " << CFangleX << " " << CFangleY <<endl;
	//cout << "runtime: "<<  runtime << "\t AccRaw "<< AccXangle <<" "<< AccYangle <<" emaWinkel: "<< emaAccAngleX<< " "<< emaAccAngleY <<" CFFilter1,2 "<< CFangleX << " " << CFangleY << " " <<CF2angleX << " " << CF2angleY << endl;
//	cout << "runtime: "<<  runtime << "\t AccRaw "<< AccXangle <<" "<< AccYangle << " " << gyroXangle << " CFilter1,2 "<< CFangleX << " " << CFangleY << " " <<CF2angleX << " " << CF2angleY << endl;


	// cout << "runtime: " << runtime << "\t" << "Gyro Winkel: "<< gyroXangle << " " << gyroYangle << " " <<gyroZangle << " " << "Acc Winkel " << AccXangle << " " << AccYangle << " CFANGLE " << CFangleX << " " << CFangleY<< " " << globalAcc_x << endl;
	// cout << endl;
 	//cout << "runtime: " << runtime << " " "steps: " << realstep << "\t" << "position steps: " << stepdistance << "\t" << "posACCx: " << positionx <<" " << positiony << " " << positionz << " "<< endl; //fabs(positionx) << endl;
 	//cout << "runtime: " << runtime << " " << "unfiltered: " << unfilteredACCx/offsetTolerance << " " << act_acc_x/offsetTolerance << " " << endl;
	//Each loop should be at least 20ms.
        while(mymillis() - startInt < (DT*1000))
        {
            usleep(100);
        }

	//printf("Loop Time %d\t", mymillis()- startInt);

        // Buffer werden gesetzt bzw. Variablen am Ende des Loops wieder = 0;

        act_acc_x=0.0;
        act_acc_y=0.0;
        act_acc_z=0.0;



        rotacc_x=0.0;
        rotacc_y=0.0;
        rotacc_z=0.0;

        emaAccBuff_x=emaAcc_x;
        emaAccBuff_y=emaAcc_y;
        emaAccBuff_z=emaAcc_z;

        emaAcc_x=0.0;
        emaAcc_y=0.0;
        emaAcc_z=0.0;

        emaAccAngleX=0.0;
        emaAccAngleY=0.0;

        acc_buff_x=OutputAcc_x;
        acc_buff_y=OutputAcc_y;
        acc_buff_z=OutputAcc_z;

        globalAcc_buff_x=globalAcc_x;
        globalAcc_buff_y=globalAcc_y;
        globalAcc_buff_z=globalAcc_z;
/*
        globalSpeed_buff_x=speed_acc_x;
        globalSpeed_buff_y=speed_acc_y;
        globalSpeed_buff_z=speed_acc_z;
*/

        speed_buff_x=speed_acc_x;
        speed_buff_y=speed_acc_y;
        speed_buff_z=speed_acc_z;

        OutputAcc_x=0.0;
        OutputAcc_y=0.0;
        OutputAcc_z=0.0;

    	globalAcc_x=0.0;
    	globalAcc_y=0.0;
    	globalAcc_z=0.0;

    	emaLPbuff_x=0.0;
    	emaLPbuff_y=0.0;
    	emaLPbuff_z=0.0;


	}



}

