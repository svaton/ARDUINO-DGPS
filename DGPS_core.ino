#include <TinyGPS_UBX.h>
#include <math.h> 
#include <SPI.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>
#include <fonts/allFonts.h>
#include <images/cvut.c>
#include <images/liu.c>

//GPSsv = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}  GPS SV array 1-32
TinyGPSPlus gps;
int GPSsat = 32; //number of GPS satellites in use
double t;
int rec_len = 6;
int rec_sig[12] = { 7,14,21,4,5,8 };

//general variable
#define SERIAL1_BUFFER_SIZE_RX 4096
#define SERIAL3_BUFFER_SIZE_RX 1024
#define ageLimit 20
//HW setup
#define PushButt 22
#define encoderPin1 24
#define encoderPin2 26
bool defineRES = false, defineSBAS = false, defineKalman = false;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
unsigned long button_delay;
int lastMSB = 0;
int lastLSB = 0;
int selection = 1;
bool bootSeq=true,initSeq = true,mainScr = false, disturb = true, disturb2 = true, disturb3 = false;
//LCD setup
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10
ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);
bool scrSelect[] = { false,false,false,false }; // screen selection ModeSelect/
int selectVal;
unsigned long lcdUpdate = 0;
char datePrev[10];
//////////////////////////////
// #define SBAS
double Xs[33], Ys[33], Zs[33];  //eccef satellite coords array
byte pollEPH[] = { 0xB5, 0x62, 0x0B, 0x31, 0x00, 0x00, 0x3C, 0xBF };
//rover encode var
int UBXflag = 0, counter = 0, rover_id, UBXlength = 11, rSatUsed, rSatUsedList[15];
double rLLA[4], rECEF[4], rLLA_U[4], rECEF_U[4], rcPos[4], rTow, rSpd;
unsigned short rCrs;
bool UBXckError = 0, rNewData = false, WifiEstablished = false;
double rRes[32];
unsigned long recep; // reception time
#define WifiRst 13
byte CK_A, CK_B, window[80];
//WGS84 and others reference data
double wf = 1 / 298.257223563;  // flattening 
double wR = 6378137; // elispoid equatorial radius
double wRb = wR*(1 - wf);  //earth radius to the pole 
double c = 299792458; // speed of light according to IS-GPS-200
// Ephemeris PRC calc
double tPositionLLA[3] = { 58.454107, 15.563562,71 };	// Latitude (deg), Longitude(deg), Altitude (MSL)
double tPositionECEF[3];
unsigned long dAge, tEPH;
unsigned long age[32], corAge[32];
int valid[32];
// Position correction calc vars
double LOSr[60]; // Projection matric of rover
double r[32]; // range from RS measured position to the satellites (indexing by SV number)
double dRr[32]; // range delta for computation of rover unfiltered position
double tr[32]; // true range from correct position to the satellite
double Rr[32]; // Rover range from measured position
double cR[32]; // corrected Rover pseudorange
double dcR[32]; // delta of cR with actaul Range used for correction calculation
double actualECEF[4]; // RS ECEF
double actualLLA[4]; // actual position computed and filtered by receiver
double trueECEF[4]; // ECEf coordinates of the known position
double cLLA[4]; //corrected rover LLA
double cLLA_F[4]; // corected and filtered rover ECEF
double LLA_U[4]; // unfiltered 
double cECEF[4]; // corrected rover ECEF
double cECEF_F[4]; // corrected and filtered rover ECEF 
double satCB[15]; //satellite clock bias (not used)
double PC[4]; // Position correction vector
double e[5]; // line - of - sight vector
// Kalman filter values
double correctionVector[15];
// Kalmann filter
#define States 3 // define number of measured states (X,Y,Z)
#define devQ 0.00003 // define process noise covariance
#define devR 40000	//define measurement noise covariance
double Xp[] = { 3222000, 897000, 5412000,0,0,0 }; // initial values (X,Y,Z,Vx,Vy,Vz) is defined in LCDinit()
double Xo[6]; // output vector
double P[6][6], F[6][6], cQ[3][3], R[3][3], H[3][6], G[6][3] ;
unsigned long timestepPrev;
double P_pred2[6][6];
double P_pred[6][6];
double Q_pred[6][3];
double x_pred[6];
double Q[6][6];
double K1[6][3];
double K2[3][6];
double X1[3];
double X2[6];
double K[6][3];
double P2[6][6];
double Kinv[9];
double E_pred[6][6];
//Statistic and averaging
# define _LENGTH 3600 // define interval over the RMS error is computed [number of meausrements] (3600 ~ 1 hour)
# define _AVG_LENGTH 1000 // interval for average rover position calculation
double AvgSum[4];
double StdSum[4];
double PrecSum[4];
double tempAvgLLa[4];
unsigned int avgCount = 0, stdCount = 0, precCount; // counter to determine number of measurements
double rECEF_Avg[4]; // Averaged rover position array
double rPrec[5]; //Rover precision array [Actual H,Actual V, DRMSE H, RMSE V] 
double RSAcc[8];
double rENU[4]; // ENU coordinates of rover station with respect to the average rover position
double RS_ENU[4]; // ENU coordinates of Reference station with respect to the true position

/////////////////////////////////

void setup()
{
	Serial.begin(115200);
	Serial1.begin(57600);
	Serial3.begin(115200);
	pinMode(WifiRst, OUTPUT);
	pinMode(PushButt, INPUT);
	digitalWrite(WifiRst, LOW);
	//Encoder
	pinMode(encoderPin1, INPUT);
	pinMode(encoderPin2, INPUT);
	digitalWrite(encoderPin1, HIGH);
	digitalWrite(encoderPin2, HIGH);
	attachInterrupt(24, updateEncoder, CHANGE);
	attachInterrupt(26, updateEncoder, CHANGE);
	delay(200); // to reset Wifi
	poolEPH(); // send UBX-CFG and the initital to setup the receiver
	digitalWrite(WifiRst, HIGH);
	kalmanSet(devQ, devR, States); // Set covariance matricies (devQ,devR,dimension), identity matrix, Error covariance 
	tft.begin();
	tft.setRotation(iliRotation270);
	tft.fillScreen(0x0000);
	tft.setTextScale(2);
	tft.setFont(Arial14);
	tft.printAlignedOffseted(F("Arduino - DGPS"), gTextAlignMiddleCenter, 0, -50);
	tft.drawImage(cvut, 25, 150, 90, 74);
	tft.drawImage(liu, 160, 150, 128, 74);
	dAge = millis();
	/////testing
	//defineRES = true;
	//mainScrStatic();
	//mainScr = true;
	//tft.setTextScale(1);
	/*double ENU[4];
	double tecef[] = { 3222450.76592965, 897506.554336328, 5412428.66813226 };
	double tlla[4];
	ECEFTolla(tlla, tecef);
	double ecef[] = { 3338100, 861100, 5348100 };
		
	ECEFtoENU(ENU, ecef, tecef, tlla);
	Serial.println(tlla[0], 8);
	Serial.println(tlla[1], 8);
	Serial.println(tlla[2], 4);
	Serial.println();
	Serial.println(ENU[0], 4);
	Serial.println(ENU[1], 4);
	Serial.println(ENU[2], 4);*/
	
	
}


void loop()
{
	while (Serial1.available())gps.encode(Serial1.read());
	while (Serial3.available())roverEncode(Serial3.read());
	
	if (defineRES && !initSeq)
	{
		if (rNewData )coreRES();

		if (mainScr)MainScrRES();
	}
	else if (defineSBAS && !initSeq)
	{
		if (rNewData)coreSBAS();
		if (mainScr)MainScrSBAS();
	}

	if (millis() > 3000 && (bootSeq || initSeq) && !mainScr)LCDinit();

	

}


bool pressedButton()
{
	return !digitalRead(PushButt);
}

void updateEncoder() {
	int MSB = digitalRead(encoderPin1); //MSB = most significant bit
	int LSB = digitalRead(encoderPin2); //LSB = least significant bit

	int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
	int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

	if (sum == 0b1101)  // right rotation
	{
		if (scrSelect[0])
		{
			selection++;
			if (selection >= 2)selection = 0;
			disturb = true;
		}
		else if (scrSelect[2] && disturb2 == false)
		{
			selection++;
			if (selection >= 5)selection = 0;
			disturb = true;
		}
		else if (scrSelect[3])
		{
			selection = -1;
			disturb = true;
		}
	}
		
	if (sum == 0b0111) // left rotation
	{
		if (scrSelect[0])
		{
			selection--;
			if (selection < 0)selection = 1;
			disturb = true;
		}
		else if (scrSelect[2] == true && disturb2 == false)
		{
			selection--;
			if (selection < 0)selection = 4;
			disturb = true;
		}
		else if (scrSelect[3])
		{
			selection = 1;
			disturb = true;
		}
	}
	lastEncoded = encoded; //store this value for next time
}

void roverEncode(byte b)
{
	if (UBXflag < 2)
	{
		switch (b)
		{
		case 0xB5:
		{
			UBXflag = 1;
			rNewData = false;
			break;
		}
		case 0x62:
		{
			if (UBXflag == 1)
			{
				UBXflag = 2;
				counter = 0;
				CK_A = 0;
				CK_B = 0;
				UBXckError = 0;
				UBXlength = 5;
			}
			else UBXflag = 0;
			break;
		}
		}
	}
	else if (UBXflag == 2)
	{
		window[counter] = b;

		if (counter <= UBXlength && window[0] == 0xCC && window[1] == 0x01)
		{
			if (counter == 2)UBXlength = 3 * b + 30;
			if (counter < UBXlength)   // UBX checksum
			{
				CK_A += window[counter - 1];
				CK_B += CK_A;
				CK_A &= 0xff;
				CK_B &= 0xff;
			}
			if (counter == UBXlength)
			{
				if (CK_A == window[UBXlength - 1] && CK_B == window[UBXlength])
				{
					UBXckError = 0;
					union {
						long s;
						unsigned char c[4];
					} union_signed_long;
					union {
						short z;
						unsigned char d[2];
					} union_int;
					rSatUsed = window[2];
					
					union_signed_long.c[0] = window[7];
					union_signed_long.c[1] = window[6];
					union_signed_long.c[2] = window[5];
					union_signed_long.c[3] = window[4];
					rTow = double(union_signed_long.s);
					union_signed_long.c[0] = window[11];
					union_signed_long.c[1] = window[10];
					union_signed_long.c[2] = window[9];
					union_signed_long.c[3] = window[8];
					rSpd = double(union_signed_long.s) / 100;
					union_signed_long.c[0] = window[15];
					union_signed_long.c[1] = window[14];
					union_signed_long.c[2] = window[13];
					union_signed_long.c[3] = window[12];
					rCrs = union_signed_long.s;
					union_signed_long.c[0] = window[19];
					union_signed_long.c[1] = window[18];
					union_signed_long.c[2] = window[17];
					union_signed_long.c[3] = window[16];
					rECEF[0] = double(union_signed_long.s) / 100;
					union_signed_long.c[0] = window[23];
					union_signed_long.c[1] = window[22];
					union_signed_long.c[2] = window[21];
					union_signed_long.c[3] = window[20];
					rECEF[1] = double(union_signed_long.s) / 100;
					union_signed_long.c[0] = window[27];
					union_signed_long.c[1] = window[26];
					union_signed_long.c[2] = window[25];
					union_signed_long.c[3] = window[24];
					rECEF[2] = double(union_signed_long.s) / 100;
					for (int i = 0; i < rSatUsed; i++)
					{
						rSatUsedList[i] = window[28 + i];
						union_int.d[0] = window[28 + rSatUsed + (i * 2)];
						union_int.d[1] = window[28 + rSatUsed + (i * 2) + 1];
						rRes[rSatUsedList[i] - 1] = double(union_int.z) / 100;
					}
					tft.fillCircle(300, 11, 8, 0x0f00);
					rNewData = true;
					disturb3 = true;
					recep = millis();
				}
				else UBXckError = 1;
				UBXflag = 0;

			}

		}
		else if (counter <= UBXlength && window[0] == 0xCC && window[1] == 0xFE)
		{
			if (counter >= 2)
			{
				Serial.print("Rover ID");
				Serial.print(window[2], HEX);
				Serial.println(" recognized");
				WifiEstablished = true;
				rover_id = window[2];
				UBXflag = 0;
			}
		}
		else if (counter <= UBXlength && window[0] == 0xCC && window[1] == 0xFF)
		{
			if (counter >= 2)
			{
				Serial.println("Rover connection pending");
				UBXflag = 0;
			}
		}
		else if (counter > 2)UBXflag = 0;
		counter++;
	}

}

void coreRES()
{	
	if (gps.time.isFinished())
	{
		t = millis();
		if (!gps.location.ubxecef())
		{
			actualLLA[0] = gps.location.lat();
			actualLLA[1] = gps.location.lng();
			actualLLA[2] = gps.altitude.elipsoid();
			llaToECEF(actualECEF, actualLLA, 0);
		}
		else
		{
			actualECEF[0] = gps.location.ecefX();
			actualECEF[1] = gps.location.ecefY();
			actualECEF[2] = gps.location.ecefZ();
		}
		ECEFTolla(rLLA, rECEF);
		llaToECEF(trueECEF, tPositionLLA, gps.altitude.geoid());
		
		/*Serial.print(", ");
		Serial.print(gps.ubxSBAS.status(2)); // SBAS status
		Serial.print(", ");
		Serial.print(rSatUsed);
		Serial.println(", ");
		Serial.print(rECEF[0], 2);
		Serial.print(", ");
		Serial.print(rECEF[1], 2);
		Serial.print(", ");
		Serial.print(rECEF[2], 2);
		Serial.println(", ");
		Serial.print(rLLA[0], 7);
		Serial.print(", ");
		Serial.print(rLLA[1], 7);
		Serial.print(", ");
		Serial.print(rLLA[2], 2);*/
		Serial.println(", ");
		
		for (int i = 0; i < rSatUsed; i++) //go trough the set of satellites used in rover
		{
			if (gps.ubxEPH.valid(rSatUsedList[i])) // compute PRC from Ephmemerides
			{
				int satellite = rSatUsedList[i];
				satPosition(gps.time.tow(), rSatUsedList[i], &satCB[i]); //calculate satellite position for all remaining satellites used by RS
				tr[rSatUsedList[i] - 1] = trueRange(trueECEF, rSatUsedList[i]); // true range from the known location to the satellite
				Rr[rSatUsedList[i] - 1] = trueRange(rECEF, rSatUsedList[i]); // range from the rover location to the satellite
				r[rSatUsedList[i] - 1] = trueRange(actualECEF, rSatUsedList[i]); // range from measured position (RS) to the satellite
				// prepare for calculating unfiltered rover coordinates
				dRr[rSatUsedList[i] - 1] = rRes[satellite - 1]; 
				losEPH(e, rECEF, Rr[rSatUsedList[i] - 1], satellite);// calculate LOS vector (rover)
				LOSr[(i * 4) + 0] = e[0];//convert LOS vectors into line
				LOSr[(i * 4) + 1] = e[1];
				LOSr[(i * 4) + 2] = e[2];
				LOSr[(i * 4) + 3] = e[3];
				for (int b = 0; b < (int)gps.sv.totalSatUsed(); b++) //check if rover satelites are used in RS nav solution
				{
					if (gps.sv.satUsedList(b) == rSatUsedList[i])
					{
						valid[rSatUsedList[i] - 1] = 1;
						cR[rSatUsedList[i] - 1] = Rr[rSatUsedList[i] - 1] + rRes[satellite - 1] - gps.sv.satRes(rSatUsedList[i]) + tr[rSatUsedList[i] - 1] - r[rSatUsedList[i] - 1]; //rover corrected range
						dcR[rSatUsedList[i] - 1] = cR[rSatUsedList[i] - 1] - Rr[rSatUsedList[i] - 1]; //PRC
						corAge[rSatUsedList[i] - 1] = 0;
						age[rSatUsedList[i] - 1] = 0;
						break;
					}
					else if (b == gps.sv.totalSatUsed() - 1)
					{
						valid[rSatUsedList[i] - 1] = 0;
						age[rSatUsedList[i] - 1] += millis() - dAge;
						corAge[rSatUsedList[i] - 1] = age[rSatUsedList[i] - 1] / 1000;
						if (age[rSatUsedList[i] - 1] > ageLimit && gps.ubxSBAS.status(1) == 1 && gps.ubxSBAS.valid(rSatUsedList[i]))
						{
							dcR[rSatUsedList[i] - 1] = gps.ubxSBAS.fast(rSatUsedList[i]) + gps.ubxSBAS.iono(rSatUsedList[i]);
							corAge[rSatUsedList[i] - 1] = 0;
							valid[rSatUsedList[i] - 1] = 2;
						}
					}
				}
			}
			else
			{
				valid[rSatUsedList[i] - 1] = 0;
				corAge[rSatUsedList[i] - 1] = 999;
			}
		}
		dAge = millis();
		for (int i = 0; i < rSatUsed; i++)correctionVector[i] = dRr[rSatUsedList[i] - 1];
		posCorr(PC, correctionVector, LOSr, rSatUsed); // calculate unfiltered rover position from ranges and residuals
		rECEF_U[0] = rECEF[0] + PC[0];
		rECEF_U[1] = rECEF[1] + PC[1];
		rECEF_U[2] = rECEF[2] + PC[2];
		for (int i = 0; i < rSatUsed; i++)correctionVector[i] = dcR[rSatUsedList[i] - 1];
		posCorr(PC, correctionVector, LOSr, rSatUsed); // calculate position correction
		cECEF[0] = rECEF[0] + PC[0];
		cECEF[1] = rECEF[1] + PC[1];
		cECEF[2] = rECEF[2] + PC[2];
		/*for (int i = 0; i < gps.sv.totalSatUsed(); i++)correctionVector[i] = dRSr[gps.sv.satUsedList(i) - 1];
		posCorr(PC, dRSr, LOS, rSatUsed); // calculate RS unfiltered
		actualECEF_U[0] = actualECEF[0] + PC[0];
		actualECEF_U[1] = actualECEF[1] + PC[1];
		actualECEF_U[2] = actualECEF[2] + PC[2];*/
		ECEFTolla(cLLA, cECEF);
		ECEFTolla(rLLA_U, rECEF_U);
		// Kalmann filter
		if (defineKalman)
		{
			
			kalmanProc(States, cECEF, Xo, Xp);
			for (int i = 0; i < 3; i++) cECEF_F[i] = Xo[i];
			ECEFTolla(cLLA_F, cECEF_F);
			running_average(rECEF_Avg, cECEF_F, _AVG_LENGTH); // Averaged rover position 
			ECEFTolla(tempAvgLLa, rECEF_Avg); // get geodetic coords of the average
			ECEFtoENU(rENU, cECEF_F, rECEF_Avg, tempAvgLLa); // get ENU coordintaes of the rover (with repsect to the average)
			PrecRover(rPrec, rENU, _LENGTH);
		}
		else
		{
			running_average(rECEF_Avg, cECEF, _AVG_LENGTH); // Averaged rover position 
			ECEFTolla(tempAvgLLa, rECEF_Avg); // get geodetic coords of the average
			ECEFtoENU(rENU, cECEF, rECEF_Avg, tempAvgLLa); // get ENU coordintaes of the rover (with repsect to the average)
			PrecRover(rPrec, rENU, _LENGTH);
		}

		//RS statistics
		ECEFtoENU(RS_ENU, actualECEF, trueECEF, tPositionLLA);
		AccStdDevRS(RSAcc, RS_ENU, _LENGTH);


	/*	Serial.println();
		Serial.print(RSAcc[0], 2);
		Serial.print(", ");
		Serial.print(RSAcc[1], 2);
		Serial.print(", ");
		Serial.print(RSAcc[2], 2);
		Serial.print(", ");
		Serial.print(RSAcc[3], 2);
		Serial.print(", ");
		Serial.print(RSAcc[4], 2);
		Serial.print(", ");
		Serial.print(RSAcc[5], 2);
		Serial.print(", ");
		Serial.print(RSAcc[6], 2);
		Serial.println(", ");
		Serial.println();
		Serial.print(rPrec[0], 2);
		Serial.print(", ");
		Serial.print(rPrec[1], 2);
		Serial.print(", ");
		Serial.print(rPrec[2], 2);
		Serial.print(", ");
		Serial.print(rPrec[3], 2);
		Serial.println(", ");










		/*for (int i = 0; i < rSatUsed; i++)
		{

			Serial.print(rSatUsedList[i]);
			Serial.print(", ");
			Serial.print(gps.ubxEPH.valid(rSatUsedList[i]));
			Serial.print(", ");
			Serial.print(valid[rSatUsedList[i] - 1]);
			Serial.print(", ");
			Serial.print(corAge[rSatUsedList[i] - 1]);
			Serial.print(", ");
			Serial.print(dcR[rSatUsedList[i] - 1], 4);
			Serial.print(", ");
			
			Serial.print(LOSr[(i * 4) + 0], 6);
			Serial.print(", ");
			Serial.print(LOSr[(i * 4) + 1], 6);
			Serial.print(", ");
			Serial.print(LOSr[(i * 4) + 2], 6);
			Serial.print(", ");
			Serial.println(LOSr[(i * 4) + 3], 6);
		}
		Serial.println();
		Serial.print(cECEF[0], 2);
		Serial.print(", ");
		Serial.print(cECEF[1], 2);
		Serial.print(", ");
		Serial.print(cECEF[2], 2);
		Serial.println(", ");
		Serial.print(cLLA[0], 7);
		Serial.print(", ");
		Serial.print(cLLA[1], 7);
		Serial.print(", ");
		Serial.print(cLLA[2] , 2);
		Serial.println(", ");
		Serial.print(cLLA_F[0], 7);
		Serial.print(", ");
		Serial.print(cLLA_F[1], 7);
		Serial.print(", ");
		Serial.print(cLLA_F[2], 2);
		Serial.println(", ");
		Serial.print(rLLA_U[0], 7);
		Serial.print(", ");
		Serial.print(rLLA_U[1], 7);
		Serial.print(", ");
		Serial.print(rLLA_U[2], 2);
		Serial.println(", ");
		Serial.print(LLA_U[0], 7);
		Serial.print(", ");
		Serial.print(LLA_U[1], 7);
		Serial.print(", ");
		Serial.print(LLA_U[2], 2);
		Serial.println(", ");*/
		rNewData = false;
		Serial.print(gps.time.tow());
		Serial.print(", ");
		Serial.print(rTow);
		Serial.print(", ");
		Serial.println(rTow - gps.time.tow());
		Serial.println(millis() - t);
		if (millis() - tEPH > 600000)poolEPH();  // pollEPH every 10 minute
	}
}

void coreSBAS()
{
	if (gps.time.isFinished())
	{
		Serial.print("debil");
		if (gps.ubxSBAS.status(2) == 1)
		{
			Serial.println("debil2");
			t = millis();
			if (!gps.location.ubxecef())
			{
				actualLLA[0] = gps.location.lat();
				actualLLA[1] = gps.location.lng();
				actualLLA[2] = gps.altitude.elipsoid();
				llaToECEF(actualECEF, actualLLA, 0);
			}
			else
			{
				actualECEF[0] = gps.location.ecefX();
				actualECEF[1] = gps.location.ecefY();
				actualECEF[2] = gps.location.ecefZ();
			}
			ECEFTolla(rLLA, rECEF);

			for (int i = 0; i < rSatUsed; i++)
			{

				for (int b = 0; b < gps.sv.totalSat(); b++)
				{
					if (gps.sv.viewSat(b) == rSatUsedList[i]) //check if rovers satelites are available in RS view
					{
						losAE(e, actualLLA, rSatUsedList[i]);
						LOSr[(i * 4) + 0] = e[0];
						LOSr[(i * 4) + 1] = e[1];
						LOSr[(i * 4) + 2] = e[2];
						LOSr[(i * 4) + 3] = e[3];
						if (gps.ubxSBAS.valid(rSatUsedList[i])) // check if PRC is available from SBAS
						{
							dcR[rSatUsedList[i] - 1] = gps.ubxSBAS.fast(rSatUsedList[i]) + gps.ubxSBAS.iono(rSatUsedList[i]);
							valid[rSatUsedList[i] - 1] = 2;
							age[rSatUsedList[i] - 1] = 0;
							corAge[rSatUsedList[i] - 1] = 0;
						}
						else
						{
							age[rSatUsedList[i] - 1] += millis() - dAge;
							corAge[rSatUsedList[i] - 1] = age[rSatUsedList[i] - 1] / 1000;
						}
						break;
					}
					else if (b == gps.sv.totalSat() - 1)
					{
						valid[rSatUsedList[i] - 1] = 0;
						valid[rSatUsedList[i] - 1] = 0;
						corAge[rSatUsedList[i] - 1] = 999;
					}

				}
				
				Serial.print(rSatUsedList[i]);
				Serial.print(" - ");
				Serial.print(gps.ubxSBAS.valid(rSatUsedList[i]));
				Serial.print(" - ");
				Serial.print(valid[rSatUsedList[i] - 1]);
				Serial.print(" - ");
				Serial.print(corAge[rSatUsedList[i] - 1]);
				Serial.print(" - ");
				Serial.print(dcR[rSatUsedList[i] - 1], 2);
				Serial.print(" - ");
				Serial.print(LOSr[(i * 4) + 0], 6);
				Serial.print(", ");
				Serial.print(LOSr[(i * 4) + 1], 6);
				Serial.print(", ");
				Serial.print(LOSr[(i * 4) + 2], 6);
				Serial.print(", ");
				Serial.print(LOSr[(i * 4) + 3], 6);
				Serial.print("\n");
			}
			dAge = millis() / 1000;

			for (int i = 0; i < rSatUsed; i++)correctionVector[i] = dcR[rSatUsedList[i] - 1];
			posCorr(PC, correctionVector, LOSr, rSatUsed); // calculate position correction
			cECEF[0] = rECEF[0] + PC[0];
			cECEF[1] = rECEF[1] + PC[1];
			cECEF[2] = rECEF[2] + PC[2];
			ECEFTolla(cLLA, cECEF);
			Serial.println(cLLA[0], 8);
			Serial.println(cLLA[1], 8);
			Serial.println(cLLA[2], 8);

			running_average(rECEF_Avg, cECEF, _AVG_LENGTH); // Averaged rover position 
			ECEFTolla(tempAvgLLa, rECEF_Avg); // get geodetic coords of the average
			ECEFtoENU(rENU, cECEF, rECEF_Avg, tempAvgLLa); // get ENU coordintaes of the rover (with repsect to the average)
			PrecRover(rPrec, rENU, _LENGTH);

			//position error
			ECEFtoENU(RS_ENU, cECEF, actualECEF, actualLLA); // RS position error derived from position shift PC
			AccStdDevRS(RSAcc, RS_ENU, _LENGTH);


			/*	Serial.println();
			Serial.print(RSAcc[0], 2);
			Serial.print(", ");
			Serial.print(RSAcc[1], 2);
			Serial.print(", ");
			Serial.print(RSAcc[2], 2);
			Serial.print(", ");
			Serial.print(RSAcc[3], 2);
			Serial.print(", ");
			Serial.print(RSAcc[4], 2);
			Serial.print(", ");
			Serial.print(RSAcc[5], 2);
			Serial.print(", ");
			Serial.print(RSAcc[6], 2);
			Serial.println(", ");
			Serial.println();
			Serial.print(rPrec[0], 2);
			Serial.print(", ");
			Serial.print(rPrec[1], 2);
			Serial.print(", ");
			Serial.print(rPrec[2], 2);
			Serial.print(", ");
			Serial.print(rPrec[3], 2);
			Serial.println(", ");

			/*for (int i = 0; i < rSatUsed; i++)
			{

			Serial.print(rSatUsedList[i]);
			Serial.print(", ");
			Serial.print(gps.ubxEPH.valid(rSatUsedList[i]));
			Serial.print(", ");
			Serial.print(valid[rSatUsedList[i] - 1]);
			Serial.print(", ");
			Serial.print(corAge[rSatUsedList[i] - 1]);
			Serial.print(", ");
			Serial.print(dcR[rSatUsedList[i] - 1], 4);
			Serial.print(", ");

			Serial.print(LOSr[(i * 4) + 0], 6);
			Serial.print(", ");
			Serial.print(LOSr[(i * 4) + 1], 6);
			Serial.print(", ");
			Serial.print(LOSr[(i * 4) + 2], 6);
			Serial.print(", ");
			Serial.println(LOSr[(i * 4) + 3], 6);
			}
			Serial.println();
			Serial.print(cECEF[0], 2);
			Serial.print(", ");
			Serial.print(cECEF[1], 2);
			Serial.print(", ");
			Serial.print(cECEF[2], 2);
			Serial.println(", ");
			Serial.print(cLLA[0], 7);
			Serial.print(", ");
			Serial.print(cLLA[1], 7);
			Serial.print(", ");
			Serial.print(cLLA[2] , 2);
			Serial.println(", ");
			Serial.print(cLLA_F[0], 7);
			Serial.print(", ");
			Serial.print(cLLA_F[1], 7);
			Serial.print(", ");
			Serial.print(cLLA_F[2], 2);
			Serial.println(", ");
			Serial.print(rLLA_U[0], 7);
			Serial.print(", ");
			Serial.print(rLLA_U[1], 7);
			Serial.print(", ");
			Serial.print(rLLA_U[2], 2);
			Serial.println(", ");
			Serial.print(LLA_U[0], 7);
			Serial.print(", ");
			Serial.print(LLA_U[1], 7);
			Serial.print(", ");
			Serial.print(LLA_U[2], 2);
			Serial.println(", ");*/
			rNewData = false;
			Serial.print(gps.time.tow());
			Serial.print(", ");
			Serial.print(rTow);
			Serial.print(", ");
			Serial.println(rTow - gps.time.tow());
			Serial.println(millis() - t);
		}
	}
}

void satPosition(double t, int sv, double * scB) // scB output variable satellite clock bias [us]
{
	double GM = 3986005;						// *10^8
	double Om_e = 72.921157;					//*10^-6
	double dT = t - gps.ubxEPH.Toe(sv);

	if (dT > 302400) dT = dT - 604800;			// week overflow
	else if (dT < -302400) dT = dT + 604800;
	double n = ((sqrt(GM*pow(10, 8)) / sqrt(pow(gps.ubxEPH.SqrtA(sv), 6))) + (gps.ubxEPH.Dn(sv)*pow(10, -9)));		//Mean motion
	double M = gps.ubxEPH.M_0(sv) + n*dT;
	double E = M;
	for (int i = 0; i < 4; i++)
	{
		E = M + gps.ubxEPH.Ecc(sv)*sin(E);
	}
	// True anomaly
	double cosv = (cos(E) - gps.ubxEPH.Ecc(sv)) / (1 - gps.ubxEPH.Ecc(sv)*cos(E));
	double sinv = (sqrt(1 - sq(gps.ubxEPH.Ecc(sv)))*sin(E)) / (1 - gps.ubxEPH.Ecc(sv)*cos(E));
	double v = atan2(sinv, cosv);
	//Corrected Values of Lattitude, Radius, Inclination and Ascending node
	double u = v + gps.ubxEPH.Omega(sv) + (((gps.ubxEPH.Cuc(sv)*pow(10, -6))*cos(2 * (v + gps.ubxEPH.Omega(sv)))) + ((gps.ubxEPH.Cus(sv)*pow(10, -6))*sin(2 * (v + gps.ubxEPH.Omega(sv)))));
	double r = (sq(gps.ubxEPH.SqrtA(sv))*(1 - gps.ubxEPH.Ecc(sv)*cos(E))) + (((gps.ubxEPH.Crc(sv))*cos(2 * (v + gps.ubxEPH.Omega(sv)))) + ((gps.ubxEPH.Crs(sv))*sin(2 * (v + gps.ubxEPH.Omega(sv)))));
	double i = gps.ubxEPH.I_0(sv) + (gps.ubxEPH.Idot(sv)*pow(10.0, -11)*dT) + (((gps.ubxEPH.Cic(sv)*pow(10.0, -9))*cos(2 * (v + gps.ubxEPH.Omega(sv)))) + ((gps.ubxEPH.Cis(sv)*pow(10.0, -9))*sin(2 * (v + gps.ubxEPH.Omega(sv)))));
	double Om = gps.ubxEPH.Om_0(sv) + (gps.ubxEPH.Om_dot(sv)*pow(10.0, -9) - Om_e*pow(10.0, -6))*dT - (Om_e*pow(10.0, -6) * gps.ubxEPH.Toe(sv));
	//Compute satellite vehicle position
	//Satellite position in orbital plane
	double x_o = r * cos(u);
	double y_o = r * sin(u);
	//Satellite position in ECEF
	if (r > 0)
	{
		Xs[sv - 1] = x_o*cos(Om) - y_o*sin(Om)*cos(i);
		Ys[sv - 1] = x_o*sin(Om) + y_o*cos(Om)*cos(i);
		Zs[sv - 1] = y_o*sin(i);
	}
	//Satellite clock bias
	double f = ((-2 * sqrt(GM*pow10(8))) / (c*c));
	double dtr = f*gps.ubxEPH.Ecc(sv)*gps.ubxEPH.SqrtA(sv)*sin(E);
	*scB = (gps.ubxEPH.Af0(sv)*pow10(-5)) + ((gps.ubxEPH.Af1(sv)*pow10(-12))*(t - gps.ubxEPH.Toc(sv))) + (gps.ubxEPH.Af2(sv)*sq((t - gps.ubxEPH.Toc(sv)))) + dtr;

}

void LCDinit()
{
	
	if (bootSeq)
	{ 
		if (disturb)
		{
			tft.fillScreen(0x0000);
			tft.setTextScale(1);
			tft.drawRoundRect(0, 0, 319, 25, 5, 0xC618);
			tft.printAlignedOffseted(F("Boot sequence"), gTextAlignTopCenter, 0, 7);
			tft.setTextColor(0x00ff00);
			tft.setFont(System5x7);
			tft.printAlignedOffseted(F("RS GPS:"), gTextAlignTopLeft, 10, 40);
			while (!Serial1.available())
			{
				if (disturb2)
				{
					tft.printAlignedOffseted(F("Waiting for connection"), gTextAlignTopLeft, 120, 40);
					disturb2 = false;
				}
			}
			disturb2 = true;
			poolEPH();
			tft.printAlignedOffseted(F("Connected !             "), gTextAlignTopLeft, 120, 40);
			tft.printAlignedOffseted(F(" CLOCK  POSECEF   SBAS  TIMEGPS  SVINFO  EPH"), gTextAlignTopLeft, 0, 60);
			button_delay = millis();
			disturb = false;
		}
		else if (millis() - button_delay >= 1000)
		{
			bool UBXsen[] = { gps.time.CLOCKavailable(), gps.location.ubxecef(), gps.ubxSBAS.SBASavailable(), gps.time.TIMEGPSavailable(), gps.sv.SVINFOavailable(), gps.ubxEPH.available() };
			int UBXoffset[] = { 10,65,125,175,234,283 };
			for (int i = 0; i < 6; i++)
			{
				if (UBXsen[i] == true)
				{
					tft.setTextColor(0xd69a);
					tft.printAlignedOffseted(F(" on"), gTextAlignTopLeft, UBXoffset[i], 70);
				}
				else
				{
					tft.setTextColor(0xf800);
					tft.printAlignedOffseted(F(" off"), gTextAlignTopLeft, UBXoffset[i], 70);
				}
			}
			tft.drawLine(0, 85, 320, 85, 0x00ff);
			tft.setTextColor(0x00ff00);
			tft.printAlignedOffseted(F("Rover status:"), gTextAlignTopLeft, 10, 100);

			while (!WifiEstablished)
			{
				tft.setTextColor(0xf800);
				tft.printAlignedOffseted(F("Waiting for connection"), gTextAlignTopLeft, 120, 100);
				if (Serial3.available())roverEncode(Serial3.read());
			}
			tft.setTextColor(0x00ff00);
			char Rover_con[20];
			int r_c = sprintf(Rover_con, "Rover ID%03d Connected !", rover_id);
			tft.printAlignedOffseted(Rover_con, gTextAlignTopLeft, 120, 100);
			bootSeq = false;
			disturb = true;
			scrSelect[0] = true;
			tft.printAlignedOffseted(F("Select Mode:"), gTextAlignTopLeft, 10, 130);
			tft.setFont(Arial14);
			tft.setTextColor(0xd69a);
			tft.printAlignedOffseted(F("SBAS"), gTextAlignTopLeft, 130, 128);
			tft.printAlignedOffseted(F("RESIDUALS"), gTextAlignTopLeft, 220, 128);
		}
	}
	else if (initSeq)
	{
		// Select mode
		if (scrSelect[0])
		{
			switch (selection)
			{
			case 0:
			{
				if (disturb)
				{
					tft.drawRoundRect(124, 124, 50, 20, 5, 0x00ff00);
					tft.drawRoundRect(214, 124, 90, 20, 5, 0x0000);
					disturb = false;
				}
				if (pressedButton())
				{
					defineSBAS = true;
					scrSelect[0] = false;
					disturb = true;
					button_delay = millis();
					tft.setFont(Arial14);
				}
				break;
			}
			case 1:
			{
				if (disturb)
				{
					tft.drawRoundRect(124, 124, 50, 20, 5, 0x0000);
					tft.drawRoundRect(214, 124, 90, 20, 5, 0x00ff00);
					disturb = false;
				}
				if (pressedButton())
				{
					scrSelect[0] = false;
					scrSelect[1] = true;
					disturb = true;
					poolEPH();
					tft.setTextColor(0x00ff00);
					tft.setFont(System5x7);
					tft.printAlignedOffseted(F("Set true position:"), gTextAlignTopLeft, 10, 160);
				}
				break;
			}
			}
		}
		if (defineSBAS && millis() - button_delay >= 700)
		{
			if (gps.ubxSBAS.status(1) != 1 && disturb)
			{
				tft.setTextColor(0x00ff00);
				tft.printAlignedOffseted(F("Waiting for SBAS"), gTextAlignBottomCenter, 0, -30);
				disturb = false;
			}
			if (gps.ubxSBAS.status(1) == 1)
			{
				tft.setFont(Arial_big);
				tft.setTextColor(0x00ff00);
				tft.printAlignedOffseted(F(" ------     READY !    ------ "), gTextAlignBottomCenter, 0, -30);
				delay(500);
				initSeq = false;
				defineSBAS = true;
				defineRES = false;
				button_delay = millis();
				selection = 0;
				disturb = true;
				mainScrStatic();
			}
		}
		if (scrSelect[1])
		{
			if (gps.satellites.status() == 0)
			{
				if (disturb)
				{
					tft.setTextColor(0xf800);
					tft.printAlignedOffseted(F("Waiting for Fix"), gTextAlignTopLeft, 150, 160);
					disturb = false;
				}
			}
			else
			{
				tft.setFont(System5x7);
				tft.setTextColor(0x00ff00);
				tft.printAlignedOffseted(F("3D Fix Available  "), gTextAlignTopLeft, 150, 160);

				disturb = true;
				scrSelect[1] = false;
				scrSelect[2] = true;
				button_delay = millis();
			}
		}
		if (scrSelect[2] && millis() - button_delay >= 300)
		{
			if (disturb2)
			{
				tPositionLLA[0] = gps.location.lat();
				tPositionLLA[1] = gps.location.lng();
				tPositionLLA[2] = gps.altitude.elipsoid();
				tft.setTextColor(0x00ff00);
				tft.printAlignedOffseted(F("Latitude:"), gTextAlignTopLeft, 10, 180);
				tft.printAlignedOffseted(F("Longitude:"), gTextAlignTopLeft, 10, 200);
				tft.printAlignedOffseted(F("Altitude (MSL):"), gTextAlignTopLeft, 10, 220);
				tft.drawRoundRect(255, 177, 50, 24, 5, ILI9341_WHITE);
				tft.drawRoundRect(255, 203, 50, 24, 5, ILI9341_WHITE);
				tft.setFont(Arial_small);
				tft.printAt(F("K_f  on"), 260, 185);
				tft.printAt(F("K_f off"), 260, 211);
				tft.setFont(Arial14);
				tft.setTextColor(0xd69a);
				printNumF(tPositionLLA[0], 7, 120, 178, 46, 10, 48);
				printNumF(tPositionLLA[1], 7, 120, 198, 46, 11, 48);
				printNumF(tPositionLLA[2], 2, 120, 218, 46, 5, 48);
				disturb2 = false;
			}
			else
			{
				switch (selection)
				{
				case 0:
				{
					if (disturb)
					{
						tft.drawRoundRect(114, 174, 105, 20, 5, 0x00ff00);
						tft.drawRoundRect(114, 194, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 214, 80, 20, 5, 0x0000);
						tft.drawRoundRect(255, 203, 50, 24, 5, ILI9341_WHITE);
						tft.drawRoundRect(255, 177, 50, 24, 5, ILI9341_WHITE);
						disturb = false;
					}
					if (pressedButton())
					{
						tft.drawRoundRect(114, 174, 105, 20, 5, 0xffff);
						scrSelect[2] = false;
						scrSelect[3] = true;
						button_delay = millis();
						selectVal = 0;
						selection = 0;
						disturb = true;
					}
					break;
				}
				case 1:
				{
					if (disturb)
					{
						tft.drawRoundRect(114, 174, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 194, 105, 20, 5, 0x00ff00);
						tft.drawRoundRect(114, 214, 80, 20, 5, 0x0000);
						tft.drawRoundRect(255, 203, 50, 24, 5, ILI9341_WHITE);
						tft.drawRoundRect(255, 177, 50, 24, 5, ILI9341_WHITE);
						disturb = false;
					}
					if (pressedButton())
					{
						tft.drawRoundRect(114, 194, 105, 20, 5, 0xffff);
						scrSelect[2] = false;
						scrSelect[3] = true;
						button_delay = millis();
						selectVal = 1;
						selection = 0;
						disturb = true;
					}
					break;
				}
				case 2:
				{
					if (disturb)
					{
						tft.drawRoundRect(114, 194, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 174, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 214, 80, 20, 5, 0x00ff00);
						tft.drawRoundRect(255, 203, 50, 24, 5, ILI9341_WHITE);
						tft.drawRoundRect(255, 177, 50, 24, 5, ILI9341_WHITE);
						disturb = false;
					}
					if (pressedButton())
					{
						tft.drawRoundRect(114, 214, 80, 20, 5, 0xffff);
						scrSelect[2] = false;
						scrSelect[3] = true;
						button_delay = millis();
						selectVal = 2;
						selection = 0;
						disturb = true;
					}
					break;
				}
				case 3:
				{
					if (disturb)
					{
						tft.drawRoundRect(114, 194, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 174, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 214, 80, 20, 5, 0x0000);
						tft.drawRoundRect(255, 177, 50, 24, 5, 0x0f00);
						tft.drawRoundRect(255, 203, 50, 24, 5, ILI9341_WHITE);
						disturb = false;
					}
					if (pressedButton())
					{
						tft.fillRoundRect(255, 177, 50, 50, 5, 0x0000);
						for (int i = 0; i < 360; i++)
						{
							tft.fillArc(280, 202, 20, 4, 0, i, 0x00ff00);
							delay(1);
						}
						scrSelect[2] = false;
						defineKalman = true;
						defineRES = true;
						selection = 0;
						disturb = true;
						double kalman_init_lla[] = { gps.location.lat(), gps.location.lng(), gps.altitude.elipsoid() };
						llaToECEF(Xp, kalman_init_lla, 0);
						button_delay = millis();
						mainScrStatic();
						initSeq = false;
						
					}
					break;
				}
				case 4:
				{
					if (disturb)
					{
						tft.drawRoundRect(114, 194, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 174, 105, 20, 5, 0x0000);
						tft.drawRoundRect(114, 214, 80, 20, 5, 0x0000);
						tft.drawRoundRect(255, 177, 50, 24, 5, ILI9341_WHITE);
						tft.drawRoundRect(255, 203, 50, 24, 5, 0x0f00);
						disturb = false;
					}
					if (pressedButton())
					{
						tft.fillRoundRect(255, 177, 50, 50, 5, 0x0000);
						for (int i = 0; i < 360; i++)
						{
							tft.fillArc(280, 202, 20, 4, 0, i, 0x00ff00);
							delay(1);
						}
						scrSelect[2] = false;
						defineRES = true;
						defineKalman = false;
						selection = 0;
						disturb = true;
						button_delay = millis();
						mainScrStatic();
						initSeq = false;
					}
					break;
				}
				}
			}
		}

		if (scrSelect[3] && millis() - button_delay >= 300)
		{
			if (selectVal == 0 && disturb)
			{
				tPositionLLA[0] += double(selection) / 10000000;
				tft.setTextColor(0x00ff00);
				printNumF(tPositionLLA[0], 7, 120, 178, 46, 10, 48);

				disturb = false;
			}
			else if (selectVal == 1 && disturb)
			{
				tPositionLLA[1] += double(selection) / 10000000;
				tft.setTextColor(0x00ff00);
				printNumF(tPositionLLA[1], 7, 120, 198, 46, 11, 48);
				disturb = false;
			}
			else if (selectVal == 2 && disturb)
			{
				tPositionLLA[2] += double(selection) / 100;
				tft.setTextColor(0x00ff00);
				printNumF(tPositionLLA[2], 2, 120, 218, 46, 5, 48);

				disturb = false;
			}
			if (pressedButton())
			{
				button_delay = millis();
				scrSelect[3] = false;
				scrSelect[2] = true;
				selection = selectVal;
				disturb = true;
			}
		}
	}
}

void mainScrStatic()
{
	tft.fillScreen(0x0000);
	tft.setFont(SystemFont5x7);
	tft.setTextColor(0x0f00);
	if (defineRES)
	{
		tft.drawRoundRect(0, 0, 280, 22, 5, 0xC618);
		tft.drawRoundRect(109, 36, 211, 109, 5, 0xC618);
		tft.drawRoundRect(109, 159, 211, 81, 5, 0xC618);
		tft.drawCircle(300, 11, 9, 0xC618);
		tft.drawRoundRect(0, 26, 105, 214, 5, 0xc618);
		tft.setFont(SystemFont5x7);
		tft.setTextColor(0x0f00);
		tft.printAt(F("Reference Station"), 160, 26);
		tft.printAt(F("Rover Station"), 173, 150);
		tft.printAt(F("Lat"), 140, 39);
		tft.printAt(F("Lon"), 219, 39);
		tft.printAt(F("Alt"), 287, 39);
		tft.printAt(F("m"), 310, 51);
		tft.printAt(F("Week"), 118, 99);
		tft.printAt(F("Tow"), 165, 99);
		tft.printAt(F("Sat"), 211, 99);
		tft.printAt(F("SBAS"), 245, 99);
		tft.printAt(F("Bat"), 290, 99);
		tft.printAt(F("s"), 198, 113);
		tft.printAt(F("Hacc"), 116, 69);
		tft.printAt(F("Vacc"), 159, 69);
		tft.printAt(F("DRMSE"), 196, 69);
		tft.printAt(F("RMSEv"), 241, 69);
		tft.printAt(F("CEP"), 292, 69);
		tft.printAt(F("Runup time :"), 129, 129);
		tft.printAt(F("Lat"), 140, 164);
		tft.printAt(F("Lon"), 219, 164);
		tft.printAt(F("Alt"), 287, 164);
		tft.printAt(F("m"), 310, 176);
		tft.printAt(F("Hp"), 135, 189);
		tft.printAt(F("Vp"), 180, 189);
		tft.printAt(F("DRMSE"), 218, 189);
		tft.printAt(F("RMSEv"), 268, 189);
		tft.printAt(F("Age"), 124, 214);
		tft.printAt(F("SOG"), 172, 214);
		tft.printAt(F("COG"), 213, 214);
		tft.printAt(F("Sat"), 250, 214);
		tft.printAt(F("Bat"), 290, 214);
	}
	else if (defineSBAS)
	{
		tft.drawRoundRect(0, 0, 280, 22, 5, 0xC618);
		tft.drawRoundRect(109, 36, 211, 109, 5, 0xC618);
		tft.drawRoundRect(109, 159, 211, 81, 5, 0xC618);
		tft.drawCircle(300, 11, 9, 0xC618);
		tft.drawRoundRect(0, 26, 105, 214, 5, 0xc618);
		
		tft.printAt(F("Reference Station"), 160, 26);
		tft.printAt(F("Rover Station"), 173, 150);
		tft.printAt(F("Lat"), 140, 39);
		tft.printAt(F("Lon"), 219, 39);
		tft.printAt(F("Alt"), 287, 39);
		tft.printAt(F("m"), 310, 51);
		tft.printAt(F("Week"), 118, 99);
		tft.printAt(F("Tow"), 165, 99);
		tft.printAt(F("Sat"), 211, 99);
		tft.printAt(F("SBAS"), 245, 99);
		tft.printAt(F("Bat"), 290, 99);
		tft.printAt(F("s"), 198, 113);
		tft.printAt(F("Herr"), 130, 69);
		tft.printAt(F("Verr"), 175, 69);
		tft.printAt(F("RMSEh"), 218, 69);
		tft.printAt(F("RMSEv"), 268, 69);
		tft.printAt(F("Runup time :"), 129, 129);
		tft.printAt(F("Lat"), 140, 164);
		tft.printAt(F("Lon"), 219, 164);
		tft.printAt(F("Alt"), 287, 164);
		tft.printAt(F("m"), 310, 176);
		tft.printAt(F("Hp"), 135, 189);
		tft.printAt(F("Vp"), 180, 189);
		tft.printAt(F("DRMSE"), 218, 189);
		tft.printAt(F("RMSEv"), 268, 189);
		tft.printAt(F("Age"), 124, 214);
		tft.printAt(F("SOG"), 172, 214);
		tft.printAt(F("COG"), 213, 214);
		tft.printAt(F("Sat"), 250, 214);
		tft.printAt(F("Bat"), 290, 214);
		Serial.println("SBAAS");
	}
	mainScr = true;
	disturb = true;
	disturb2 = true;
	disturb3 = true;
}

void MainScrRES()
{
	if ((millis() - lcdUpdate >= 1000))
	{
		corPositionR(116, 175);
		r_state(116, 225);
		r_Stats(125, 200);
		Stats(115, 81);
		svStatus(5, true);
		lcdUpdate = millis();
		RS_state(116, 112);
		timer(30, 4);
		dateform(140, 4);
		curPositionRS(116, 50);
		startupTime(226, 129);
		rNewData = false;
	}
	if (millis() - recep > 100 && disturb3)
	{
		tft.fillCircle(300, 11, 8, 0x0000);
		disturb3 = false;
	}

}

void MainScrSBAS()
{
	if ((millis() - lcdUpdate >= 1000))
	{
		lcdUpdate = millis();
		corPositionR(116, 175);
		r_state(116, 225);
		r_Stats(125, 200);
		Stats(125, 81);
		svStatus(5, defineRES);
		RS_state(116, 112);
		timer(30, 4);
		dateform(140, 4);
		curPositionRS(116, 50);
		startupTime(226, 129);
		rNewData = false;

	}
	if (millis() - recep > 100 && disturb3)
	{
		tft.fillCircle(300, 11, 8, 0x0000);
		disturb3 = false;
	}

}

void llaToECEF(double* ECEF, double* lla, double gSep)   // transformation from LLA to ECEF coordinates system ( lla- input array [lat,lon,alt], ECEF - output array, geoSep - geoid separation)
{
	double lambdaS = atan(sq(1 - wf)* tan(lla[0] * PI / 180));  //geocentric latitude at mean sea-level 
	double rS = sqrt(sq(wR) / (1 + (1 / sq(1 - wf) - 1)*sq(sin(lambdaS)))); // radius at a surface point
	ECEF[0] = (rS*cos(lambdaS)*cos(lla[1] * PI / 180)) + ((lla[2] + gSep)*cos(lla[0] * PI / 180)*cos(lla[1] * PI / 180));
	ECEF[1] = (rS*cos(lambdaS)*sin(lla[1] * PI / 180)) + ((lla[2] + gSep)*cos(lla[0] * PI / 180)*sin(lla[1] * PI / 180));
	ECEF[2] = (rS*sin(lambdaS)) + ((lla[2] + gSep)*sin(lla[0] * PI / 180));
}

void ECEFtoENU(double * ENU, double* ECEF,double* tECEF, double* tlla)   // transformation from ECEF to ENU coordinates system ( tlla- input array [lat,lon,alt] of the reference position, tECEF - input array of the reference position, ECEF - input array of the measured,ENU - output array)
{
	ENU[0] = (-(ECEF[0] - tECEF[0])*sin(tlla[1] * PI / 180) + (ECEF[1] - tECEF[1])*cos(tlla[1] * PI / 180));
	ENU[1] = (-(ECEF[0] - tECEF[0])*sin(tlla[0] * PI / 180)*cos(tlla[1] * PI / 180) - (ECEF[1] - tECEF[1])*sin(tlla[0] * PI / 180)*sin(tlla[1] * PI / 180) + (ECEF[2] - tECEF[2])*cos(tlla[0] * PI / 180));
	ENU[2] = ((ECEF[0] - tECEF[0])*cos(tlla[0] * PI / 180)*cos(tlla[1] * PI / 180) + (ECEF[1] - tECEF[1])*cos(tlla[0] * PI / 180)*sin(tlla[1] * PI / 180) + (ECEF[2] - tECEF[2])*sin(tlla[0] * PI / 180));
}

void ECEFTolla(double* lla, double* ecef)   // transformation from ECEF to LLA coordinates system ( lla - output array [lat,lon,hae], ECEF - input array)
{
	double e1 = sqrt((sq(wR) - sq(wRb)) / sq(wR)); // first eccentricity
	double e2 = sqrt((sq(wR) - sq(wRb)) / sq(wRb)); // second eccentricity
	double p = sqrt(sq(ecef[0]) + sq(ecef[1]));
	double d = atan((ecef[2] * wR) / (p*wRb));
	double lat = atan((ecef[2] + (sq(e2)*wRb*pow(sin(d), 3))) / (p - (sq(e1)*wR*pow(cos(d), 3))))*(180 / PI);
	double lon = atan(ecef[1] / ecef[0])*(180 / PI);
	double N = wR / sqrt(1.0 - (sq(e1)*sq(sin(lat * (PI / 180)))));
	double alt = ((p / cos(lat * (PI / 180))) - N);
	lla[0] = lat;
	lla[1] = lon;
	lla[2] = alt; //HAE
}

double trueRange(double * posECEF, int sv) // calculation of range from the defined position to the defined satelltie (pythagoras)
{
	double range = sqrt(sq(Xs[sv - 1] - posECEF[0]) + sq(Ys[sv - 1] - posECEF[1]) + sq(Zs[sv - 1] - posECEF[2]));
	return range;
}

void poolEPH()
{
	for (int i = 0; i < sizeof(pollEPH); i++)Serial1.write(pollEPH[i]); // pool ephem
	tEPH = millis();
}

void losEPH(double * e, double * posECEF, double r, int sv) // e - output array, where LOS vector will be sotred
{
	e[0] = -(Xs[sv - 1] - posECEF[0]) / r;
	e[1] = -(Ys[sv - 1] - posECEF[1]) / r;
	e[2] = -(Zs[sv - 1] - posECEF[2]) / r;
	e[3] = 1;
}

void losAE(double * e, double * posLLA, int sv)
{
	double eLoc[4];
	double R[4][4];
	float Az = float(gps.sv.Az(sv));
	float El = float(gps.sv.El(sv));
	eLoc[0] = sin(Az*(PI / 180))*cos(El*(PI / 180));
	eLoc[1] = cos(Az*(PI / 180))*cos(El*(PI / 180));
	eLoc[2] = sin(El*(PI / 180));
	R[0][0] = -sin(posLLA[1] * (PI / 180));
	R[0][1] = -cos(posLLA[1] * (PI / 180))*sin(posLLA[0] * (PI / 180));
	R[0][2] = cos(posLLA[1] * (PI / 180))*cos(posLLA[0] * (PI / 180));
	R[1][0] = cos(posLLA[1] * (PI / 180));
	R[1][1] = -sin(posLLA[1] * (PI / 180))*sin(posLLA[0] * (PI / 180));
	R[1][2] = sin(posLLA[1] * (PI / 180))*cos(posLLA[0] * (PI / 180));
	R[2][0] = 0;
	R[2][1] = cos(posLLA[0] * (PI / 180));
	R[2][2] = sin(posLLA[0] * (PI / 180));
	for (int i = 0; i < 3; i++)e[i] = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int x = 0; x < 3; x++)
		{
			e[i] += R[i][x] * eLoc[x];
		}
	}
}

int Invert(double* A, int n) // matrix inversion from MatrixMath lib
{
	// A = input matrix AND result matrix
	// n = number of rows = number of columns in A (n x n)
	int pivrow;     // keeps track of current pivot row
	int k, i, j;      // k: overall index along diagonal; i: row index; j: col index
	int pivrows[n]; // keeps track of rows swaps to undo at end
	double tmp;      // used for finding max value and making column swaps

	for (k = 0; k < n; k++)
	{
		// find pivot row, the row with biggest entry in current column
		tmp = 0;
		for (i = k; i < n; i++)
		{
			if (abs(A[i*n + k]) >= tmp)   // 'Avoid using other functions inside abs()?'
			{
				tmp = abs(A[i*n + k]);
				pivrow = i;
			}
		}

		// check for singular matrix
		if (A[pivrow*n + k] == 0.0f)
		{
			Serial.println("Inversion failed");
			return 0;
		}

		// Execute pivot (row swap) if needed
		if (pivrow != k)
		{
			// swap row k with pivrow
			for (j = 0; j < n; j++)
			{
				tmp = A[k*n + j];
				A[k*n + j] = A[pivrow*n + j];
				A[pivrow*n + j] = tmp;
			}
		}
		pivrows[k] = pivrow;    // record row swap (even if no swap happened)

		tmp = 1.0f / A[k*n + k];    // invert pivot element
		A[k*n + k] = 1.0f;        // This element of input matrix becomes result matrix

								  // Perform row reduction (divide every element by pivot)
		for (j = 0; j < n; j++)
		{
			A[k*n + j] = A[k*n + j] * tmp;
		}

		// Now eliminate all other entries in this column
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				tmp = A[i*n + k];
				A[i*n + k] = 0.0f;  // The other place where in matrix becomes result mat
				for (j = 0; j < n; j++)
				{
					A[i*n + j] = A[i*n + j] - A[k*n + j] * tmp;
				}
			}
		}
	}

	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
	for (k = n - 1; k >= 0; k--)
	{
		if (pivrows[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				tmp = A[i*n + k];
				A[i*n + k] = A[i*n + pivrows[k]];
				A[i*n + pivrows[k]] = tmp;
			}
		}
	}
	return 1;
}

void posCorr(double * pC, double * rCorr, double * H, int Svs) //pC output array with position correction
{
	double tH[5][15];
	double adH[17];
	double pdH[5][17];
	double lsH[5][15];
	bool loop = true;
	for (int j = 0; j < 4; j++)
	{
		loop = true;
		for (int k = 0; k < 4; k++)
		{
			adH[(j * 4) + k] = 0;
			for (int i = 0; i < Svs; i++)
			{
				if (loop)
				{
					tH[j][i] = 0;
					tH[j][i] = H[(i * 4) + j];
				}
				adH[(j * 4) + k] += tH[j][i] * H[(i * 4) + k]; //adH matrix in line	
			}
			loop = false;
		}
	}
	if (Invert(adH, 4) == 1)
	{
		for (int k = 0; k < 4; k++)
		{
			pC[k] = 0;
			for (int i = 0; i < Svs; i++)
			{
				lsH[k][i] = 0;
				for (int j = 0; j < 4; j++)
				{
					lsH[k][i] += (adH[(k * 4) + j] * tH[j][i]);
				}
				pC[k] += lsH[k][i] * rCorr[i];
			}
		}
	}
}

void kalmanSet(double q, double r, int s)
{
	for (int i = 0; i < s*2; i++)
	{
		for (int j = 0; j < s*2; j++)
		{
			Xo[i] = 0;
			if (j < s)G[i][j] = 0;
			if (i < s && j < s)
			{
				if (i == j)
				{
					cQ[i][j] = q;
					R[i][j] = r;
					P[i][j] = 1;
					H[i][j] = 1;
					F[i][j] = 1;
				}
				else
				{
					cQ[i][j] = 0;
					R[i][j] = 0;
					P[i][j] = 0;
					H[i][j] = 0;
					F[i][j] = 0;
				}
			}
			else
			{
				
				if (i == j)
				{
					P[i][j] = 1;
					F[i][j] = 1;
				}
				else if (i < s && j == i + s)
				{
					F[i][j] = 1;
				}
				else
				{
					P[i][j] = 0;
					F[i][j] = 0;
					if (i < s) H[i][j] = 0; 
				}
			}
		}
	}
}

void kalmanProc(int s,double* z, double * out, double* in)  // S- number of states, z measurement vector input, out - filtered position output, in - initial values
{
	double timestep = double(recep - timestepPrev) / 1000;
	//Time update
	for (int x = 0; x < s * 2; x++)
	{
		for (int i = 0; i < s * 2; i++)
		{
			if (i < s)
			{
				Q_pred[x][i] = 0;
				K1[x][i] = 0;
				K[x][i] = 0;
				X1[i] = 0;
				if(x<s)Kinv[x*s + i] = 0;
			}
			if(x == 0)x_pred[i] = 0;
			X2[i] = 0;
			if (x<s)K2[x][i] = 0;
			E_pred[x][i] = 0;
			P_pred[x][i] = 0;
			P2[x][i] = 0;
			Q[x][i] = 0;
			P_pred2[x][i] = 0;
			for (int j = 0; j < s * 2; j++)
			{
				if (x == 0)
				{
					if (i < s && j == i)
					{
						F[i][j + s] = timestep;
						G[i][j] = sq(timestep) / 2;
						G[i + s][j] = timestep;
					}
					x_pred[i] += F[i][j] * in[j]; // state estimate prediciton
				}
				if (j < s && i < s)Q_pred[x][i] += G[x][j] * cQ[j][i];
				P_pred2[x][i] += F[x][j] * P[j][i];	
			}
		}
		for (int i = 0; i < s * 2; i++)
		{
			for (int j = 0; j < s * 2; j++)
			{
				E_pred[x][i] += P_pred2[x][j] * F[i][j]; 
				if (j < s)Q[x][i] += Q_pred[x][j] * G[i][j];	
			}
			P_pred[x][i] = E_pred[x][i] + Q[x][i]; // Error covariance prediction	
			for (int k = 0; k < s; k++)
			{
				K1[x][k] += P_pred[x][i] * H[k][i];
				K2[k][x] += H[k][i] * P_pred[i][x];
			}
		}
	}

	// Measurement update
	for (int i = 0; i < s; i++)
	{
		for (int j = 0; j < s; j++)
		{
			Kinv[i*s + j] = K2[i][j] + R[i][j];
		}	
	}
	Invert(Kinv, s);
	for (int k = 0; k < s * 2; k++)
	{
		P2[k][k] = 1;
		for (int i = 0; i < s; i++)
		{
			for (int j = 0; j < s; j++)
			{
				K[k][i] += K1[k][j] * Kinv[i*s + j]; // Kalman gain
			}
			if(k<1)X1[i] = z[i] - x_pred[i];
			if (k == i || k == i + s)	P2[k][i] = P2[k][i] - K[k][i];
		}
		for (int i = 0; i < s; i++)
		{
			X2[k] += K[k][i] * X1[i];
		}
		out[k] = x_pred[k] + X2[k];
		in[k] = out[k];
	}
// Error probability update
	for (int k = 0; k < s * 2; k++)
	{
		for (int i = 0; i < s * 2; i++)
		{
			P[k][i] = 0;
			for (int j = 0; j < s * 2; j++)
			{
				P[k][i] += P2[k][j] * P_pred[j][i];
			}
		}
	}
	timestepPrev = millis();
}

void _convert_float(char *buf, double num, int width, byte prec) // UTFT printNumF
{
	char format[20];

	sprintf(format, "%%%i.%if", width, prec);
	sprintf(buf, format, num);
}

void running_average(double* out, double* mes, int length) // average of the output position [ECEF] over defined interval (length) of the rover 
{
	if (avgCount == 0)for (int i = 0; i < 3; i++)AvgSum[i] = 0;
	for (int i = 0; i < 3; i++)
	{
		AvgSum[i] += mes[i];
	}
	avgCount++;
	for (int i = 0; i < 3; i++)
	{
		out[i] = (AvgSum[i] / avgCount);
	}
	if (avgCount >= length)avgCount = 0;
}

void AccStdDevRS(double* out, double* mesENU, int length) // RS standard deviations in each direction for the RS (input is ENU vector of meausred coordinates with respect to the true position)
{
	out[0] = sqrt(sq(mesENU[0]) + sq(mesENU[1])); // Actual Horizontal Error
	out[1] = mesENU[2];// Actual Vertical Error 
	double std[4];
	if (stdCount == 0)for (int i = 0; i < 3; i++)StdSum[i] = 0;
	for (int i = 0; i < 3; i++)
	{
		StdSum[i] += sq(mesENU[i]);
	}
	stdCount++;
	for (int i = 0; i < 3; i++)
	{
		std[i] = sqrt(StdSum[i] / stdCount);
	}
	out[2] = sqrt(sq(std[0]) + sq(std[1]));  // DRMSE (H)
	out[3] = 2 * sqrt(sq(std[0]) + sq(std[1]));  // 2DRMSE (H)
	out[4] = abs(std[2]);  // RMSE (V)
	out[5] = 0.62*std[1] + 0.56*std[0]; //  CEP
	out[6] = 0.833*(std[0] + std[1] + std[2]);  // SEP 90%
	if (stdCount >= length)stdCount = 0;
}

void PrecRover(double* out, double* mesENU, int length) // Rover precision actual and RMS H,V in terms of Standrd deviation with respect to mean position value.
{
	double std[4];
	out[0] = sqrt(sq(mesENU[0]) + sq(mesENU[1])); // Actual Horizontal Error
	out[1] = mesENU[2];// Actual Vertical Error 
	if (precCount == 0)for (int i = 0; i < 3; i++)PrecSum[i] = 0;
	for (int i = 0; i < 3; i++)
	{
		PrecSum[i] += sq(mesENU[i]);
	}
	precCount++;
	for (int i = 0; i < 3; i++)
	{
		std[i] = sqrt(PrecSum[i] / precCount);
	}
	out[2] = sqrt(sq(std[0]) + sq(std[1])); // RMS Horizontal Error
	out[3] = abs(std[2]);// RMS Vertical Error 
	if (precCount >= length)precCount = 0;
}

// TFT Print functions
void printNumI(long num, int x, int y, int length, char filler) // UTFT
{
	char buf[25];
	char st[27];
	boolean neg = false;
	int c = 0, f = 0;

	if (num == 0)
	{
		if (length != 0)
		{
			for (c = 0; c<(length - 1); c++)
				st[c] = filler;
			st[c] = 48;
			st[c + 1] = 0;
		}
		else
		{
			st[0] = 48;
			st[1] = 0;
		}
	}
	else
	{
		if (num<0)
		{
			neg = true;
			num = -num;
		}

		while (num>0)
		{
			buf[c] = 48 + (num % 10);
			c++;
			num = (num - (num % 10)) / 10;
		}
		buf[c] = 0;

		if (neg)
		{
			st[0] = 45;
		}

		if (length>(c + neg))
		{
			for (int i = 0; i<(length - c - neg); i++)
			{
				st[i + neg] = filler;
				f++;
			}
		}

		for (int i = 0; i<c; i++)
		{
			st[i + neg + f] = buf[c - i - 1];
		}
		st[c + neg + f] = 0;

	}

	tft.printAt(st, x, y);
}

void printNumF(double num, byte dec, int x, int y, char divider, int length, char filler) // UTFT
{
	char st[27];
	boolean neg = false;

	if (dec<1)
		dec = 1;
	else if (dec>8)
		dec = 8;

	if (num<0)
		neg = true;

	_convert_float(st, num, length, dec);

	if (divider != '.')
	{
		for (int i = 0; i<sizeof(st); i++)
			if (st[i] == '.')
				st[i] = divider;
	}

	if (filler != ' ')
	{
		if (neg)
		{
			st[0] = '-';
			for (int i = 1; i<sizeof(st); i++)
				if ((st[i] == ' ') || (st[i] == '-'))
					st[i] = filler;
		}
		else
		{
			for (int i = 0; i<sizeof(st); i++)
				if (st[i] == ' ')
					st[i] = filler;
		}
	}

	tft.printAt(st, x, y);
}

void dateform(int x, int y)
{
	char Date[10];
	int s_d = sprintf(Date, "%02d.%02d.%02d", gps.date.day(), gps.date.month(), gps.date.year());
	if (datePrev[1] != Date[1])
	{
		tft.setFont(Arial_big);
		tft.setTextColor(0x0f00);
		tft.printAt(Date, x, y);
		datePrev[1] = Date[1];
	}


#ifdef DEBUG

	Serial.print(F(" || DATE="));        Serial.print(Date);

#endif
}

void timer(int x, int y)
{

	char Time[10];
	int s_t = sprintf(Time, "%02d : %02d : %02d", gps.time.hour(), gps.time.minute(), gps.time.second());
	tft.setFont(Arial_big);
	tft.setTextColor(0x0f00);
	tft.printAt(Time, x, y);

#ifdef DEBUG
	Serial.print(F(" || TIME=")); Serial.print(Time);
#endif
}

void startupTime(int x, int y)
{
	char SU_tid[11];
	unsigned long t = millis();
	unsigned int hours = (t % 86400000) / 3600000;
	unsigned int minutes = ((t % 86400000) % 3600000) / 60000;
	unsigned int seconds = (((t % 86400000) % 3600000) % 60000) / 1000;
	int s_t = sprintf(SU_tid, "%03d : %02d : %02d", hours, minutes, seconds);
	tft.printAt(SU_tid, x, y);

#ifdef DEBUG
	Serial.print(F(" || S/U TIME=")); Serial.print(SU_tid);
#endif
}

void curPositionRS(int x, int y)
{
	tft.setFont(Arial_small);
	tft.setTextColor(0xffff);
	printNumF(gps.location.lat(), 5, x + 15 , y, 46, 8, 127);
	printNumF(gps.location.lng(), 5, x + 91, y, 46, 9, 48);
	if (gps.location.lat() > 0)tft.printAt("N", x, y);
	else tft.printAt("S", x , y);
	if (gps.location.lng() > 0)tft.printAt("E   ", x + 76, y);
	else tft.printAt("W", x + 76, y);
	printNumF(gps.altitude.meters(), 2, x + 148, y, 46, 7, 127);

#ifdef DEBUG

	Serial.print(F(" || RS_LAT="));        Serial.print(  Serial.print(gps.location.lat(), 7);
	Serial.print(F(" || RS_LNG="));        Serial.print(  Serial.print(gps.location.lng(), 7);

#endif
}

void corPositionR(int x, int y)
{
	tft.setFont(Arial_small);
	tft.setTextColor(0xffff);
	if (defineKalman)
	{
		printNumF(cLLA_F[0], 5, x + 15, y, 46, 8, 127);
		printNumF(cLLA_F[1], 5, x + 91, y, 46, 9, 48);
		if (cLLA_F[0] > 0)tft.printAt("N", x, y);
		else tft.printAt("S", x, y);
		if (cLLA_F[1] > 0)tft.printAt("E   ", x + 76, y);
		else tft.printAt("W", x + 76, y);
		printNumF(cLLA_F[2], 2, x + 148, y, 46, 7, 127);

#ifdef DEBUG

		Serial.print(F(" || cR_LAT_F="));        Serial.print(Serial.print(cLLA_F[0], 7);
		Serial.print(F(" || cR_LNG_F="));        Serial.print(Serial.print(cLLA_F[1], 7);

#endif
	}
	else 
	{
		printNumF(cLLA[0], 5, x + 15, y, 46, 8, 127);
		printNumF(cLLA[1], 5, x + 91, y, 46, 9, 48);
		if (cLLA[0] > 0)tft.printAt("N", x, y);
		else tft.printAt("S", x, y);
		if (cLLA[1] > 0)tft.printAt("E   ", x + 76, y);
		else tft.printAt("W", x + 76, y);
		printNumF(cLLA[2], 2, x + 148, y, 46, 7, 127);

#ifdef DEBUG

		Serial.print(F(" || cR_LAT="));        Serial.print(Serial.print(cLLA[0], 7);
		Serial.print(F(" || cR_LNG="));        Serial.print(Serial.print(cLLA[1], 7);

#endif
	}
}

void Stats(int x, int y)
{
	tft.setFont(Arial_small);
	tft.setTextColor(0xffff);
	if (defineRES)
	{
		if (RSAcc[0] > 99.9)printNumF(99.9, 1, x, y, 46, 4, 127);
		else printNumF(RSAcc[0], 1, x, y, 46, 4, 127);
		if (RSAcc[1] > 99.9 || RSAcc[1] < -99.9)printNumF(99.9, 1, x + 39, y, 46, 5, 127);
		else printNumF(RSAcc[1], 1, x + 39, y, 46, 5, 127);
		if (RSAcc[2] > 99.9)printNumF(99.9, 1, x + 84, y, 46, 4, 127);
		else printNumF(RSAcc[2], 1, x + 84, y, 46, 4, 127);
		if (RSAcc[4] > 99.9)printNumF(99.9, 1, x + 128, y, 46, 4, 127);
		else printNumF(RSAcc[4], 1, x + 128, y, 46, 4, 127);
		if (RSAcc[5] > 99.9)printNumF(99.9, 2, x + 173, y, 46, 4, 127);
		else printNumF(RSAcc[5], 1, x + 173, y, 46, 4, 127);
	}
	else
	{
		if (RSAcc[0] > 99.9)printNumF(99.9, 1, x, y, 46, 4, 127);
		else printNumF(RSAcc[0], 1, x, y, 46, 4, 127);
		if (RSAcc[1] > 99.9 || RSAcc[1] < -99.9)printNumF(99.9, 1, x + 39, y, 46, 5, 127);
		else printNumF(RSAcc[1], 1, x + 39, y, 46, 5, 127);
		if (RSAcc[2] > 99.9)printNumF(99.9, 1, x + 94, y, 46, 4, 127);
		else printNumF(RSAcc[2], 1, x + 94, y, 46, 4, 127);
		if (RSAcc[4] > 99.9)printNumF(99.9, 1, x + 140, y, 46, 4, 127);
		else printNumF(RSAcc[4], 1, x + 140, y, 46, 4, 127);	
	}
}

void r_Stats(int x, int y)
{
	if (rPrec[0] > 99.9)printNumF(99.9, 1, x, y, 46, 4, 127);
	else printNumF(rPrec[0], 1, x, y, 46, 4, 127);
	if (rPrec[1] > 99.9 || rPrec[1]< -99.9)printNumF(99.9, 1, x + 39, y, 46, 5, 127);
	else printNumF(rPrec[1], 1, x + 39, y, 46, 5, 127);
	if (rPrec[2] > 99.9)printNumF(99.9, 1, x + 94, y, 46, 4, 127);
	else printNumF(rPrec[2], 1, x + 94, y, 46, 4, 127);
	if (rPrec[3] > 99.9)printNumF(99.9, 1, x + 140, y, 46, 4, 127);
	else printNumF(rPrec[3], 1, x + 140, y, 46, 4, 127);
}

void svStatus(int x, bool res)
{
	tft.setFont(Arial_small);
	tft.setTextColor(0xffff);
	if (res)
	{
		for (int i = 0; i < rSatUsed; i++)
		{
			printNumI(rSatUsedList[i], x, i * 17 + 33, 2, 127);
			if (gps.ubxEPH.valid(rSatUsedList[i]))tft.fillRect(x + 20, 17 * (i)+35, 8, 5, 0x0f00);
			else tft.fillRect(x + 20, 17 * (i)+35, 8, 5, 0xf800);
			if (valid[rSatUsedList[i] - 1] == 1)tft.fillRect(x + 30, 17 * (i)+35, 8, 5, 0x0f00);
			else if (valid[rSatUsedList[i] - 1] == 2)tft.fillRect(x + 30, 17 * (i)+35, 8, 5, 0xffe0);
			else tft.fillRect(x + 30, 17 * (i)+35, 8, 5, 0xf800);
			if (dcR[rSatUsedList[i] - 1] > 99.9 || dcR[rSatUsedList[i] - 1] < -99.9)printNumF(99.9, 1, x + 44, i * 17 + 33, 46, 5, 127);
			else printNumF(dcR[rSatUsedList[i] - 1], 1, x + 44, i * 17 + 33, 46, 5, 127);
			if (corAge[rSatUsedList[i] - 1] > 99)printNumI(99, x + 80, i * 17 + 33, 2, 127);
			else printNumI(corAge[rSatUsedList[i] - 1], x + 80, i * 17 + 33, 2, 127); // 127 Special character (space of then number width)
		}
		for (int j = rSatUsed - 1; j < 11; j++)
		{
			tft.fillRect(x, j * 17 + 48, 98, 15, 0x0000);
		}
	}
	else
	{
		if (gps.ubxSBAS.status(2) == 0)
		{
			tft.fillRect(x, 31, 98, 190, 0x0000);
			tft.setFont(Arial_big);
			tft.setTextColor(0xf800);
			tft.printAt("NO SBAS", x + 10, 160);
			tft.setFont(Arial_small);
			tft.setTextColor(0xffff);
		}
		else
		{
			for (int i = 0; i < rSatUsed; i++)
			{
				printNumI(rSatUsedList[i], x, i * 17 + 33, 2, 127);
				if (valid[rSatUsedList[i] - 1] == 2)tft.fillRect(x + 20, 17 * (i)+35, 15, 5, 0xffe0);
				else tft.fillRect(x + 20, 17 * (i)+35, 15, 5, 0xf800);
				if (dcR[rSatUsedList[i] - 1] > 99.9 || dcR[rSatUsedList[i] - 1] < -99.9)printNumF(99.9, 1, x + 44, i * 17 + 33, 46, 5, 127);
				else printNumF(dcR[rSatUsedList[i] - 1], 1, x + 44, i * 17 + 33, 46, 5, 127);
				if (corAge[rSatUsedList[i] - 1] > 99)printNumI(99, x + 80, i * 17 + 33, 2, 127);
				else printNumI(corAge[rSatUsedList[i] - 1], x + 80, i * 17 + 33, 2, 127);
			}
			for (int j = rSatUsed - 1; j < 11; j++)
			{
				tft.fillRect(x, j * 17 + 48, 98, 15, 0x0000);
			}
		}
	}
}

void RS_state(int x, int y)
{
	tft.setFont(Arial_small);
	tft.setTextColor(0xffff);
	tft.printAt(String(gps.time.week()), x+1, y);
	printNumI(uint32_t(gps.time.tow()), x + 39, y, 6, 127);
	printNumI(gps.sv.totalSatUsed(), x + 98, y, 2, 127);
	if (gps.ubxSBAS.status(2) == 1)tft.printAt(F("EGNOS"), x + 122, y);
	else if (gps.ubxSBAS.status(2) == 0 && gps.ubxSBAS.status(1) != 0)tft.printAt(F("WAAS"), x + 124, y);
	else if (gps.ubxSBAS.status(2) == 2)tft.printAt(F("MSAS"), x + 124, y);
	else tft.printAt(F("      N/A      "), x + 120, y);
	float RSVolt = 12.2;
	printNumF(RSVolt, 1, x + 171, y, 46, 4, 127);
	for (int i = 1; i < 5; i++)
	{
		Serial.print(gps.ubxSBAS.status(i));
		Serial.print(", ");
	}
	Serial.println();
	/*for (int i = 0; i < 32; i++)
	{
		Serial.print(gps.ubxSBAS.valid(i));
		Serial.print(", ");
		Serial.print(gps.ubxSBAS.fast(i));
		Serial.print(", ");
		Serial.print(gps.ubxSBAS.iono(i));
		Serial.print(" - ");
	}
	Serial.println();*/
}

void r_state(int x, int y)
{
	printNumI(long((millis() - recep)/1000), x+8 , y, 3, 48);
	printNumF(rSpd, 1, x + 46, y, 46, 5, 127);
	printNumI(rCrs, x + 97, y, 3, 48);
	printNumI(rSatUsed, x + 138, y, 2, 127);
	printNumF(99.9, 1, x + 171, y, 46, 4, 127);
}
