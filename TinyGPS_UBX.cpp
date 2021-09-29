/*
TinyGPS_UBX is modified library based on the TinyGPS++. The library add features to the original TinyGPS++
to parse some U-blox UBX sentence such as SBAS, RAW and AID. The library was
developed to support the low cost DGPS-Arduino project. 
Modified by Martin Svaton.
*/

#include "TinyGPS_UBX.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#define _GPRMCterm   "GPRMC"
#define _GPGGAterm   "GPGGA"



TinyGPSPlus::TinyGPSPlus()
  :  parity(0)
  ,  isChecksumTerm(false)
  ,  curSentenceType(GPS_SENTENCE_OTHER)
  ,  curTermNumber(0)
  ,  curTermOffset(0)
  ,  sentenceHasFix(false)
  ,  customElts(0)
  ,  customCandidates(0)
  ,  encodedCharCount(0)
  ,  sentencesWithFixCount(0)
  ,  failedChecksumCount(0)
  ,  passedChecksumCount(0)
{
  term[0] = '\0';
}

//
// public methods
//

bool TinyGPSPlus::encode(byte c)
{
  ++encodedCharCount;
  if (UBXflag < 2)
  {
	  switch (c)
	  {
	  case ',': // term terminators
		  parity ^= (uint8_t)c;
	  case '\r':
	  case '\n':
	  case '*':
	  {
		  bool isValidSentence = false;
		  if (curTermOffset < sizeof(term))
		  {
			  term[curTermOffset] = 0;
			  isValidSentence = endOfTermHandler();
		  }
		  ++curTermNumber;
		  curTermOffset = 0;
		  isChecksumTerm = c == '*';
		  return isValidSentence;
	  }
	  break;
	  case 0xB5:
	  {		  
		 UBXflag = 1;	 
      }
	  break;
	  case 0x62:
	  {
		  if (UBXflag == 1)
		  {
			  UBXflag = 2;
			  counter = 0;
			  CK_A = 0;
			  CK_B = 0;
			  UBXckError = 0;
			  UBXlength = 7;
			  data[0] = 0x00;
			  data[1] = 0x00;
			  data[2] = 0x00;
		  }
		  else UBXflag = 0;
	  }
	  break;

	  case '$': // sentence begin

		  curTermNumber = curTermOffset = 0;
		  parity = 0;
		  curSentenceType = GPS_SENTENCE_OTHER;
		  isChecksumTerm = false;
		  sentenceHasFix = false;
		  return false;

	  default: // ordinary characters
		  if (curTermOffset < sizeof(term) - 1)
			  term[curTermOffset++] = c;
		 
		  if (!isChecksumTerm)
			  parity ^= c;
		  return false;
	  }

	  return false;
  }

  else if (UBXflag == 2)
  {
	  
	  data[counter] = c;
	 
	  if (counter <= UBXlength && data[0] == 0x01 && data[1] == 0x32)  // UBX NAV-SBAS
	  {
		  
		  if (counter == 2)UBXlength = c + 5;
		  if (counter < UBXlength )   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;
			 
		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  parseSBAS(); 
				  ubxSBAS.SBASavail = true;
			  }
			  else UBXckError = 1;
			  UBXflag = 0;
		  }
	  }
	  else if (counter <= UBXlength && data[0] == 0x02 && data[1] == 0x10)  //UBX RXM-RAW
	  {
		  if (counter == 10)UBXlength = 8+24*c + 5; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;
			 
		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  parseRAW();		  
			 
			  }
			  else UBXckError = 1;
			 
			  UBXflag = 0;

			
		  }
		  
	  }
	  else if (counter <= UBXlength && data[0] == 0x0B && data[1] == 0x01) // UBX AID-INI
	  {
		  UBXlength = 53; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;

		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  parseINI();
				  location.posecef = true;
				 
			  }
			  else UBXckError = 1;
			  UBXflag = 0;

		
		  }

	  }
	  else if (counter <= UBXlength && data[0] == 0x01 && data[1] == 0x22) // UBX NAV-CLOCK
	  {
		  union
		  {
			  long i;
			  byte c[4];
		  }union_int;
		  UBXlength = 25; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;

		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  union_int.c[0] = data[8];
				  union_int.c[1] = data[9];
				  union_int.c[2] = data[10];
				  union_int.c[3] = data[11];
				  time.rcB = double(union_int.i) * pow(10.0,-9);
				  time.CLOCKavail = true;
			  }
			  else UBXckError = 1;
			  UBXflag = 0;

		  }

	  }
	  else if (counter <= UBXlength && data[0] == 0x01 && data[1] == 0x20) //UBX NAV-TIMEGPS
	  {
		 
		  UBXlength = 21; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;
		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  parseTIMEGPS();
				  time.TIMEGPSavail = true;
			  }
			  else UBXckError = 1;
			  UBXflag = 0;

		  }

	  }
	  else if (counter <= UBXlength && data[0] == 0x01 && data[1] == 0x30) //UBX NAV-SVINFO
	  {
		  UBXlength = data[8]*12 + 13; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;

		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  parseSVINFO();
				  sv.SVINFO = true;
			  }
			  else UBXckError = 1;
			  UBXflag = 0;

		  }

	  }
	  else if (counter <= UBXlength && data[0] == 0x01 && data[1] == 0x02) //UBX NAV-POSLLH
	  {
		  UBXlength = 28 + 5; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;

		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  UBXckError = 0;
				  parsePOSLLH();
				  location.posllh = true;
				 
			  }
			  else UBXckError = 1;
			  UBXflag = 0;

		  }

	  }
	  else if (counter <= UBXlength && data[0] == 0x01 && data[1] == 0x01) //UBX NAV-POSECEF
	  {
		  UBXlength = 20 + 5; // length of UBX include ID
		  if (counter < UBXlength)   // UBX checksum
		  {
			  CK_A += data[counter - 1];
			  CK_B += CK_A;
			  CK_A &= 0xff;
			  CK_B &= 0xff;

		  }
		  if (counter == UBXlength)
		  {
			  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
			  {
				  parsePOSECEF();
				  location.posecef = true;
			  }
			  UBXflag = 0;
		  }
	  }
	  else if (counter <= UBXlength && data[0] == 0x0B && data[1] == 0x31 ) //UBX AID-EPH
	  {
		 	  ubxEPH.on = true;
			  if (data[2] == 0x68)
			  {
				  UBXlength = 109; // length of UBX include ID
				  int SV = data[4];
				  if (counter < UBXlength + 1)   // UBX checksum
				  {
					  CK_A += data[counter - 2];
					  CK_B += CK_A;
					  CK_A &= 0xff;
					  CK_B &= 0xff;
				  }

				  if (counter == UBXlength)
				  {
					  if (CK_A == data[UBXlength - 1] && CK_B == data[UBXlength])
					  {
						  UBXckError = 0;
						  parseEPH(SV);
					  }
					  else UBXckError = 1;
					  UBXflag = 0;
					  UBXlength = 0;
				  }
			  }
		  
	  }
	  else if(counter > 3)UBXflag = 0;
	  counter++;
  }
}


//
// internal utilities
//

void TinyGPSPlus::parseEPH(int sv)
{
	union {
		unsigned short s;
		unsigned char c[2];
	} union_unsigned_short;
	union {
		short s;
		unsigned char c[2];
	} union_short;
	union {
		unsigned int i;
		unsigned char c[4];
	} union_unsigned_int;
	union {
		int i;
		unsigned char c[4];
	} union_int;

	ubxEPH.avail[sv-1] = true;
	//a_f2
	ubxEPH.a2[sv - 1] = ((double)data[38]) * pow(2.0, -55);
	
	//a_f1 *10^12
	union_short.c[0] = data[36];
	union_short.c[1] = data[37];
	ubxEPH.a1[sv - 1] = ((double)union_short.s) * pow(2.0, -43)* pow(10.0,12);

	//a_f0 *10^5
	union_int.c[0] = data[40];
	union_int.c[1] = data[41];
	union_int.c[2] = data[42];
	union_int.i = union_int.i >> 2;

	if ((union_int.c[2] >> 5) & 0x01)
	{
		union_int.c[2] = union_int.c[2] | 0xC0;
		union_int.c[3] = 0xff;
	}
	else
	{
		union_int.c[2] = union_int.c[2] & 0x3F;
		union_int.c[3] = 0x00;
	}
	ubxEPH.a0[sv-1] = ((double)union_int.i) * pow(2.0, -31) * pow(10.0,5);

	//t_oc [s]
	union_unsigned_short.c[0] = data[32];
	union_unsigned_short.c[1] = data[33];
	ubxEPH.toc[sv - 1] = ((double)union_unsigned_short.s) * pow(2.0, 4);
	// M_0 Mean anomaly at reference time [rad]
	union_int.c[0] = data[52];
	union_int.c[1] = data[53];
	union_int.c[2] = data[54];
	union_int.c[3] = data[48];
	ubxEPH.m_0[sv - 1] = ((double)union_int.i) * pow(2.0, -31) * Pi;
	//ecc - Eccentricity [-]
	union_unsigned_int.c[0] = data[60];
	union_unsigned_int.c[1] = data[61];
	union_unsigned_int.c[2] = data[62];
	union_unsigned_int.c[3] = data[56];
	ubxEPH.ecc[sv - 1] = ((double)union_unsigned_int.i) * pow(2.0, -33);
	//deltan - Mean motion difference [rad *10^9]
	union_short.c[0] = data[49];
	union_short.c[1] = data[50];
	ubxEPH.dn[sv - 1] = ((double)union_short.s) * pow(2.0, -43)* Pi * pow(10.0,9);
	//sqrtA - Square root of the semi-major axis [m^0.5]
	union_unsigned_int.c[0] = data[68];
	union_unsigned_int.c[1] = data[69];
	union_unsigned_int.c[2] = data[70];
	union_unsigned_int.c[3] = data[64];
	ubxEPH.sqrtA[sv - 1] = ((double)union_unsigned_int.i) * pow(2.0, -19);
	//omega - Argument of Perigee [rad]
	union_int.c[0] = data[96];
	union_int.c[1] = data[97];
	union_int.c[2] = data[98];
	union_int.c[3] = data[92];
	ubxEPH.omega[sv - 1] = ((double)union_int.i) * pow(2.0, -31)*Pi;
	//i_0 - Inclination Angle [rad]
	union_int.c[0] = data[88];
	union_int.c[1] = data[89];
	union_int.c[2] = data[90];
	union_int.c[3] = data[84];
	ubxEPH.i_0[sv - 1] = ((double)union_int.i) * pow(2.0, -31) *Pi;
	//OMEGA_0 - Longitude of Ascending Node of Orbit Plane at Weekly Epoch [rad]
	union_int.c[0] = data[80];
	union_int.c[1] = data[81];
	union_int.c[2] = data[82];
	union_int.c[3] = data[76];
	ubxEPH.om_0[sv - 1] = ((double)union_int.i) * pow(2.0, -31) *Pi;
	//OMEGADOT - Rate of right ascension [rad *10^9]
	union_int.c[0] = data[100];
	union_int.c[1] = data[101];
	union_int.c[2] = data[102];

	if ((union_int.c[2] >> 7) & 0x01)
	{
		union_int.c[3] = 0xff;
	}
	else
	{
		union_int.c[3] = 0x00;
	}
	ubxEPH.om_dot[sv - 1] = ((double)union_int.i) * pow(2.0, -43) * Pi* pow(10.0,9);

	//IDOT - Rate of Inclination Angle  [rad *10^11]
	union_short.c[0] = data[104];
	union_short.c[1] = data[105];
	union_short.s = union_short.s >> 2;
	if ((union_short.c[1] >> 5) & 0x01)
	{
		union_int.c[1] = union_short.c[1] | 0xC0;
	}
	else
	{
		union_int.c[1] = union_short.c[1] & 0x3F;
	}
	ubxEPH.i[sv - 1] = ((double)union_short.s) * pow(2.0, -43) * Pi * pow(10.0,11);
	// Correction to argument of latitude
	//C_ucos  [rad *10^6]
	union_short.c[0] = data[57];
	union_short.c[1] = data[58];
	ubxEPH.cuc[sv - 1] = ((double)union_short.s) * pow(2.0, -29)* pow(10.0,6);

	//C_usin  [rad *10^6]
	union_short.c[0] = data[65];
	union_short.c[1] = data[66];
	ubxEPH.cus[sv - 1] = ((double)union_short.s) * pow(2.0, -29)* pow(10.0,6);
	// Correction to orbit radius
	//C_rcos [m]
	union_short.c[0] = data[93];
	union_short.c[1] = data[94];
	ubxEPH.crc[sv - 1] = ((double)union_short.s) * pow(2.0, -5);

	//C_rsin [m]
	union_short.c[0] = data[44];
	union_short.c[1] = data[45];
	ubxEPH.crs[sv - 1] = ((double)union_short.s) * pow(2.0, -5);
	// Correction to inclination radius
	//C_icos [rad *10^9]
	union_short.c[0] = data[77];
	union_short.c[1] = data[78];
	ubxEPH.cic[sv - 1] = ((double)union_short.s) * pow(2.0, -29)* pow(10.0,9);

	//C_isin  [rad *10^9]
	union_short.c[0] = data[85];
	union_short.c[1] = data[86];
	ubxEPH.cis[sv - 1] = ((double)union_short.s) * pow(2.0, -29)* pow(10.0,9);

	//t_oe [s]
	union_unsigned_short.c[0] = data[73];
	union_unsigned_short.c[1] = data[74];
	ubxEPH.toe[sv-1] = ((double)union_unsigned_short.s) * pow(2.0, 4);
	
	/*//testing parameters
	ubxEPH.toe[8] = 14400;
	ubxEPH.sqrtA[8] = 5153.68885040;
	ubxEPH.dn[8] = 6.6770638408; //*10^-9
	ubxEPH.m_0[8] = 1.947876;
	ubxEPH.omega[8] = 0.233996741372;
	ubxEPH.ecc[8] = 0.00439238466788;
	ubxEPH.i[8] = -33.1442377334;	//*10^11
	ubxEPH.i_0[8] = 0.9002982524;
	ubxEPH.om_0[8] = -1.09222818;
	ubxEPH.om_dot[8] = -9.3028875026;	//*10^-9
	ubxEPH.crc[8] = 283.21875;
	ubxEPH.crs[8] = -31.96875;
	ubxEPH.cuc[8] = -1.55344605446;	//*10^-6
	ubxEPH.cus[8] = 3.33040952682;	//*10^-6
	ubxEPH.cic[8] = -87.5443220139;	//*10^-9
	ubxEPH.cis[8] = 143.423676491;	//*10^-9*/
	Serial.println(sv);
	//Serial.println(ubxEPH.avail[sv - 1]);
	/*Serial.println(ubxEPH.toc[sv - 1]);
	Serial.println(ubxEPH.m_0[sv - 1],9);
	Serial.println(ubxEPH.ecc[sv - 1],9);
	Serial.println(ubxEPH.dn[sv - 1], 9);
	Serial.println(ubxEPH.sqrtA[sv - 1], 9);
	Serial.println(ubxEPH.omega[sv - 1], 9);
	Serial.println(ubxEPH.i_0[sv - 1], 9);
	Serial.println(ubxEPH.om_0[sv - 1], 9);
	Serial.println(ubxEPH.om_dot[sv - 1], 9);
	Serial.println(ubxEPH.i[sv - 1], 9);
	Serial.println(ubxEPH.crc[sv - 1], 9);
	Serial.println(ubxEPH.crs[sv - 1], 9);
	Serial.println(ubxEPH.cuc[sv - 1], 9);
	Serial.println(ubxEPH.cus[sv - 1], 9);
	Serial.println(ubxEPH.cic[sv - 1], 9);
	Serial.println(ubxEPH.cis[sv - 1], 9);
	Serial.println(ubxEPH.toe[sv - 1]);
	Serial.println(ubxEPH.a2[sv - 1],9);
	Serial.println(ubxEPH.a1[sv - 1],9);
	Serial.println(ubxEPH.a0[sv - 1],9);*/
}
void TinyGPSPlus::parseSVINFO()
{
	int c = 0;
	int d = 0;
	union
	{
		long a;
		byte d[4];
	}union_long;

	for (int i = 0; i < data[8]; i++)
	{
		int sat = data[12 + (12 * i) + 1];
		if (sat <= 32)
		{
			sv.satelliteEl[sat-1] = data[12 + (12 * i) + 5];
			sv.satelliteAz[sat-1] = word(data[12 + (12 * i) + 7],data[12 + (12 * i) + 6]);
			union_long.d[0] = data[12 + (12 * i) + 8];
			union_long.d[1] = data[12 + (12 * i) + 9];
			union_long.d[2] = data[12 + (12 * i) + 10];
			union_long.d[3] = data[12 + (12 * i) + 11];
			sv.res[sat-1] = double(union_long.a) / 100;
			if ((data[12 + (12 * i) + 2]) & 0x01)
			{
				sv.sat[d] = sat;
				d++;
			}
			
			sv.satellitesInView[c] = sat;
			c++;
		}
	}
	sv.totSatUsed = d;
	sv.totSatinView = c;
}
void TinyGPSPlus::parseINI()
{
	time.w = word(data[23], data[22]); //Week number
	union
	{
		 long i;
		byte c[4];
	}union_long;

	union_long.c[0] = data[4];
	union_long.c[1] = data[5];
	union_long.c[2] = data[6];
	union_long.c[3] = data[7];
	location.eX = double(union_long.i) / 100;  // ecefX
	union_long.c[0] = data[8];
	union_long.c[1] = data[9];
	union_long.c[2] = data[10];
	union_long.c[3] = data[11];
	location.eY = double(union_long.i) / 100;  // ecefY
	union_long.c[0] = data[12];
	union_long.c[1] = data[13];
	union_long.c[2] = data[14];
	union_long.c[3] = data[15];
	location.eZ = double(union_long.i) / 100;  // ecefZ
	/*union_long.c[0] = data[24];
	union_long.c[1] = data[25];
	union_long.c[2] = data[26];
	union_long.c[3] = data[27];
	time.ToW = double(union_long.i) / 1000;  // tow*/
	
}
void TinyGPSPlus::parseTIMEGPS()
{
	union
	{
		long i;
		byte c[4];
	}union_long;

	union_long.c[0] = data[8];
	union_long.c[1] = data[9];
	union_long.c[2] = data[10];
	union_long.c[3] = data[11];
	time.ftoW = double(union_long.i)* pow(10.0, -9);
	union_long.c[0] = data[4];
	union_long.c[1] = data[5];
	union_long.c[2] = data[6];
	union_long.c[3] = data[7];
	time.ToW = (double(union_long.i)* pow(10.0, -3)) + time.ftoW;
	time.w = word(data[13], data[12]); //Week number
}
void TinyGPSPlus::parsePOSLLH()
{
	union
	{
		long i;
		byte c[4];
	}union_long;
	
	union_long.c[0] = data[8];
	union_long.c[1] = data[9];
	union_long.c[2] = data[10];
	union_long.c[3] = data[11];
	location.ubxLng = double(union_long.i)* pow(10.0, -7);
	union_long.c[0] = data[12];
	union_long.c[1] = data[13];
	union_long.c[2] = data[14];
	union_long.c[3] = data[15];
	location.ubxLat = double(union_long.i)* pow(10.0, -7);
	union_long.c[0] = data[20];
	union_long.c[1] = data[21];
	union_long.c[2] = data[22];
	union_long.c[3] = data[23];
	altitude.val = union_long.i;
	union_long.c[0] = data[16];
	union_long.c[1] = data[17];
	union_long.c[2] = data[18];
	union_long.c[3] = data[19];
	altitude.elips = double(union_long.i)* pow(10.0, -3);
	altitude.geo = altitude.elips - altitude.meters();
	altitude.updated = true;
	location.updated = true;
	altitude.valid = true;
}
void TinyGPSPlus::parsePOSECEF()
{
	union
	{
		long i;
		byte c[4];
	}union_long;


	union_long.c[0] = data[8];
	union_long.c[1] = data[9];
	union_long.c[2] = data[10];
	union_long.c[3] = data[11];
	location.eX = double(union_long.i) / 100;  // ecefX
	union_long.c[0] = data[12];
	union_long.c[1] = data[13];
	union_long.c[2] = data[14];
	union_long.c[3] = data[15];
	location.eY = double(union_long.i) / 100;  // ecefY
	union_long.c[0] = data[16];
	union_long.c[1] = data[17];
	union_long.c[2] = data[18];
	union_long.c[3] = data[19];
	location.eZ = double(union_long.i) / 100;  // ecefZ
}
void TinyGPSPlus::parseRAW()
{
	ubxRAW.RAW[0] = data[10]; //number of measured satellites
	ubxRAW.RAW[1] = word(data[9],data[8]); //Week number
	union 
	{
			unsigned long i;
			byte c[4];
	}union_unsigned_int;

		union_unsigned_int.c[0] = data[4];
		union_unsigned_int.c[1] = data[5];
		union_unsigned_int.c[2] = data[6];
		union_unsigned_int.c[3] = data[7];
		ubxRAW.RAW[2] = union_unsigned_int.i/1000;  // s of the GPS week
		for (int x = 0; x < data[10]; x++) // pseudoranges data[10]
		{
			int SV = data[12 + 24 * x + 20];
			union_unsigned_int.c[0] = data[12 + 24 * x + 12];
			union_unsigned_int.c[1] = data[12 + 24 * x + 13];
			union_unsigned_int.c[2] = data[12 + 24 * x + 14];
			union_unsigned_int.c[3] = data[12 + 24 * x + 15];
			unsigned long raw1 = union_unsigned_int.i;
			union_unsigned_int.c[0] = data[12 + 24 * x + 8];
			union_unsigned_int.c[1] = data[12 + 24 * x + 9];
			union_unsigned_int.c[2] = data[12 + 24 * x + 10];
			union_unsigned_int.c[3] = data[12 + 24 * x + 11];
			unsigned long raw2 = union_unsigned_int.i;
			ubxRAW.Prange[SV-1] = iee754double(raw1, raw2);
		}
}
void TinyGPSPlus::parseSBAS()
{
	union {
		long i;
		unsigned char c[4];
	} union_long;
	union {
		short s;
		unsigned char b[2];
	} union_short;

	union_long.c[0] = data[4];
	union_long.c[1] = data[5];
	union_long.c[2] = data[6];
	union_long.c[3] = data[7];
	ubxSBAS.SBAS[0] = (union_long.i) /1000; //time of the week
	ubxSBAS.SBAS[1] = data[9]; //SBAS Mode
	ubxSBAS.SBAS[2] = data[10]; //system (1 =EGNOS)
	ubxSBAS.SBAS[3] = data[12]; //SV numbers following
	for (int i = 0; i < 32;i++)ubxSBAS.SBASvalid[i] = false;
	for (int i = 0; i < ubxSBAS.SBAS[3]; i++)
	{
		int sat = data[16 + (12 * i)];
		if (data[16 + (12 * i) + 3] == 0x10 ) //select GPS
		{
				union_short.b[0] = data[16 + (12 * i) + 6];
				union_short.b[1] = data[16 + (12 * i) + 7];
				ubxSBAS.SBASfast[sat - 1] = double(union_short.s) / 100; //fast correction
				union_short.b[0] = data[16 + (12 * i) + 10];
				union_short.b[1] = data[16 + (12 * i) + 11];
				ubxSBAS.SBASiono[sat - 1] = double(union_short.s) / 100; //ionospheric correction 

				if (ubxSBAS.SBASiono[sat - 1] != 0 || ubxSBAS.SBASfast[sat - 1] != 0)
				{
					ubxSBAS.SBASvalid[sat - 1] = true;
				}
				else
				{
					ubxSBAS.SBASvalid[sat - 1] = false;
				}
		}
	}
}

int TinyGPSPlus::fromHex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

// static
// Parse a (potentially negative) number with up to 3 decimal digits -xxxx.yyy
int32_t TinyGPSPlus::parseDecimal(const char *term)
{
  bool negative = *term == '-';
  if (negative) ++term;
  int32_t ret = 100 * (int32_t)atol(term);
  while (isdigit(*term)) ++term;
  if (*term == '.' && isdigit(term[1]))
  {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM
void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg)
{
	uint32_t leftOfDecimal = (uint32_t)atol(term);
	uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
	uint32_t multiplier = 10000000UL;
	uint32_t tenMillionthsOfMinutes = minutes * multiplier;

	deg.deg = (int16_t)(leftOfDecimal / 100);

	while (isdigit(*term))
		++term;

	if (*term == '.')
		while (isdigit(*++term))
		{
			multiplier /= 10;
			tenMillionthsOfMinutes += (*term - '0') * multiplier;
		}

	deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
	deg.negative = false;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPSPlus::endOfTermHandler()
{
  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm)
  {
    byte checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
    if (checksum == parity)
    {
      passedChecksumCount++;
      if (sentenceHasFix)
        ++sentencesWithFixCount;

      switch(curSentenceType)
      {
      case GPS_SENTENCE_GPRMC:
        date.commit();
        time.commit();
        if (sentenceHasFix)
        {
			if (!location.posllh)location.commit();
           speed.commit();
		   if (!location.posllh) course.commit();
        }
        break;
      case GPS_SENTENCE_GPGGA:
        //time.commit();
        if (sentenceHasFix && !location.posllh)
        {
			 location.commit();
			 altitude.commit();
        }
        satellites.commit();
        hdop.commit();
        break;
	 
	  }

      // Commit all custom listeners of this sentence type
      for (TinyGPSCustom *p = customCandidates; p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0; p = p->next)
         p->commit();
      return true;
    }

    else
    {
      ++failedChecksumCount;
    }

    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0)
  {
    if (!strcmp(term, _GPRMCterm))
      curSentenceType = GPS_SENTENCE_GPRMC;
    else if (!strcmp(term, _GPGGAterm))
      curSentenceType = GPS_SENTENCE_GPGGA;
    else
      curSentenceType = GPS_SENTENCE_OTHER;

    // Any custom candidates of this sentence type?
    for (customCandidates = customElts; customCandidates != NULL && strcmp(customCandidates->sentenceName, term) < 0; customCandidates = customCandidates->next);
    if (customCandidates != NULL && strcmp(customCandidates->sentenceName, term) > 0)
       customCandidates = NULL;

    return false;
  }

  if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    switch(COMBINE(curSentenceType, curTermNumber))
  {
    case COMBINE(GPS_SENTENCE_GPRMC, 1): // Time in both sentences
    //case COMBINE(GPS_SENTENCE_GPGGA, 1):
      time.setTime(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      sentenceHasFix = term[0] == 'A';
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(GPS_SENTENCE_GPGGA, 2):
      if (!location.posllh)
		location.setLatitude(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(GPS_SENTENCE_GPGGA, 3):
      location.rawNewLatData.negative = term[0] == 'S';
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(GPS_SENTENCE_GPGGA, 4):
		if (!location.posllh)
		location.setLongitude(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(GPS_SENTENCE_GPGGA, 5):
      location.rawNewLngData.negative = term[0] == 'W';
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      speed.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      course.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
      date.setDate(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      sentenceHasFix = term[0] > '0';
	  satellites.fix = atoi(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
      satellites.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 8): // HDOP
      hdop.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
		if (!location.posllh)altitude.set(term);
      break;
	case COMBINE(GPS_SENTENCE_GPGGA, 11): // Geoid Separation (GPGGA)
		if (!location.posllh)
		{
			altitude.geo = atof(term);
			altitude.elips = altitude.geo + altitude.meters();
		}
		time.end = true;
	  break;
  }

  // Set custom values as needed
  for (TinyGPSCustom *p = customCandidates; p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0 && p->termNumber <= curTermNumber; p = p->next)
    if (p->termNumber == curTermNumber)
         p->set(term);

  return false;
}

/* static */
double TinyGPSPlus::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double TinyGPSPlus::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *TinyGPSPlus::cardinal(double course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

void TinyGPSLocation::commit()
{
   rawLatData = rawNewLatData;
   rawLngData = rawNewLngData;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSLocation::setLatitude(const char *term)
{
   TinyGPSPlus::parseDegrees(term, rawNewLatData);
}

void TinyGPSLocation::setLongitude(const char *term)
{
   TinyGPSPlus::parseDegrees(term, rawNewLngData);
}
double TinyGPSLocation::lat()
{
	updated = false;
	if (!posllh)
	{
		double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
		return rawLatData.negative ? -ret : ret;
	}
	else return ubxLat;
}

double TinyGPSLocation::lng()
{
	updated = false;
	if (!posllh)
	{
		double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
		return rawLngData.negative ? -ret : ret;
	}
	else return ubxLng;
}

void TinyGPSDate::commit()
{
   date = newDate;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSTime::commit()
{
   time = newTime;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSTime::setTime(const char *term)
{
	newTime = (uint32_t)TinyGPSPlus::parseDecimal(term);
}

void TinyGPSDate::setDate(const char *term)
{
   newDate = atol(term);
}

uint16_t TinyGPSDate::year()
{
   updated = false;
   uint16_t year = date % 100;
   return year + 2000;
}

uint8_t TinyGPSDate::month()
{
   updated = false;
   return (date / 100) % 100;
}

uint8_t TinyGPSDate::day()
{
   updated = false;
   return date / 10000;
}

uint8_t TinyGPSTime::hour()
{
   updated = false;
   return time / 1000000;
}

uint8_t TinyGPSTime::minute()
{
   updated = false;
   return (time / 10000) % 100;
}

uint8_t TinyGPSTime::second()
{
   updated = false;
   return (time / 100) % 100;
}

uint8_t TinyGPSTime::centisecond()
{
   updated = false;
   return time % 100;
}

void TinyGPSDecimal::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSDecimal::set(const char *term)
{
   newval = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSInteger::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSInteger::set(const char *term)
{
   newval = atol(term);
}

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   lastCommitTime = 0;
   updated = valid = false;
   sentenceName = _sentenceName;
   termNumber = _termNumber;
   memset(stagingBuffer, '\0', sizeof(stagingBuffer));
   memset(buffer, '\0', sizeof(buffer));

   // Insert this item into the GPS tree
   gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit()
{
   strcpy(this->buffer, this->stagingBuffer);
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSCustom::set(const char *term)
{
   strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer));
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int termNumber)
{
   TinyGPSCustom **ppelt;

   for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
   {
      int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
      if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
         break;
   }

   pElt->next = *ppelt;
   *ppelt = pElt;
}


double TinyGPSPlus::iee754double(unsigned long raw1, unsigned long raw2)
{
	String a, b;
	char raw[65];
	int exponent = 0;
	double mantisse = 1.0;
	String raw1bin = String(raw1, BIN);
	String raw2bin = String(raw2, BIN);
	int shift1 = 32 - raw1bin.length();
	int shift2 = 32 - raw2bin.length();
	for (int x = 0; x<shift1; x++)
	{
		a += "0";
	}
	a += raw1bin;
	for (int x = 0; x<shift2; x++)
	{
		b += "0";
	}
	b += raw2bin;
	a += b;
	a.toCharArray(raw, 65);
	for (int x = 0; x<11; x++)
	{
		if (raw[x + 1] == 49)
		{
			exponent += pow(2.0, 10 - x);
		}
	}
	exponent = exponent - 1023;
	for (int x = 12; x < 62; x++)
	{
		if (raw[x] == 49)
		{
			mantisse += pow(2.0, 11 - x);
		}
	}
	double decFloat = mantisse*pow(2.0, exponent);
	if (raw[0] == 49)return -decFloat;
	else return decFloat;
}

