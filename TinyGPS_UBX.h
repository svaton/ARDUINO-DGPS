/*
TinyGPS_UBX is modified library based on the TinyGPS++. The library add features 
to parse some UBX sentence such as SBAS, RAW and some AID. The library was 
developed to support the low cost DGPS-Arduino project.

TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
Modified by Martin Svaton 2016
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __TinyGPSPlus_h
#define __TinyGPSPlus_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <limits.h>
#define _GPS_VERSION "0.92" // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
#define _GPS_FEET_PER_METER 3.2808399
#define _GPS_MAX_FIELD_SIZE 15

struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;
public:
   RawDegrees() : deg(0), billionths(0), negative(false)
   {}
};

struct TinyGPSLocation
{
   friend class TinyGPSPlus;
public:
	bool ubxllh() const { return posllh; }
	bool ubxecef() const { return posecef; }
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const RawDegrees &rawLat()     { updated = false; return rawLatData; }
   const RawDegrees &rawLng()     { updated = false; return rawLngData; }
   double lat();
   double lng();
   double ecefX() { return eX; }
   double ecefY() { return eY; }
   double ecefZ() { return eZ; }
   TinyGPSLocation() : valid(false), updated(false)
   {}

private:
   bool valid, updated, posllh, posecef;
   RawDegrees rawLatData, rawLngData, rawNewLatData, rawNewLngData;
   uint32_t lastCommitTime;
   void commit();
   void setLatitude(const char *term);
   void setLongitude(const char *term);
   double eX, eY, eZ, ubxLat, ubxLng;
};

struct TinyGPSDate
{
   friend class TinyGPSPlus;
public:
   bool isValid() const       { return valid; }
   bool isUpdated() const     { return updated; }
   uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   uint32_t value()           { updated = false; return date; }
   uint16_t year();
   uint8_t month();
   uint8_t day();

   TinyGPSDate() : valid(false), updated(false), date(0)
   {}

private:
   bool valid, updated;
   uint32_t date, newDate;
   uint32_t lastCommitTime;
   void commit();
   void setDate(const char *term);
};


struct TinyGPS_SBAS
{
	friend class TinyGPSPlus;
public:
	double iono(int i){ return SBASiono[i-1]; }
	double fast(int i){ return SBASfast[i-1]; }
	int status(int i){ return SBAS[i]; }
	bool valid(int i) { return SBASvalid[i - 1]; }
	bool SBASavailable() { return SBASavail; }
	
private:
	int SBAS[5];
	double SBASiono[32];
	double SBASfast[32];
	bool SBASvalid[32],SBASavail = false;
};

struct TinyGPS_RAW
{
	friend class TinyGPSPlus;
public:
	double pRange(int i) { return Prange[i-1]; }
	unsigned long raw(int i) { return RAW[i]; }
private:
	unsigned long RAW[4];
	double Prange[32];
};

struct TinyGPS_EPH
{
	friend class TinyGPSPlus;
public:
	double Toc(int i) { return toc[i-1]; }
	double M_0(int i) { return m_0[i-1]; }
	double Ecc(int i) { return ecc[i-1]; }
	double Dn(int i) { return dn[i-1]; }
	double SqrtA(int i) { return sqrtA[i-1]; }
	double Omega(int i) { return omega[i-1]; }
	double I_0(int i) { return i_0[i-1]; }
	double Om_0(int i) { return om_0[i-1]; }
	double Om_dot(int i) { return om_dot[i-1]; }
	double Idot(int x) { return i[x-1]; }
	double Cuc(int i) { return cuc[i-1]; }
	double Cus(int i) { return cus[i-1]; }
	double Cis(int i) { return cis[i-1]; }
	double Cic(int i) { return cic[i-1]; }
	double Crs(int i) { return crs[i-1]; }
	double Crc(int i) { return crc[i-1]; }
	double Af0(int i) { return a0[i-1]; }
	double Af1(int i) { return a1[i-1]; }
	double Af2(int i) { return a2[i-1]; }
	double Toe(int i) { return toe[i-1]; }
	bool valid(int i) { return avail[i - 1]; }
	bool available() { return on; }

private:
	double toc[33];
	double m_0[33];
	double ecc[33];
	double dn[33];
	double sqrtA[33];
	double omega[33];
	double i_0[33];
	double om_0[33];
	double om_dot[33];
	double i[33];
	double cuc[33];
	double cus[33];
	double cis[33];
	double cic[33];
	double crs[33];
	double crc[33];
	double a1[33];
	double a2[33];
	double a0[33];
	double toe[33];
	bool avail[33];
	bool on = false;
		
	

};

struct TinyGPS_SV
{
	friend class TinyGPSPlus;
public:
	int Az(int i){ return satelliteAz[i-1]; }
	int El(int i){ return satelliteEl[i-1]; }
	int viewSat(int i){ return satellitesInView[i]; }
	int totalSat(){ return totSatinView; }
	int totalSatUsed() { return totSatUsed; }
	bool SVINFOavailable() { return SVINFO; }
	int satUsedList(int i) { return sat[i]; }
	double satRes(int i) { return res[i - 1]; }

	
private:
	int totSatinView;
	int totSatUsed;
	int sentenceN;
	bool SVINFO = false;
	int satellitesInView[25];
	int satelliteAz[33];
	int satelliteEl[33];
	int i, satN;
	int sat[15];
	double res[33];
	
};

struct TinyGPSTime
{
   friend class TinyGPSPlus;
public:
	bool isFinished() {
		if (end) { end = false; return true; }
		else return false;
	}
   bool isValid() const       { return valid; }
   bool isUpdated() const     { return updated; }
   uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   uint32_t value()           { updated = false; return time; }
   uint8_t hour();
   uint8_t minute();
   uint8_t second();
   uint8_t centisecond();
   int week() { return w; }
   double tow() { return ToW; }
   double rcBias() { return rcB; }
   bool CLOCKavailable() { return CLOCKavail; }
   bool TIMEGPSavailable() { return TIMEGPSavail; }
   

   TinyGPSTime() : valid(false), updated(false), time(0)
   {}

private:
	bool valid, updated, end ;
   uint32_t time, newTime;
   uint32_t lastCommitTime;
   void commit();
   void setTime(const char *term);
   int w; 
   double ToW;
   double ftoW;
   double rcB;
   bool CLOCKavail = false, TIMEGPSavail = false;
};

struct TinyGPSDecimal
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   int32_t value()         { updated = false; return val; }

   TinyGPSDecimal() : valid(false), updated(false), val(0)
   {}

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   int32_t val, newval;
   void commit();
   void set(const char *term);
};

struct TinyGPSInteger
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   uint32_t value()        { updated = false; return val; }
   uint32_t status() { return fix; }
   TinyGPSInteger() : valid(false), updated(false), val(0)
   {}

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   uint32_t val, newval, fix;
   void commit();
   void set(const char *term);
  
  
};

struct TinyGPSSpeed : TinyGPSDecimal
{
   double knots()    { return value() / 100.0; }
   double mph()      { return _GPS_MPH_PER_KNOT * value() / 100.0; }
   double mps()      { return _GPS_MPS_PER_KNOT * value() / 100.0; }
   double kmph()     { return _GPS_KMPH_PER_KNOT * value() / 100.0; }
};

struct TinyGPSCourse : public TinyGPSDecimal
{
   double deg()      { return value() / 100.0; }
};

struct TinyGPSAltitude : TinyGPSDecimal
{
	friend class TinyGPSPlus;
public:
   double meters()       { return value() / 100.0; }
   double miles()        { return _GPS_MILES_PER_METER * value() / 100.0; }
   double kilometers()   { return _GPS_KM_PER_METER * value() / 100.0; }
   double feet()         { return _GPS_FEET_PER_METER * value() / 100.0; }
   double geoid() { return geo; }
   double elipsoid() { return elips; }
private:
	double geo,elips;
};

class TinyGPSPlus;
class TinyGPSCustom
{
public:
   TinyGPSCustom() {};
   TinyGPSCustom(TinyGPSPlus &gps, const char *sentenceName, int termNumber);
   void begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber);

   bool isUpdated() const  { return updated; }
   bool isValid() const    { return valid; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const char *value()     { updated = false; return buffer; }

private:
   void commit();
   void set(const char *term);

   char stagingBuffer[_GPS_MAX_FIELD_SIZE + 1];
   char buffer[_GPS_MAX_FIELD_SIZE + 1];
   unsigned long lastCommitTime;
   bool valid, updated;
   const char *sentenceName;
   int termNumber;
   friend class TinyGPSPlus;
   TinyGPSCustom *next;
};

class TinyGPSPlus
{
public:
  TinyGPSPlus();
  bool encode(byte c); // process one character received from GPS
  TinyGPSPlus &operator << (char c) {encode(c); return *this;}

  TinyGPS_SV sv;
  TinyGPS_SBAS ubxSBAS;
  TinyGPS_RAW ubxRAW;
  TinyGPS_EPH ubxEPH;
  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSDecimal hdop;

  static const char *libraryVersion() { return _GPS_VERSION; }

  static double distanceBetween(double lat1, double long1, double lat2, double long2);
  static double courseTo(double lat1, double long1, double lat2, double long2);
  static const char *cardinal(double course);

  static int32_t parseDecimal(const char *term);
  static void parseDegrees(const char *term, RawDegrees &deg);

  uint32_t charsProcessed()   const { return encodedCharCount; }
  uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
  uint32_t failedChecksum()   const { return failedChecksumCount; }
  uint32_t passedChecksum()   const { return passedChecksumCount; }
  
private:
	enum { GPS_SENTENCE_GPGGA, GPS_SENTENCE_GPRMC, GPS_SENTENCE_OTHER };

  // parsing state variables
  uint8_t parity;
  bool isChecksumTerm;
  char term[_GPS_MAX_FIELD_SIZE];
  uint8_t curSentenceType;
  uint8_t curTermNumber;
  uint8_t curTermOffset;
  bool sentenceHasFix;

  // custom element support
  friend class TinyGPSCustom;
  TinyGPSCustom *customElts;
  TinyGPSCustom *customCandidates;
  void insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int index);

  // statistics
  uint32_t encodedCharCount;
  uint32_t sentencesWithFixCount;
  uint32_t failedChecksumCount;
  uint32_t passedChecksumCount;

  // internal utilities
  int fromHex(char a);
  bool endOfTermHandler();
  void parseSBAS();
  void parseRAW();
  void parseEPH(int sv);
  void parseINI();
  void parseSVINFO();
  void parseTIMEGPS();
  void parsePOSLLH();
  void parsePOSECEF();
  double iee754double(unsigned long raw1, unsigned long raw2);
  int n=0,m=0;
 
  int UBXflag = 0, counter, UBXlength = 15;
  uint8_t CK_A = 0;
  uint8_t CK_B = 0;
  bool UBXckError;
  byte data[400];
  const double Pi = 3.1415926535;
};

#endif // def(__TinyGPSPlus_h)
