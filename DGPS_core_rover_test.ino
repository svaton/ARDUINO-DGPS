#include <TinyGPSPlus.h>
#include <AltSoftSerial.h>

byte header[]={0xB5,0x62,0xCC,0x01};
int satLength=5, sats[]={23,13,16,9,5};
//int satLength=7, sats[]={2,30,5,28,7,16,9};
//int satLength=8, sats[]={30,28,7,8,13,18,20,15};
long tow = 15234500, LAT=541348765, LNG=151348765, ALT=22500; //x lat
unsigned long t1;
byte buf[60], window[50];
int UBXflag, counter,charC, UBXlength ;
byte CK_A, CK_B;
bool WifiEstablished = false;
String Str;
#define LED 2
#define RESET 3 
TinyGPSPlus gps;
AltSoftSerial altSerial;

void setup()
{
  Serial.begin(57600);
  altSerial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET,LOW);
  delay(100);
  digitalWrite(RESET,HIGH);
  
}

void loop()
{
if (Serial.available() > 0)gps.encode(Serial.read());
if (altSerial.available() >0)wifiRSencode(altSerial.read()&0xff);
if (WifiEstablished)
{
  programSend();
  ledBlink(0);
 
}
else digitalWrite(LED,LOW);
}


void wifiRSencode(byte b)
{
 if (UBXflag < 2)
 {
    switch (b)
    {
    case 0xB5:
    {
      UBXflag = 1;
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
        UBXlength = 8;
      }
      else UBXflag = 0;
      break;
    }
    }
  }
  else if (UBXflag == 2)
  {
    window[counter] = b;

    if (counter <= UBXlength && window[0] == 0xCC && window[1] == 0xFE)
    {
      if (counter >= 2)
      {
      Serial.print(F("RS ID"));
      Serial.print(window[2],HEX);
      Serial.println(F(" recognized"));
      WifiEstablished = true;
      digitalWrite(LED,HIGH);
      UBXflag = 0;
      }
    }
    else if (counter <= UBXlength && window[0] == 0xCC && window[1] == 0xFF)
    {
      if (counter >= 2)
      {
      Serial.println(F("RS connection pending"));
      UBXflag = 0;
      }
    }

    else if (counter > 2)UBXflag = 0;
    counter++;
  }

}
void ledBlink(unsigned long t)
{
  if (t>0)t1 = t;
  if (millis() - t1 <= 50 || millis() - t1 >= 5000)digitalWrite(LED,HIGH);
  else digitalWrite(LED,LOW);
  
}
void programSend()
{
 if (gps.time.isFinished())//(gps.satellites.isUpdated() && gps.location.isValid()) gps.time.value()/100 - t > 0 && 
 {
  ledBlink(millis());
   satLength=gps.sv.totalSatUsed();
    LAT = gps.location.latDeg()*10000000 + gps.location.lat();
    LNG = gps.location.lngDeg()*10000000 + gps.location.lng();
    
    ALT = long(gps.altitude.meters()*100 + gps.altitude.geo()*100);
  Serial.print(gps.location.latDeg());
  Serial.print(".");
  Serial.println(gps.location.lat()); 
  Serial.print(gps.location.lngDeg());
  Serial.print(".");
  Serial.println(gps.location.lng());
  Serial.println(LAT);
  Serial.println(LNG);
  Serial.println(gps.location.ecefX());
  Serial.println(gps.location.ecefY());
  Serial.println(gps.location.ecefZ());
  Serial.println(satLength);
  Serial.println(ALT);
  Serial.println(gps.time.tow());
   Serial.println(gps.speed.kmph());
  Serial.println(gps.course.deg());
 
  for(int i = 0; i < gps.sv.totalSatUsed();i++)
  {
    Serial.print(gps.sv.satUsedList(i));
    Serial.print(",");  
    Serial.print(gps.sv.satRes(gps.sv.satUsedList(i)));
    Serial.println(",");  

  }
  Serial.println();
  // build HEX bytes from int
union {
    long integer;
    char d[4];
} temp32bitint;
union {
    int integer2;
    char c[2];
} temp16bitint;
for (int i=0;i<4;i++)
 {
  buf[i]=header[i];
 }
buf[4] = satLength;
buf[5] = 0;
temp32bitint.integer = long(gps.time.tow());
buf[6] = temp32bitint.d[3];
buf[7] = temp32bitint.d[2];
buf[8] = temp32bitint.d[1];
buf[9] = temp32bitint.d[0];
temp32bitint.integer = long(gps.speed.kmph()*100);
buf[10] = temp32bitint.d[3];
buf[11] = temp32bitint.d[2];
buf[12] = temp32bitint.d[1];
buf[13] = temp32bitint.d[0];
temp32bitint.integer = long(gps.course.deg());
buf[14] = temp32bitint.d[3];
buf[15] = temp32bitint.d[2];
buf[16] = temp32bitint.d[1];
buf[17] = temp32bitint.d[0];
temp32bitint.integer = gps.location.ecefX();
buf[18] = temp32bitint.d[3];
buf[19] = temp32bitint.d[2];
buf[20] = temp32bitint.d[1];
buf[21] = temp32bitint.d[0];
temp32bitint.integer = gps.location.ecefY();
buf[22] = temp32bitint.d[3];
buf[23] = temp32bitint.d[2];
buf[24] = temp32bitint.d[1];
buf[25] = temp32bitint.d[0];
temp32bitint.integer = gps.location.ecefZ();
buf[26] = temp32bitint.d[3];
buf[27] = temp32bitint.d[2];
buf[28] = temp32bitint.d[1];
buf[29] = temp32bitint.d[0];
 for (int i=0;i<satLength;i++)
 {
  buf[30+i]=gps.sv.satUsedList(i);
 }
 for (int i=0;i<satLength;i++)
 {
  temp16bitint.integer2 = gps.sv.satRes(gps.sv.satUsedList(i))*100;
 buf[30 + satLength + (i*2)] = temp16bitint.c[0];
 buf[30 + satLength + (i*2)+1] = temp16bitint.c[1];
 }
  buf[30 + satLength + ((satLength-1)*2)+2] = 0x00;
 byte CK_A = 0, CK_B = 0 ;// checksum declaration including header
  for (int i=0;i<=30 + 3*satLength;i++)
 {
 //checksum
 if(i > 1)
 {
   CK_A += buf[i];
   CK_B += CK_A;
   CK_A &= 0xFF;
   CK_B &= 0xFF;
 } 
    Str += ",";
    if (buf[i] < 0x10)Str += "0";
    Str += String(buf[i],HEX);  
    
    
 }
 
   Str += ",";
   if (CK_A < 0x10)Str += "0";
   Str += String(CK_A,HEX);  
   Str += ",";
   if (CK_B < 0x10)Str += "0";
   Str += String(CK_B,HEX); 
    
    Serial.println(Str);
    altSerial.print(Str);
    altSerial.print("*");
    Str = "";
/*for (int i=0;i<=22 + 3*satLength;i++)
 {
    Serial.print(buf[i],HEX);
    Serial.print(" "); 
 }
 Serial.print(CK_A,HEX); 
  Serial.print(" ");
    Serial.println(CK_B,HEX);
 
 */
 }
}
