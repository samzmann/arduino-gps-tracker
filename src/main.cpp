#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

struct loc
{
  float lat;
  float lng;
};

loc locHistory{
  lat: 0,
  lng: 0,
};

int historyLength = 0;

struct distStruct {
  unsigned long m;
  unsigned long km;
};

distStruct distance = {
  m: 0,
  km: 0,
};



/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  smartDelay(0);
}

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop()
{

  Serial.println();

  // printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  // printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.println();
  // printInt(gps.location.age(), gps.location.isValid(), 5);
  // printDateTime(gps.date, gps.time);
  // printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  // printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);

  Serial.print("GPS: ");
  Serial.println(gps.satellites.value());

  Serial.print("location valid: ");
  Serial.println(gps.location.isValid());

  if (gps.location.isValid()) {
    loc newLoc = {
      lat: gps.location.lat(),
      lng: gps.location.lng(),
    };

    unsigned long metersSinceLast = TinyGPSPlus::distanceBetween(
      newLoc.lat,
      newLoc.lng,
      locHistory.lat, 
      locHistory.lng);

      Serial.print("locHistory: ");
      Serial.print(locHistory.lat);
      Serial.print(", ");
      Serial.println(locHistory.lng);

      Serial.print("metersSinceLast: ");
      Serial.println(metersSinceLast);

    if (metersSinceLast > 2) {
      // locHistory is init with lat/lng = 0,
      // so we skip the first history to avoid
      // a false distance between (0,0) and current (lat,lng)
      if (historyLength > 0) {
        distance.m = distance.m + metersSinceLast;
        distance.km = distance.km + (metersSinceLast / 1000);
      };

      historyLength = historyLength + 1;
      locHistory = newLoc;

      Serial.println();
      Serial.print("meters: ");
      Serial.print(distance.m);
      Serial.println();
      Serial.print("km: ");
      Serial.print(distance.m);
    
    }

  };
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}