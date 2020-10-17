#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

struct loc
{
  float lat;
  float lng;
};

loc locHistory{
  lat: 0,
  lng: 0,
};

// unsigned long metersSinceLast = 0;

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
 
void scan() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
};

int getGpsData() {
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

    if (metersSinceLast > 10) {
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

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }

  // int 

  return gps.satellites.isValid() ? gps.satellites.value() : 0;
};

void setupDisplay() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(100);
};

void renderInfoOnDisplay(int satellites) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setCursor(0,0);             // Start at top-left corner
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.print("GPS");
  display.setTextColor(SSD1306_WHITE);
  display.print(": ");
  display.println(satellites);

  // display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  // display.print("M since last");
  // display.setTextColor(SSD1306_WHITE);
  // display.print(": ");
  // display.println(metersSinceLast);

  display.println("meters:");
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(distance.m);

  display.setTextSize(1);
  display.println("kilometers:");
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(distance.km, 3);

  display.display();
};

void renderDisplayIntro() {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
};

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  scan();
  setupDisplay();
  ss.begin(GPSBaud);
  renderDisplayIntro();
}

void loop()
{
  int satellites = getGpsData();
  renderInfoOnDisplay(satellites);
}