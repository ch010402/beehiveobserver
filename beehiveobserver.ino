/*
 * beehiveobserver.ino
 * 
 * ToDo:
 *  Add calance
 *  send balance sensor data
 * 
 * Description: 
 *  Phase I sends GPS position data over LoraWAN to TTN
 *  Phase II implement Temp sensor
 *  Pahse III implement balance
 * 
 * License:
 *  Copyright (c) 2020 Christoph Latzer
 *  MIT License
 *  
 * Credits:
 *  Gonzalo Casas - mkrwan_03_gps_tracker.ino - GPS Tracker example code
 *  slash-dev - NeoGPS stuff
 *  ukhas.org.uk - ublox psm 
 */


//=====includes=====================
// include MKR WAN Stuff such as LORA
#include <MKRWAN.h>
#include "arduino_secrets.h"

// include GPS 
#include <NMEAGPS.h>
//#include "GPSfix.h"
// The GPSport.h include file tries to choose a default serial port for the GPS device.
#include <GPSport.h>
// For the NeoGPS example programs, "Streamers" is common set of printing and formatting routines for GPS data, in a Comma-Separated Values text format (aka CSV).
//#include <Streamers.h>

// include temperatur sensor DS18B20
#include <DS18B20.h>

//=====definitions=====================
#define debugSerial Serial
#define gpsSerial Serial1
#define loraSerial Serial2

//// LoRaWAN
// Select your region (AS923, AU915, EU868, KR920, IN865, US915, US915_HYBRID)
_lora_band region = EU868;
LoRaModem modem(loraSerial);

//// GPS
// This object parses received characters into the gps.fix() data structure
static NMEAGPS  gps;
// Define a set of GPS fix information.
static gps_fix  fix;
const int posarraysize = 4;
int pos[posarraysize];
// Set GPS to backup mode (sets it to never wake up on its own)
uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
// Restart GPS from backup mode
uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
// Shuts GPS reciever down 
uint8_t GPSdown[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74};
// Wakes GPS receiver up
uint8_t GPSup[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76};
bool enable_gps = true;

//// TTN Mapper
unsigned long last_update = 0;
uint8_t txBuffer[12];
uint32_t latitudeBinary, longitudeBinary;
uint16_t altitudeGps;
uint8_t satsGps;

// Flash
#define VERY_FAST 50  //error
#define FAST 200      //waiting/working
#define SLOW 500      //ok
#define FOREVER -1

int count = 10;

// DS18B20
DS18B20 ds(7);
uint8_t selected;
bool enable_temp = true;
float temperatur = 123;


//=====functions=====================
// let the led flash
void flash(int times, unsigned int speed) {
  while (times == -1 || times--) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(speed);
    digitalWrite(LED_BUILTIN, LOW);
    delay(speed);
  }
}

// get the GPS position
void gpspos(int gpswait) {
  // set exit Variable false 
  bool exit = false;
  // clear array
  for ( int i = 0; i < sizeof(pos); i++ ) {
    pos[i] = 0;
  }
  // wake up GPS
  debugSerial.println(F("Wake GPS up from sleep..."));
  sendUBX(GPSup, sizeof(GPSup)/sizeof(uint8_t));
  // give GPS time [sec] to aquire satellites almanach and eph
  delay(1000 * gpswait);
  flash(2, FAST);
  
  while ( exit == false ) {
    while (gps.available( gpsSerial ) > 0) {
      fix = gps.read();
      pos[0] = fix.latitudeL();
      pos[1] = fix.longitudeL();
      pos[2] = fix.altitude_cm();
      pos[3] = fix.satellites;
      if ( fix.valid.location && fix.valid.altitude) {
        exit = true;
      }
    }
  }
  debugSerial.println(F("Send GPS to sleep..."));
  sendUBX(GPSdown, sizeof(GPSdown)/sizeof(uint8_t));
}

// send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gpsSerial.write(MSG[i]);
  }
  flash(1, SLOW);
}

// send a package over LoRa
void sendLoRa() {
  //build the package to send
  buildPacket();

  // send Data
  modem.beginPacket();
  modem.write(txBuffer, sizeof(txBuffer));
  
  // error handling
  int err = modem.endPacket(false);
  if (err > 0) {
    debugSerial.println("Data sent: ");
//    debugSerial.print(txBuffer);
    flash(2, FAST);
  }
  else {
    debugSerial.println("Error sending data");
    flash(10, VERY_FAST);
  }
}

// build the package
void buildPacket() {
  // reduce the numbers
  int shortlat = pos[0] / 100;
  int shortlon = pos[1] / 100;
  int alt_m = pos[2] / 100;
  int temp = ( temperatur + 50 ) * 10;
  //device ID
  txBuffer[0] = device[0];
  // latitude
  txBuffer[1] = ( shortlat >> 16 ) & 0xFF;
  txBuffer[2] = ( shortlat >> 8 ) & 0xFF;
  txBuffer[3] = shortlat & 0xFF;
  // longitude
  txBuffer[4] = ( shortlon >> 16 ) & 0xFF;
  txBuffer[5] = ( shortlon >> 8 ) & 0xFF;
  txBuffer[6] = shortlon & 0xFF;
  // altitude
  txBuffer[7] = ( alt_m >> 8 ) & 0xFF;
  txBuffer[8] =  alt_m & 0xFF;
  // satellites
  txBuffer[9] = pos[3] & 0xFF;
  // temperatur
  txBuffer[10] = (temp >> 8 ) & 0xFF;
  txBuffer[11] = temp & 0xFF;
}

void gettemp() {
  temperatur = ds.getTempC();
}


//=====setup=====================
void setup() {
// GPS
  gpsSerial.begin( 9600 );

// Visual feedback Startup
  pinMode(LED_BUILTIN, OUTPUT);
  flash(10, VERY_FAST);

// DebugSerial
  debugSerial.begin( 9600 );
  // Wait a maximum of 5s for Serial Monitor serial
  while (!debugSerial && millis() < 5000);
  debugSerial.println(F("Starting up..."));
  flash(2, FAST);

// Begin LoRa modem
  if (!modem.begin(region)) {
    debugSerial.println(F("Failed to start module"));
    flash(FOREVER, VERY_FAST);
  };

  debugSerial.print("Your module version is: ");
  debugSerial.println(modem.version());
  debugSerial.print(F("Your device EUI is: "));
  debugSerial.println(modem.deviceEUI());
  flash(2, FAST);

  int connected = modem.joinOTAA(appEui, appKey);
  while (!connected) {
    // retry
    flash(10, VERY_FAST);
    debugSerial.println(F("Retry connection"));
    connected = modem.joinOTAA(appEui, appKey);
    /*if (!connected) {
      debugSerial.println(F("Something went wrong; are you indoor? Move near a window and retry"));
      flash(FOREVER, VERY_FAST);
    }*/
  }
  debugSerial.println(F("Successfully joined the network!"));
  flash(2, FAST);

  debugSerial.println(F("Enabling ADR and setting low spreading factor"));
  modem.setADR(true);
  modem.dataRate(5);
  flash(2, FAST);

//  DS18B20
  selected = ds.select(address);
  while (selected && millis() < 5000);
  if (!selected) {
    debugSerial.println(F("Device DS18B20 not found!"));
    flash(FOREVER, VERY_FAST);
  }
  else { flash(2, SLOW); }
// Output so Serial
  debugSerial.println(F("Latitude,Longitude,Heigh,Satellites,Temperatur"));

// enable / disable Module
  enable_gps = true;
  enable_temp = true;
}

//=====main=====================

void loop() {
  if (enable_gps) {
    if (count >= 10 ) {
      debugSerial.println(F("GPS full power"));
      gpspos(72); // Time in secons GPS is fully powered to receive data
      count = 1;
    }
    else {
      gpspos(10);
      count++;
    }
  }
  if (enable_temp) {
    gettemp();
  }
  
  sendLoRa();

  for (int i=0; i< sizeof(pos); i++) {
     debugSerial.print(pos[i]);
     debugSerial.print(",");
  }
  debugSerial.print(temperatur);
  debugSerial.print("\n");

  for (int i=0; i< sizeof(txBuffer); i++) {
     debugSerial.print(txBuffer[i]);
     debugSerial.print(" ");
  }
  debugSerial.print("\n");
  
  delay(1000 * 60 * 10);
}