//Project collar writed by 5050 
//released  GPLV3 

// 1. Anyone can copy, modify and distribute this software.
// 2. You have to include the license and copyright notice with each and every distribution.
// 3. You can use this software privately.
// 4. You can use this software for commercial purposes.
// 5. If you dare build your business solely from this code, you risk open-sourcing the whole code base.
// 6. If you modify it, you have to indicate changes made to the code.
// 7. Any modifications of this code base MUST be distributed with the same license, GPLv3.
// 8. This software is provided without warranty.
// 9. The software author or license can not be held liable for any damages inflicted by the software.


#include "Arduino.h"
#include <NMEAGPS.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include <analogWrite.h>

double currentSpeed; //Speed

// these double values will hold ourGPS data
double lat;
double lon;
double ele;

// Speed we should update the gps information
// every IO_LOOP_DELAY milliseconds
#define IO_FAST_LOOP_DELAY 250
#define IO_SLOW_LOOP_DELAY 120000

#define GPSport_h

#ifdef DEV_BUILD
#define DEBUG_PORT Serial
#endif

#define gpsPort GPS_Serial //was GPS_Serial(1)
#define GPS_PORT_NAME "GPS_Serial 1"

//-6 for central time zone
#define timeZoneTimeCorrection 6

//Internal Pin for the bat voltage
#define VBAT_PIN 35

//For the perotic Timers
unsigned long lastUpdateFast, lastUpdateSlow;

//Var used to store battery voltage
float vBatt;

float lastSavedLat, lastSavedLon, lastSavedAlt, lastSavedSpeed;

NMEAGPS gps;
gps_fix fix;

HardwareSerial GPS_Serial(1); // use UART #1

// Set these values to the offset of your timezone from GMT

static const int32_t zone_hours = -timeZoneTimeCorrection; // EST was -6L
static const int32_t zone_minutes = 0L;                    // usually zero
static const NeoGPS::clock_t zone_offset =
    zone_hours * NeoGPS::SECONDS_PER_HOUR +
    zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

#if defined(USA_DST)
static const uint8_t springMonth = 3;
static const uint8_t springDate = 14; // latest 2nd Sunday
static const uint8_t springHour = 2;
static const uint8_t fallMonth = 11;
static const uint8_t fallDate = 7; // latest 1st Sunday
static const uint8_t fallHour = 2;
#define CALCULATE_DST

#elif defined(EU_DST)
static const uint8_t springMonth = 3;
static const uint8_t springDate = 31; // latest last Sunday
static const uint8_t springHour = 1;
static const uint8_t fallMonth = 10;
static const uint8_t fallDate = 31; // latest last Sunday
static const uint8_t fallHour = 1;
#define CALCULATE_DST
#endif

// Setting up our feeds to send

//inputs to io
AdafruitIO_Feed *location = io.feed("location");
AdafruitIO_Feed *battery = io.feed("batteryLevel");
AdafruitIO_Feed *all_log = io.feed("log");

//outputs from io
AdafruitIO_Feed *vibration = io.feed("vibration");
AdafruitIO_Feed *shockSetLevel = io.feed("shockLevel");
AdafruitIO_Feed *doShock = io.feed("sendShock");
AdafruitIO_Feed *setVision = io.feed("vision");

////// MESSAGE HANDLERS ///////////
int vibrationLevel, shockLevel, visionLevel; //Vars to pull from online
bool sendShock;

//Holds the time the shock started in ms
unsigned long shockStartTime;

void handleMessageVibration(AdafruitIO_Data *data)
{

    // convert the data to integer
    int reading = data->toInt();
    vibrationLevel = reading;

    analogWrite(LED_BUILTIN, map(reading, 0, 100, 0, 255)); //for levels

    //analogWrite(VIBRATION_PIN, reading); //for direct control

    Serial.print("received: Vibration <- ");
    Serial.println(reading);
}

void handleMessageShockSetLevel(AdafruitIO_Data *data)
{

    // convert the data to integer
    int reading = data->toInt();
    shockLevel = reading;
    if (shockLevel == 0)
    {
        digitalWrite(SHOCK_PIN, false); //shut off the shock if the slizer goes to zero
    }
    Serial.print("received: Shock Level Change <- ");
    Serial.println(reading);
}

void handleMessageDoShock(AdafruitIO_Data *data)
{
    int reading = data->toBool(); //changed from toint to tobool
    sendShock = reading;
    if (reading == 0)
    { //dont care what happonds if the reading is a zero
        return;
    }

    //check if we waited long enough and that shck level isnt zero
    if (millis() > (shockStartTime + SHOCK_WAIT_TIME) && (shockLevel != 0))
    {
        all_log->save(String("Drone#") + String(DRONE_ID) + String(":") + String("SHOCK:STARTED"));

        digitalWrite(SHOCK_PIN, true);

        shockStartTime = millis();
    }
    else if (digitalRead(SHOCK_PIN))
    {
        all_log->save(String("Drone#") + String(DRONE_ID) + String(":") + String("SHOCK:ACTIVE"));
    }
    else
    {
        all_log->save(String("Drone#") + String(DRONE_ID) + String(":") + String("SHOCK:ERROR"));
    }
}

void handleMessageSetVision(AdafruitIO_Data *data)
{

    // convert the data to integer
    int reading = data->toInt();
    visionLevel = reading;
    Serial.print("received: Vision Level Change <- ");
    Serial.println(reading);
}

////// END MESSAGE HANDLERS ///////////

void setup()
{
    // Handle some sick io
    pinMode(LED_BUILTIN, OUTPUT); //Dev Stuff
    pinMode(VIBRATION_PIN, OUTPUT);
    pinMode(SHOCK_PIN, OUTPUT);

    pinMode(VBAT_PIN, INPUT); //Used for the battery voltage pin

    analogWriteResolution(LED_BUILTIN, 12);

    // start the serial connection
    DEBUG_PORT.begin(115200);

    GPS_Serial.begin(9600, SERIAL_8N1, GPIO_NUM_34, GPIO_NUM_35); //Start the GPS serial

    // start the serial connection

    // wait for serial monitor to open
    while (!Serial)
        ;

    Serial.print("Connecting to Adafruit IO");

    // connect to io.adafruit.com
    io.connect();
    //Setup handlers
    vibration->onMessage(handleMessageVibration);
    shockSetLevel->onMessage(handleMessageShockSetLevel);
    doShock->onMessage(handleMessageDoShock);
    setVision->onMessage(handleMessageSetVision);
    // wait for a connection
    while (io.status() < AIO_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }

    // we are connected
    Serial.println();
    Serial.println(io.statusText());
    // This dosent seem to be working as intended
    //get the current data from the dashboard
    setVision->get();
    all_log->save(String("Drone#") + String(DRONE_ID) + String(":") + String("ONLINE"));
}

void adjustTime(NeoGPS::time_t &dt) //We only use this for the display (24 hr time hard)
{
    NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds

#ifdef CALCULATE_DST
        //  Calculate DST changeover times once per reset and year!
    static NeoGPS::time_t changeover;
    static NeoGPS::clock_t springForward, fallBack;

    if ((springForward == 0) || (changeover.year != dt.year))
    {

        //  Calculate the spring changeover time (seconds)
        changeover.year = dt.year;
        changeover.month = springMonth;
        changeover.date = springDate;
        changeover.hours = springHour;
        changeover.minutes = 0;
        changeover.seconds = 0;
        changeover.set_day();
        // Step back to a Sunday, if day != SUNDAY
        changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
        springForward = (NeoGPS::clock_t)changeover;

        //  Calculate the fall changeover time (seconds)
        changeover.month = fallMonth;
        changeover.date = fallDate;
        changeover.hours = fallHour - 1; // to account for the "apparent" DST +1
        changeover.set_day();
        // Step back to a Sunday, if day != SUNDAY
        changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
        fallBack = (NeoGPS::clock_t)changeover;
    }
#endif

    //  First, offset from UTC to the local timezone
    seconds += zone_offset;

#ifdef CALCULATE_DST
    //  Then add an hour if DST is in effect
    if ((springForward <= seconds) && (seconds < fallBack))
        seconds += NeoGPS::SECONDS_PER_HOUR;
#endif

    dt = seconds; // convert seconds back to a date/time structure
}

void gpsLoop()
{
    while (gps.available(GPS_Serial))
    {
        fix = gps.read();

        if (fix.valid.time && fix.valid.date)
        {
            adjustTime(fix.dateTime);
        }
    }

    lat = fix.latitude();
    lon = fix.longitude();
    ele = fix.altitude_ft();
    currentSpeed = fix.speed_mph();
    if (currentSpeed < 1)
    {
        currentSpeed = 0;
    }

    if (lat == 0)
    {
        Serial.println("LAT = 0");
        return;
    }

#ifdef DEV_BUILD
    Serial.println("----- sending -----");
    Serial.print("Speed: ");
    Serial.println(currentSpeed);
    Serial.print("lat: ");
    Serial.println(lat, 6);
    Serial.print("lon: ");
    Serial.println(lon, 6);
    Serial.print("ele: ");
    Serial.println(ele, 2);
#endif

    location->save(currentSpeed, lat, lon, ele);
    lastSavedLat = lat;
    lastSavedLon = lon;
}

void printVars() // Prints out the values receved to the console
{
    // int vibrationLevel, shockLevel, visionLevel; //Vars to pull from online
    // bool sendShock;
    Serial.println("-----IN-------");
    Serial.print("Shock Level = ");
    Serial.println(shockLevel);
    Serial.print("vibration level ");
    Serial.println(vibrationLevel);
    Serial.print("vision level ");
    Serial.println(visionLevel);
    Serial.print("Send shock = ");
    Serial.println(sendShock);
    Serial.println("----OUT------");
    Serial.print("vBatt = ");
    Serial.println(vBatt);
}

void updateBatteryInfo() // Will send battery info to dashboard
{

    //Save our battry level
    vBatt = (float)(analogRead(VBAT_PIN)) / 4095 * 2 * 3.3 * 1.1;

#ifdef BATT_ICON
    //code here to make battery icon
    //battery-0 Dead, battery-4 battery full
    //2.4 v is dead battory
    int icon = map(vBatt, 2.4, 4.2, 0, 4);
    if (vBatt < 2)
    {
        battery->save("warning");
    }
    else
    {
        switch (icon)
        {
        case 0:
            battery->save("battery-0");
            break;
        case 1:
            battery->save("battery-1");
            break;
        case 2:
            battery->save("battery-2");
            break;
        case 3:
            battery->save("battery-3");
            break;
        case 4:
            battery->save("battery-4");
            break;
        default:
            battery->save("warning");
            break;
        }
    }
    Serial.println("sent: vBatt -> Status");
    Serial.print("Batt Voltage:");
    Serial.println(vBatt);

#endif
#ifndef BATT_ICON
    battery->save(vBatt);
    Serial.print("sent: vBatt -> ");
    Serial.println(vBatt);
#endif
}

void peroticLoopFast() //here is things that need to run quicker
{
    if (millis() > (lastUpdateFast + IO_FAST_LOOP_DELAY))
    {
        // Shock Reset handler
        if (millis() > (shockStartTime + shockLevel * 500) && digitalRead(SHOCK_PIN))
        {
            digitalWrite(SHOCK_PIN, false);
        }

        lastUpdateFast = millis();
    }
}

void peroticLoopSlow() //Here will go things that need to every so offten
{
    if (millis() > (lastUpdateSlow + IO_SLOW_LOOP_DELAY))
    {
        updateBatteryInfo();
        gpsLoop();
        //last thing to do
        lastUpdateSlow = millis();
    }
}

void loop()
{
    io.run(); //handle adafruit io connection stuff
    peroticLoopFast();
    peroticLoopSlow();
}
