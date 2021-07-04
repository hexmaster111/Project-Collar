
#include "Arduino.h"
#include <NMEAGPS.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Wire.h>
#include "config.h"

double currentSpeed; //Speed

// these double values will hold ourGPS data
double lat;
double lon;
double ele;

// Speed we should update the gps information
// every IO_LOOP_DELAY milliseconds
#define IO_LOOP_DELAY 30000
unsigned long lastUpdate;

#define DEV_BUILD

//Pin to light up the leds
#define LED_PIN 5

// Assined ID
#define DRONE_ID 5050

#define GPSport_h

#ifdef DEV_BUILD
#define DEBUG_PORT Serial
#endif

#define gpsPort GPS_Serial //was GPS_Serial(1)
#define GPS_PORT_NAME "GPS_Serial 1"

//Only needed if your board is irreragluler like mine >.<
#define LED_BUILTIN 23

//-6 for central time zone
#define timeZoneTimeCorrection 6

//Internal Pin for the bat voltage
#define VBAT_PIN 35

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

// Uncomment one DST changeover rule, or define your own:
#define USA_DST
//#define EU_DST

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

//outputs from io
AdafruitIO_Feed *vibration = io.feed("vibration");
AdafruitIO_Feed *shockSetLevel = io.feed("shockLevel");
AdafruitIO_Feed *doShock = io.feed("sendShock");
AdafruitIO_Feed *setVision = io.feed("vision");

//Vibration Handler
void handleMessageVibration(AdafruitIO_Data *data)
{

    // convert the data to integer
    int reading = data->toInt();

    Serial.print("received: Vibration <- ");
    Serial.println(reading);
}

void handleMessageShockSetLevel(AdafruitIO_Data *data)
{

    // convert the data to integer
    int reading = data->toInt();

    Serial.print("received: Shock Level Change <- ");
    Serial.println(reading);
}

void handleMessageDoShock(AdafruitIO_Data *data)
{
    int reading = data->toInt();

    Serial.print("received: Do Shock <- ");
    Serial.println(reading);
}

void handleMessageSetVision(AdafruitIO_Data *data)
{

    // convert the data to integer
    int reading = data->toInt();

    Serial.print("received: Vision Level Change <- ");
    Serial.println(reading);
}

void setup()
{
    // Handle some sick io
    pinMode(LED_BUILTIN, OUTPUT); //Dev Stuff
    pinMode(VBAT_PIN, INPUT);     //Used for the battery voltage pin

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
    // location->get();
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
    // save value and location data to the
    // 'location' feed on Adafruit IO
    location->save(currentSpeed, lat, lon, ele);
    lastSavedLat = lat;
    lastSavedLon = lon;
    lastUpdate = millis();
}

void loop()
{

    io.run();
    if (millis() > (lastUpdate + IO_LOOP_DELAY)) // slow to save power and data
    {
        gpsLoop();
    }
}
