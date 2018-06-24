#include <IridiumSBD.h>
#include <SD.h>
#include <TinyGPS++.h>
#include "wiring_private.h"

/*
   BasicSend

   This sketch sends a "Hello, world!" message from the satellite modem.
   If you have activated your account and have credits, this message
   should arrive at the endpoints you have configured (email address or
   HTTP POST).

   Assumptions

   The sketch assumes an Arduino Mega or other Arduino-like device with
   multiple HardwareSerial ports.  It assumes the satellite modem is
   connected to Serial3.  Change this as needed.  SoftwareSerial on an Uno
   works fine as well.
*/

#define IridiumSerial Serial1
#define GpsSerial Serial2 //Serial2 // TODO
#define OutputSerial Serial
#define DIAGNOSTICS false // Change this to see diagnostics
#define SLEEP_PIN_NO 7
#define SBD_RX_BUFFER_SIZE 270 // Max size of an SBD message

#define GPS_BAUD 9600
#define SBD_BAUD 19200

// Declare the IridiumSBD object
IridiumSBD modem(IridiumSerial, SLEEP_PIN_NO);

Uart Serial2 (&sercom1, 10, 11, SERCOM_RX_PAD_2, UART_TX_PAD_0);

// The TinyGPS++ object
TinyGPSPlus gps;

File gpsLogFile;
File rxLogFile;
File txLogFile;



const long messageTimeInterval = 300000; // In milliseconds; 300000 is 5 minutes; defines how frequenty the program sends messages TODO TODO TODO FIXME FIXME FIXME XXX XXX                                                                        
const long shutdownTimeInterval = 14400000; // In milliseconds; 14400000 is 4 hours; defines after what period of time the program stops sending messages
const String gpsLogName = "77SBDG.LOG"; // Must be in form XXXXXXXX.log; no more than 8 'X' characters
const String eventLogName = "77SBDE.LOG"; // Must be in form XXXXXXXX.log; no more than 8 'X' characters
const String rxLogName = "77SBDR.LOG"; // Must be in form XXXXXXXX.log; no more than 8 'X' characters
const String txLogName = "77SBDT.LOG"; // Must be in form XXXXXXXX.log; no more than 8 'X' characters



// DO NOT CHANGE THESE
const int chipSelect = 4; // Pinn for SPI
unsigned long startTime; // The start time of the program
unsigned long lastMillisOfMessage = 0;
bool sendingMessages = true; // Whether or not the device is sending messages; begins as true TODO
uint8_t sbd_buf[28]; // TODO
uint8_t sbd_rx_buf[SBD_RX_BUFFER_SIZE];

uint8_t gps_hour;
uint8_t gps_min;
uint8_t gps_sec;
double gps_lat;
double gps_lng;
double gps_alt;
double gps_hdop;
int sbd_csq;

#define SBD

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Start the console serial port
  Serial.begin(115200);
  for (int i = 0; i < 10; i++)
  {
    Serial.println(i);
    delay(950);
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
  }
  GpsSerial.begin(GPS_BAUD);
  IridiumSerial.begin(SBD_BAUD);


  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  SD.begin(chipSelect);
  gpsLogFile = SD.open(gpsLogName, FILE_WRITE);
  rxLogFile = SD.open(rxLogName, FILE_WRITE);
  txLogFile = SD.open(txLogName, FILE_WRITE);


  for (int k = 0; k < SBD_RX_BUFFER_SIZE; k++)
  {
    sbd_rx_buf[k] = 0;
  }

  int err;

  // Start the serial port connected to the satellite modem
  OutputSerial.print("Began Iridium serial.");

  OutputSerial.print("Began GPS serial.");


#ifdef SBD
  // Begin satellite modem operation
  OutputSerial.println("Starting modem...");
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    OutputSerial.print("Begin failed: error ");
    OutputSerial.println(err);
    logprint("Begin failed: error ");
    logprintln(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      SerialUSB.println("No modem detected: check wiring.");
    return;
  }
  OutputSerial.println("Modem started");

#ifdef ISBD_CHECK_FIRMWARE
  // Example: Print the firmware revision
  char version[12];
  err = modem.getFirmwareVersion(version, sizeof(version)); // TODO
  if (err != ISBD_SUCCESS)
  {
    OutputSerial.print("FirmwareVersion failed: error ");
    OutputSerial.println(err);
    logprint("FirmwareVersion failed: error ");
    logprintln(err);
    return;
  }
  OutputSerial.print("Firmware Version is ");
  OutputSerial.print(version);
  OutputSerial.println(".");
#endif

  // Example: Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  err = modem.getSignalQuality(sbd_csq);
  if (err != ISBD_SUCCESS)
  {
    OutputSerial.print("SignalQuality failed: error ");
    OutputSerial.println(err);
    logprint("SignalQuality failed: error ");
    logprintln(err);
    sbd_csq = 0;
    return;
  }

  OutputSerial.print("On a scale of 0 to 5, signal quality is currently ");
  OutputSerial.print(sbd_csq);
  OutputSerial.println(".");


  // Send the message
  OutputSerial.print("Trying to send the message.  This might take several minutes.\r\n");
  err = modem.sendSBDText("Hello, world!");
  if (err != ISBD_SUCCESS)
  {
    OutputSerial.print("sendSBDText failed: error ");
    OutputSerial.println(err);
    logprint("sendSBDText failed: error ");
    logprintln(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      OutputSerial.println("Try again with a better view of the sky.");
  }

  else
  {
    OutputSerial.println("Hey, it worked!");
    logprintln("Hey, it worked!");
  }
#endif
}

void loop()
{
  ISBDCallback();

  int new_csq = 0;
  int csq_err = modem.getSignalQuality(new_csq);
  if(csq_err == 0) // if executed during an Iridium session, this will yield an ISBD_REENTRANT; keep previous value
  {
    sbd_csq = new_csq;
  }
  else
  {
    Serial.print("CSQ Error: CSQ error code = ");
    Serial.println(csq_err);
  }

  if ((sbd_csq > 0 && (millis() - lastMillisOfMessage) > messageTimeInterval) && (millis() < shutdownTimeInterval) && sendingMessages) {

    String logString = "";
    logString += gps_hour;
    logString += ":";
    logString += gps_min;
    logString += ":";
    logString += gps_sec;
    logString += "\t";

    uint32_t gps_time = gps.time.value();

    memcpy(&sbd_buf    , &gps_time, 4U);
    memcpy(&sbd_buf[4] , &gps_lat,  8U);
    memcpy(&sbd_buf[12], &gps_lng,  8U);
    memcpy(&sbd_buf[20], &gps_alt,  8U);
    
    size_t rx_buf_size = sizeof(sbd_rx_buf); // TODO

   
    txLogFile.print(logString);

    for (int k = 0; k < 28; k++)
    {
      txLogFile.print(sbd_buf[k], HEX);
    }

    // TODO txLogFile.flush();

    logprint(logString);
    logprintln("Attempted to transmit SBD message");
    
    uint8_t sbd_err = modem.sendReceiveSBDBinary(sbd_buf, 28, sbd_rx_buf, rx_buf_size);

    // update time
    logString = "";
    logString += gps_hour;
    logString += ":";
    logString += gps_min;
    logString += ":";
    logString += gps_sec;
    logString += "\t";

    logprint("SBD send receive completed with return code: ");
    logprintln(sbd_err);

    rxLogFile.print(logString);
    rxLogFile.print("\t");
    rxLogFile.print(rx_buf_size);
    rxLogFile.print("\t");
    for (int k = 0; k < rx_buf_size; k++)
    {
      rxLogFile.print(sbd_rx_buf[k], HEX);
      sbd_rx_buf[k] = 0;
    }
    rxLogFile.println();
    rxLogFile.flush();

    lastMillisOfMessage = millis();
  }

  smartDelay(250);

}

bool ISBDCallback()
{

  if (gps.location.isUpdated())
  {
    gps_hour = gps.time.hour();
    gps_min = gps.time.minute();
    gps_sec = gps.time.second();
    gps_lat = gps.location.lat();
    gps_lng = gps.location.lng();
    gps_alt = gps.altitude.meters();
    gps_hdop = gps.hdop.value();



    String logString = "";
    logString += gps_hour;
    logString += ":";
    logString += gps_min;
    logString += ":";
    logString += gps_sec;
    Serial.print(logString);
    Serial.print(",");
    Serial.print(gps_lat, 6);
    Serial.print(",");
    Serial.print(gps_lng, 6);
    Serial.print(",");
    Serial.print(gps_alt, 2);
    Serial.print(",");
    Serial.print(gps_hdop);
    Serial.print(",");
    Serial.print(sbd_csq);
    Serial.println();
    if (gpsLogFile) {
      gpsLogFile.print(logString);
      gpsLogFile.print(",");
      gpsLogFile.print(gps_lat, 6);
      gpsLogFile.print(",");
      gpsLogFile.print(gps_lng, 6);
      gpsLogFile.print(",");
      gpsLogFile.print(gps_alt, 2);
      gpsLogFile.print(",");
      gpsLogFile.print(gps_hdop);
      gpsLogFile.print(",");
      gpsLogFile.print(sbd_csq);
      gpsLogFile.println();
      gpsLogFile.flush();
    }

  }
  else
  {
    smartDelay(50);
    //Serial.println("No Update.");
  }

  return true; // ISBD operation should continue


}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GpsSerial.available())
    {
      gps.encode(GpsSerial.read());
    }
  } while (millis() - start < ms);
}



static void logprint(String s)
{
  File dataFile = SD.open(eventLogName, FILE_WRITE);
  dataFile.print(s);
  dataFile.close();
}

static void logprint(int i)
{
  File dataFile = SD.open(eventLogName, FILE_WRITE);
  dataFile.print(i);
  dataFile.close();
}


static void logprint(double d)
{
  File dataFile = SD.open(eventLogName, FILE_WRITE);
  dataFile.print(d);
  dataFile.close();
}

static void logprintln(String s)
{
  File dataFile = SD.open(eventLogName, FILE_WRITE);
  dataFile.println(s);
  dataFile.close();
}

static void logprintln(int i)
{
  File dataFile = SD.open(eventLogName, FILE_WRITE);
  dataFile.println(i);
  dataFile.close();
}

static void logprintln(double d)
{
  File dataFile = SD.open(eventLogName, FILE_WRITE);
  dataFile.println(d);
  dataFile.close();
}



#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  OutputSerial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  OutputSerial.write(c);
}
#endif
