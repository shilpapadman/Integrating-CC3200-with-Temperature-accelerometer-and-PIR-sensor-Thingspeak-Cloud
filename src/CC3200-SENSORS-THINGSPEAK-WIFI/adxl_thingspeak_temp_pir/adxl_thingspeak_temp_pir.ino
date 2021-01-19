
#include <WiFi.h>
#include <Wire.h>
#include <BMA222.h>
#include "Adafruit_TMP006.h"
#include <stdlib.h>

// ThingSpeak Settings
char thingSpeakAddress[] = "api.thingspeak.com";

String writeAPIKey = "...............";   // give the write API key of the ThingsSpeak cloud 
const int updateThingSpeakInterval = 16 * 1000; // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)

//buffer for float to string
char buffer[25];
char buf_x[10];
char buf_y[10];
char buf_z[10];
char buf_pir[3];
char pir_string[5];
char tilt_string[5];
// your network name also called SSID
//char ssid[] = "JioFi2_0989F6";
// your network password
//char password[] = "9596aj5puj";
//char ssid[] = "AndroidAP6712";
//char password[] = "mfee5283";
//char ssid[] = "TP-LINK_DA271C";
//char password[] = "abhinav301295";
char ssid[] = "Sitecom0BF59E";
char password[] = "QAZY8UY6PRN3";
// initialize the library instance:
WiFiClient client;

unsigned long lastConnectionTime = 0; // last time you connected to the server, in milliseconds
boolean lastConnected = false;        // state of the connection last time through the main loop
const unsigned long postingInterval = 10*1000; //delay between updates to xively.com
int failedCounter = 0;
BMA222 mySensor;   // accelerometer sensor

float x;
float y;
float z;
Adafruit_TMP006 tmp006(0x41); // start with a diferent i2c address!

// PIR SENSOR 
#define PIR_MOTION_SENSOR        8           /* sig pin of the PIR sensor */
#define LED                      RED_LED      /* led */


#define ON                       HIGH                    /* led on */
#define OFF                      LOW                     /* led off */
#define _handle_led(x)           digitalWrite(LED, x)    /* handle led */

//ADXL sensor 

#define DEVICE (0x53) // Device address as specified in data sheet ADXL345 

 //byte _buff[6]={0xF0,0X00,0X00,0X80,0X0F,0X0F};
  byte _buff[6];
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

void setup() {
 //Initialize serial and wait for port to open:
 Serial.begin(115200);

 // attempt to connect to Wifi network:
 Serial.print("Attempting to connect to Network named: ");
 // print the network name (SSID);
 Serial.println(ssid);
 // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
 WiFi.begin(ssid, password);
 while ( WiFi.status() != WL_CONNECTED) {
  // print dots while we wait to connect
  Serial.print(".");
  delay(300);
}

  mySensor.begin();
  uint8_t chipID = mySensor.chipID();
  Serial.print("chipID: ");
  Serial.println(chipID);
if (! tmp006.begin()) {
Serial.println("No sensor found");
while (1);
}
  
//  uint8_t chipID = mySensor.chipID();
//  Serial.print("chipID: ");
//  Serial.println(chipID);

Serial.println("\nYou're connected to the network");
Serial.println("Waiting for an ip address");

while (WiFi.localIP() == INADDR_NONE) {
// print dots while we wait for an ip addresss
Serial.print(".");
delay(300);
}

Serial.println("\nIP Address obtained");
printWifiStatus();

//PIR 
 pinMode(PIR_MOTION_SENSOR, INPUT_PULLUP);   /* declare the sig pin as an INPUT */
// interrupts ();
 
//  attachInterrupt (0, isPeopleDetected, RISING); 
 //attachInterrupt(digitalPinToInterrupt(8), isPeopleDetected, RISING);
    pinMode(RED_LED, OUTPUT);            /* declare the red_led pin as an OUTPUT */
    _handle_led(OFF);

    //ADXL
    //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
    writeTo(DATA_FORMAT, 0x00);
    //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
    writeTo(POWER_CTL, 0x08);
    
}
void loop() {

 int pir_check= 0;
 int tilt_check=0;
 // if there's incoming data from the net connection.
 // send it out the serial port. This is for debugging
 // purposes only:
 while (client.available()) {
   char c = client.read();
   Serial.print(c);
 }

  // if there's no net connection, but there was one last time
  // through the loop, then stop the client:
  if (!client.connected() && lastConnected) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
 }

  // if you're not connected, and ten seconds have passed since
  // your last connection, then connect again and send data:
  if (!client.connected() && (millis() - lastConnectionTime > postingInterval)) {

  // accelerometer sensor 
  Serial.println("BMA222 ACCELROMETER SENSOR READING:");
  x = mySensor.readXData();
  Serial.print("X: ");
  Serial.println(x);
  String sobjtx = dtostrf(x,3,3,buf_x);
  Serial.print("sobjtx:");
  Serial.println(sobjtx);
  y = mySensor.readYData();
  Serial.print(" Y: ");
  Serial.println(y);
  String sobjty = dtostrf(y,3,3,buf_y);
  Serial.print("sobjty:");
 Serial.println(sobjty);
  z = mySensor.readZData();
  Serial.print(" Z: ");
  Serial.println(z);
 String sobjtz = dtostrf(z,3,3,buf_z);
 Serial.print("sobjtz:");
 Serial.println(sobjtz);

// tilt check 
 if(x<0 || y<0 || z<0)
 {  
   tilt_check=1;
 }
 if(tilt_check==1)
 {
    Serial.println("tilt check: tilted");
 }
 else
 {
  Serial.println("tilt check: not tilted");
 }
 
  //PIR SENSOR reading 
  if(isPeopleDetected()) {
        _handle_led(ON);
         Serial.println("Movement check: people movement detected");
         pir_check=1;
         delay(300);
        /* if we detect a people, turn on the led */
    } else {
       
        _handle_led(OFF);          /* found nobody, turn off the light */
        Serial.println("Movement check: people movement not detected");
        pir_check=0;
        delay(300);
    }
     
  tmp006.wake();
  // read the temp sensor:
  float objt = tmp006.readObjTempC();
  Serial.println("Temperature Reading: ");
  Serial.print("Object Temperature: ");
  String sobjt = dtostrf(objt,3,3,buffer);
  Serial.print(sobjt);
  Serial.println("*C");
  float diet = tmp006.readDieTempC();
  //Serial.print("Die Temperature: "); Serial.print(diet); 
  //Serial.println("*C");

//PIR SENSOR 
  if(isPeopleDetected()) {
        _handle_led(ON);
        pir_check=1;
         Serial.println("Movement check: people movement detected");
         delay(300);
        /* if we detect a people, turn on the led */
    } else {
       
        _handle_led(OFF);          /* found nobody, turn off the light */
        Serial.println("Movement check: people movement not detected");
        pir_check=0;
        delay(300);
    }

// ADXL
  Serial.println("ADXL345 SENSOR READING:");
  readAccel(); // read the x/y/z tilt
  delay(800); // only read every 0,5 seconds

  
  //convert motion_check reading to a string 
  sprintf(pir_string, "%d", pir_check);
  Serial.print("PIR STRING:");
  Serial.println(pir_string);
  
  //convert tilt_check into a string 
  sprintf(tilt_string, "%d", tilt_check);
  Serial.print("TILT STRING:");
  Serial.println(tilt_string);

  //send to server
  updateThingSpeak("field1=" + sobjt+"&field2="+sobjtx+"&field3="+sobjty+"&field4="+sobjtz+"&field5="+pir_string+"&field6="+tilt_string);
  delay(300);
  }
  
 // store the state of the connection for next time through
 // the loop:
 lastConnected = client.connected();
}
void updateThingSpeak(String tsData)
{
  if (client.connect(thingSpeakAddress, 80))
  {
  client.print("POST /update HTTP/1.1\n");
  client.print("Host: api.thingspeak.com\n");
  client.print("Connection: close\n");
  client.print("X-THINGSPEAKAPIKEY: "+writeAPIKey+"\n");
  client.print("Content-Type: application/x-www-form-urlencoded\n");
  client.print("Content-Length: ");
  client.print(tsData.length());
  Serial.println(">>TSDATALength=" + tsData.length());
  client.print("\n\n");
  client.print(tsData);
  Serial.println(">>TSDATA=" + tsData);
  lastConnectionTime = millis();
  if (client.connected())
  {
  Serial.println("Connecting to ThingSpeak...");
  Serial.println();
  failedCounter = 0;
  }
  else
  {
  failedCounter++;
  Serial.println("Connection to ThingSpeak failed ("+String(failedCounter,DEC)+")");
  Serial.println();
  }
  
  }
  else
  {
  failedCounter++;
  Serial.println("Connection to ThingSpeak Failed ("+String(failedCounter, DEC)+")");
  Serial.println();
  
  lastConnectionTime = millis();
  }
}
// This method calculates the number of digits in the
// sensor reading. Since each digit of the ASCII decimal
// representation is a byte, the number of digits equals
// the number of bytes:

int getLength(int someValue) {
  // there's at least one byte:
  int digits = 1;
  // continually divide the value by ten,
  // adding one to the digit count for each
  // time you divide, until you're at 0:
  int dividend = someValue / 10;
  while (dividend > 0) {
  dividend = dividend / 10;
  digits++;
}
// return the number of digits:
return digits;
}

/* judge if there is a people around */
boolean isPeopleDetected() {
    int sensor_val = digitalRead(PIR_MOTION_SENSOR);        /* read sig pin */
    if(HIGH == sensor_val) {
        return true;                                        /* people detected */
    } else {
        return false;                                       /* people un-detected */
    }
}

void readAccel() {
  uint8_t howManyBytesToRead =6;
  readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
 int x = (((int16_t)_buff[3]) << 8) | _buff[2];   
  int y = (((int)_buff[1]) << 8) | _buff[0];
  int z = (((int)_buff[5]) << 8) | _buff[4];
  if(x>50000)
  {

    x=x-(pow(2,16)-1);
  }

  if(y>50000)
  {

    y=y-(pow(2,16)-1);
  }

  if(z>50000)
  {

    z=z-(pow(2,16)-1);
  }
  Serial.print("ADXL READING:");
   Serial.print("x: ");
  Serial.print( x );
  Serial.print(" y: ");
  Serial.print( y );
  Serial.print(" z: ");
  Serial.println( z );
}

void writeTo(byte address, byte val) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff[i] = Wire.read();    // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

 
