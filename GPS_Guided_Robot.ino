




// Remember to run the HMC5883L compass calibration test and enter the results into the code below under void Setup() compass.setOffset(0,0);      



#include <Servo.h>                                                  // servo control library
#include <CytronMotorDriver.h>                                     // cytron motor driver library
#include "Wire.h"                                                 // Used by I2C and HMC5883L compass
#include "I2Cdev.h"                                             // I2C Communications Library (used for compass)
//#include "HMC5883L.h"                                             // Library for the compass - Download from Github @ https://github.com/jarzebski/Arduino-HMC5883L
#include <DFRobot_QMC5883.h>                                         // library for QMC5338L compass https://github.com/mechasolution/Mecha_QMC5883L
#include <DFRobot_SHT20.h>
#include <SoftwareSerial.h>                                       // Software Serial for Serial Communications - not used
#include <TinyGPS++.h>                                            // Tiny GPS Plus Library - Download from http://arduiniana.org/libraries/tinygpsplus/
                                                                  // TinyGPS++ object uses Hardware Serial 2 and assumes that you have a
                                                                  // 9600-baud serial GPS device hooked up on pins 16(tx) and 17(rx).   
                                                                                                           

//******************************************************************************************************                                                                  
// GPS Variables & Setup

int GPS_Course;                                                    // variable to hold the gps's determined course to destination
int Number_of_SATS;                                                // variable to hold the number of satellites acquired
TinyGPSPlus gps;                                                   // gps = instance of TinyGPS 
                                                                   // pin 17 (blue)   is connected to the TX on the GPS
                                                                   // pin 16 (yellow) is connected to the RX on the GPS

//******************************************************************************************************
// Setup Cytron Drive Motors


CytronMD motorL(PWM_DIR, 2, 3);                               
CytronMD motorR(PWM_DIR, 4, 5);                              
CytronMD Auger(PWM_DIR, 6,7);                               
CytronMD Actuator(PWM_DIR, 8,9);

                               
int turn_Speed = 100;                                              // motor speed when using the compass to turn left and right
int mtr_Spd = 150;                                                 // motor speed when moving forward and reverse
int drill_spd=255;                                                 // drilling speed
//******************************************************************************************************
// Setup Dropper Motor

Servo Dropper;


int droppercontrol(int value){                       // maps values of -100 and 100 to 1000 and 2000 so that -100-100 range can be used instead
  Dropper.writeMicroseconds(map(value,-100,100,-1000,1000));}

//******************************************************************************************************
// Compass Variables & Setup
DFRobot_QMC5883 compass;
//HMC5883L compass;                                                  // HMC5883L compass(HMC5883L)
//QMC5883L qmc;
int16_t mx, my, mz;                                                // variables to store x,y,z axis from compass (HMC5883L)
int desired_heading;                                               // initialize variable - stores value for the new desired heading
int compass_heading;                                               // initialize variable - stores value calculated from compass readings
int compass_dev = 5;                                               // the amount of deviation that is allowed in the compass heading - Adjust as Needed
                                                                   // setting this variable too low will cause the robot to continuously pivot left and right
                                                                   // setting this variable too high will cause the robot to veer off course

int Heading_A;                                                     // variable to store compass heading
int Heading_B;                                                     // variable to store compass heading in Opposite direction
int pass = 0;                                                      // variable to store which pass the robot is on

//******************************************************************************************************
// Ping Sensor for Collision Avoidance

boolean pingOn = false;                                            // Turn Collision detection On or Off

int trigPin = 43;                                                  // Trig - Orange
int echoPin = 42;                                                  // Echo - Yellow
long duration, inches;
int Ping_distance;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;                                  // Store last time Ping was updated
const long interval = 200;                                         // Ping the Distance every X miliseconds
 
//******************************************************************************************************
// Bluetooth Variables & Setup

String str;                                                        // raw string received from android to arduino
int blueToothVal;                                                  // stores the last value sent over via bluetooth
//int bt_Pin = 34;                                                   // Pin 34 of the Aruino Mega used to test the Bluetooth connection status - Not Used

//*****************************************************************************************************
// GPS Locations

unsigned long Distance_To_Home;                                    // variable for storing the distance to destination

int ac =0;                                                         // GPS array counter
int wpCount = 0;                                                   // GPS waypoint counter
double Home_LATarray[50];                                          // variable for storing the destination Latitude - Only Programmed for 5 waypoint
double Home_LONarray[50];                                          // variable for storing the destination Longitude - up to 50 waypoints


int increment = 0;

void setup() {  
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer
  Serial1.begin(9600);                                             // Serial 1 is for Bluetooth communication - DO NOT MODIFY - JY-MCU HC-06 v1.40
  Serial2.begin(9600);                                             // Serial 2 is for GPS communication at 9600 baud - DO NOT MODIFY - Ublox Neo 6m 
  Serial3.begin(9600);                                             // Serial 3 is for SHT20 Communication Through Bluetooth-Do Not Modify
                                                                                     

  // Ping Sensor
  pinMode(trigPin, OUTPUT);                                        // Ping Sensor
  pinMode(echoPin, INPUT);                                         // Ping Sensor

  // Compass
  Wire.begin();                                                    // Join I2C bus used for the HMC5883L compass
  compass.begin();                                                 // initialize the compass (HMC5883L)
  compass.setRange(HMC5883L_RANGE_1_3GA);                          // Set measurement range  
  compass.setMeasurementMode(HMC5883L_CONTINOUS);                  // Set measurement mode  
  compass.setDataRate(HMC5883L_DATARATE_30HZ);                     // Set data rate  
  compass.setSamples(HMC5883L_SAMPLES_8);                          // Set number of samples averaged  
  //compass.setOffset(0,0);    // Set calibration offset 
  pinMode(10, OUTPUT);
 Dropper.attach(10);



  Startup();                                                       // Run the Startup procedure on power-up one time
}

//********************************************************************************************************
// Main Loop

void loop()
{ 
  
  bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT                                                    
  getGPS();                                                        // Update the GPS location
  getCompass();                                                    // Update the Compass Heading
  Ping();  
  
                                           
}
