#include <Adafruit_GPS.h>
#include "location.h"
#include <string>
#include <Sabertooth.h>
#include <LSM303.h>
#include <Wire.h>
#define LAB_CAL //calibrate the compass for home operation
#include <PID_v1.h>
#include <DueTimer.h>
#include <Servo.h> //must include here to to link in payloaddumper.cpp
#include "ProximitySensor.h"
#include "PayloadDumper.h"
#include "BeaconSensor.h"
#define ARDUINO
using namespace sharknado;


enum STATES { SEARCH, BEACON, ESCAPE, DONE_WAIT };
int state; //0,1,2 = search, beacon, obstacle
int next_state;

#define MAX_SPEED 40
#define SLOW_SPEED 20
#define GPS_SERIAL Serial2 //had to hack adafruit lib to use Serial2
#define GPS_LED 53
#define MAG_LED 51
#define BTN_PIN 49

LSM303 compass;
Location loc;
ProximitySensor ultra1; //left
ProximitySensor ultra2; //center
ProximitySensor ultra3; //right
PayloadDumper payload;
BeaconSensor beacon_sensor;
Sabertooth ST(128);
Adafruit_GPS GPS(&GPS_SERIAL);

Location::latlng target1= {32.773467f, -117.073507f};
Location::latlng target2= {32.773467f, -117.073507f};
Location::latlng target3= {32.773467f, -117.073507f};
Location::latlng home= {32.773467f, -117.073507f};

int target_index=0;
Location::latlng targets [] = {target1, target2, target3, home};

//distance and heading to target
float current_target_distance=0;
double current_target_heading=0;
double compass_reading = 0;

//trusted lat lng
Location::latlng current_latlng;
Location::latlng current_gps_latlng;

//this is the trusted heading that stearing will rely upon. It should be the output of some (Kalman) filter
//which will be a function of magnetometer and GPS
double current_heading=0;
double motor_turning_coeff=0; //current val of motor steering interface
float current_gps_heading=0;
float current_magnetometer_heading=0;
double expected_heading=0;

//trusted speed -- function of encoders and GPS
double current_speed=0;
double motor_speed_coeff=0; //current val of motor speed interface
float current_gps_speed=0;
float current_encoder_speed=0;
double expected_speed=0;
bool northflag = false;

//maybe update these values w/ interrupt?
bool collision=false; //call ultrasonic proximity routine
bool beacon_range=false; //call bacon proximity routine


//Specify the links and initial tuning parameters
//PID heading_PID(&current_heading, &motor_turning_coeff, &expected_heading, 2,.1,0, DIRECT);
PID  heading_PID(&compass_reading, &motor_turning_coeff, &current_target_heading, 2,.1,0, DIRECT);
PID speed_PID  (&current_speed, &motor_speed_coeff, &expected_speed, 2,5,1, DIRECT);


                
void setup() {
	//start in search mode
	next_state=SEARCH;

    // Set uop led Pins
      pinMode(GPS_LED,OUTPUT);
      pinMode(MAG_LED,OUTPUT);
      pinMode(BTN_PIN,INPUT);
      digitalWrite(GPS_LED, LOW);
      digitalWrite(MAG_LED, LOW);
      digitalWrite(BTN_PIN, HIGH); //writing to an input?

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    Serial.begin(9600);
    Serial.println("Sharknado GPS naviation routine!");
    GPS.begin(9600);
    GPS_SERIAL.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);


    delay(1000);
    // Ask for firmware version
    GPS_SERIAL.println(PMTK_Q_RELEASE);

//initialize sabertooth
    Serial.begin(9600);
    SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    ST.autobaud();
    ST.drive(0); // The Sabertooth won't act on mixed mode until
    ST.turn(0);


    Wire.begin();
    bool compass_status=compass.init();
    Serial.print("Compass status: ");Serial.println(compass_status);
    compass.enableDefault();

#ifdef HOME_CAL
    Serial.println("Calibrating compass for home mode");
    compass.m_min = (LSM303::vector<int16_t>) { -1028,   -784,  +2487};
    compass.m_max = (LSM303::vector<int16_t>) { +1121,  +1215,  +3276};
#else
//    //min: {  -617,  -1118,   +988}    max: { +1495,  +1025,  +1155} lab
//    //min: {  -752,  -1401,  +2073}    max: { +1224,   +743,  +2942}
//    //min: { -1970,  -2568,  +1301}    max: { +1620,   +912,  +1950} field
//    //min: { -1563,  -1675,  +1059}    max: { +1797,  +1509,  +1723} outside parking
//    //min: { -1028,   -784,  +2487}    max: { +1121,  +1215,  +3276} home
//    //min: { -2024,  -1198,   +522}    max: {  +415,  +1047,  +1369} lab friday
//    //min: { -2068,  -2272,   +360}    max: { +1358,   +783,  +1189} //outsite friday
//    min: { -1809,  -1856,   +673}    max: { +1819,  +1571,  +1359}  field wed


//
    Serial.println("Calibrating compass for lab mode");
    compass.m_min = (LSM303::vector<int16_t>) { -1253,  -2072,   -244} ;
    compass.m_max = (LSM303::vector<int16_t>){ +2042,   +774,   +674};
#endif

    
    //timer interrupt 
    Timer.getAvailable().attachInterrupt(gps_interrupt).start(1000); // Calls every 50ms
     
     compass.read();
     compass_reading=compass.heading();
     current_heading = aconv(compass_reading);
    
    // loop to see if gps and navigation are set up
  
		
      while (current_heading!=current_heading) //wait for magnetometer reading, nan != nan is always true
      {
          compass_reading=compass.heading();
          current_heading = aconv(compass_reading);
          Serial.print("Magnetometer is not set up...Please unplug and try again");
          digitalWrite(MAG_LED, LOW);
      }
      digitalWrite(MAG_LED, HIGH); //turn Yellow LED on
	  
	  
      while (!GPS.fix) //wait for a GPS fix
      {
		  //need these two lines to run functions. probably don't need in if statements but I'm to lazy to change it
         // if (GPS.newNMEAreceived()) { 
         //    GPS.parse(GPS.lastNMEA());
         // } why do you need any of this?
		  
          Serial.println("No GPS Fix...Please Hold"); 
          digitalWrite(GPS_LED, LOW);
      }
      digitalWrite(GPS_LED, HIGH); //Turn Red LED on

    update_dist_and_heading_to_target(); //calculate heading and distance
    
    //initialize PIDs
    heading_PID.SetMode(AUTOMATIC);
    heading_PID.SetOutputLimits(-7,7);
    speed_PID.SetMode(AUTOMATIC);
    speed_PID.SetOutputLimits(0,50);
    
    while(digitalRead(BTN_PIN)==HIGH) //wait until button is pressed
      {  
        Serial.println("I'm waiting for you to push the button!");
      }
      digitalWrite(GPS_LED, LOW); //turn LEDs off 
      digitalWrite(MAG_LED, LOW);
      delay(1000); //wait a second then GO!



}


void loop() {
	
	//next state logic
	state=next_state;

	if(state==SEARCH) 		search_routine();
	else if(state==BEACON) 	beacon_search_routine();
	else if(state==ESCAPE)	escape_routine();
	else if(state==DONE_WAIT)done_wait();

}//end loop

void update_dist_and_heading_to_target()
{
    float results [3];
    loc.computeDistanceAndBearing(current_latlng.lat, current_latlng.lng, target1.lat, target1.lng, results);
    current_target_distance = results[0];
    current_target_heading = results[1];
   Serial.println(current_target_heading);
    if (current_target_heading<270){  //Checks if heading is in the southern hemisphere 
       if (current_target_heading>90){

        northflag=false; // if it is then north flag is false
       }else
      {
              compass_reading = aconv(compass_reading);//convert to northern heading or -180 to 180
              current_target_heading = aconv(current_target_heading);
              northflag=true;
       }
      }
    else{
              compass_reading = aconv(compass_reading);//convert to northern heading or -180 to 180
              current_target_heading = aconv(current_target_heading);
              northflag=true;
    }
}

void update_expected_speed()
{
    if(current_target_distance > 9)       expected_speed=MAX_SPEED; //full speed for up to 20 meters to beacon
    else if(current_target_distance > 3)   expected_speed=SLOW_SPEED; //go slow for 17 meters
    else if(current_target_distance > 0) expected_speed=0; //this percission is questionable
    else if(current_target_distance <=0)   expected_speed=0; //stop, we passed beacon.
}


void gps_interrupt()
{
  GPS.read();
}



void search_routine()
{	
  if(collision) next_state=ESCAPE;
  else if(beacon_range) next_state=BEACON;
  else if(current_target_distance < 3) next_state=BEACON; //hack until we get beacon RF sensor
	
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
            return;
    }
    if (GPS.fix) {
        float lat = GPS.latitude;
        float lon = GPS.longitude;
        current_latlng.lat=loc.convertDegMinToDecDeg(lat);
        current_latlng.lng=-loc.convertDegMinToDecDeg(lon);
        current_gps_heading=GPS.angle;
        current_gps_speed=GPS.speed;
        update_dist_and_heading_to_target();
    }
    else
    {
      Serial.println("no fix on the gps");
    }

    //compute expected shark speed every cycle
    update_expected_speed();

    Serial.print(current_latlng.lat, 9); Serial.print(", ");
    Serial.print(current_latlng.lng, 9); Serial.print(", ");
    Serial.print("distance: "); Serial.print(current_target_distance);Serial.print(", ");
    Serial.print("GPS heading: "); Serial.print(current_target_heading);

    //heading PID update
    compass.read();
    compass_reading=compass.heading();
    if (northflag){
      compass_reading = aconv(compass_reading); //if in northern hemisphere must convert. must have this because compass.read is called after distance update function
    }
    heading_PID.Compute();
    double turning=-motor_turning_coeff; //negate to steer correctly
    ST.turn(turning);
	
    //speed PID update
//    speed_PID.Compute();
    ST.drive(expected_speed);
	
	int coeff=expected_speed/5;
	heading_PID.SetOutputLimits(-coeff,coeff); //turning is  a function of speed

	//print heading info
    Serial.print(", motor_turning_coeff:\t");
    Serial.print(turning);
    Serial.print(", ");
    Serial.print("current heading:\t");
    Serial.print(compass_reading);
    Serial.print(", expected speed: \t");Serial.println(expected_speed);
}

void beacon_search_routine()
{
	//Drive forward slowly until RSSI is > 90
        //then drop ball
        ST.drive(SLOW_SPEED);

	//drop golf ball at some point
   //this is hacked to return random number between 50 and 100
	int rssi = beacon_sensor.sample();
        Serial.print("RSSI: ");Serial.println(rssi);

	if(rssi>90)
	{
                
		payload.dump(); //payload lib

		//set the next target 	//set next state
		target_index++;
		if( (target_index % 4) == 0) next_state=DONE_WAIT; //you are home, stop
		else next_state = SEARCH; //search for next target
	}
}

void escape_routine()
{
	// cout << "obstacle..." << endl;
	bool collision=((rand()%10)>0); //hack to fake ultrasonic sensor hit
	
        if(collision) next_state=ESCAPE;
	else next_state=SEARCH; //clear of obstacle, go back to normal search


	//implement escape logic?
	//just slow down and make a sharp left or right turn
	//ST.power(LOW);
	//ST.turn(MAX_TURN);
}

//this is just the busy wait
//the robot shouldn't do anything
void done_wait()
{
	Serial.println("finished work, home");
	return;
}


//this scales a 0-360 heading to -180-180 for the PID library input
float aconv(float theta)
{
    return fmod((theta + 180.0f), 360.0f) - 180.0f;
}
