#include <Adafruit_GPS.h>
#include "location.h"
#include <string>
#include <Sabertooth.h>
#include <LSM303.h>
#include <Wire.h>
#include <PID_v1.h>
#include <DueTimer.h>
#include <Servo.h> //must include here to to link in payloaddumper.cpp
#include "PayloadDumper.h"
#include "BeaconSensor.h"
#include "FastRunningMedian.h"
#include "FloatingFastRunningMedian.h"
#define LAB_CAL //calibrate the compass for home operation
#define ARDUINO
using namespace sharknado;


enum STATES { START, SEARCH, BEACON, ESCAPE, END };
int state; //0,1,2 = search, beacon, obstacle
int next_state;

#define MAX_SPEED 40
#define SLOW_SPEED 20
#define GPS_SERIAL Serial2 //had to hack adafruit lib to use Serial2
#define GPS_LED 50
#define MAG_LED 51
#define BTN_PIN 49




//beacon antenna
#define ANTENNA_PIN A0
FastRunningMedian<unsigned int,5, 0> antenna_median;

//ultrasonic vars
FastRunningMedian<unsigned int,5, 999> left_median;
FastRunningMedian<unsigned int,5, 999> center_median;
FastRunningMedian<unsigned int,5, 999> right_median;
#define US_ROUNDTRIP_CM 57 
#define trigPinL 32     // Pin 12 trigger output
#define trigPinR 34
#define trigPinC 36
#define echoPinL 33                                    // Pin 2 Echo input
#define echoPinR 35
#define echoPinC 37
volatile long echo_startL = 0;                         // Records start of echo pulse
volatile long echo_endL = 0;                           // Records end of echo pulse
volatile long echo_durationL = 0;                      // Duration - difference between end and start
volatile long echo_startR = 0;                         // Records start of echo pulse
volatile long echo_endR = 0;                           // Records end of echo pulse
volatile long echo_durationR = 0;                      // Duration - difference between end and start
volatile long echo_startC = 0;                         // Records start of echo pulse
volatile long echo_endC=  0;                           // Records end of echo pulse
volatile long echo_durationC = 0;                      // Duration - difference between end and start
int left_dist;
int center_dist;
int right_dist;


LSM303 compass;
FloatingFastRunningMedian compass_median(5, 0) ;
Location loc;
PayloadDumper payload;
BeaconSensor beacon_sensor;
Sabertooth ST(128);
Adafruit_GPS GPS(&GPS_SERIAL);

const int TARGET_COUNT=4;
Location::latlng target1= {32.739051f, -117.139245f};
Location::latlng target2= {32.738997f, -117.140717f};
Location::latlng target3= {32.738681f, -117.140031f};
Location::latlng home=    {32.738393f, -117.139731f};

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
bool beacon=false; //call bacon proximity routine


//Specify the links and initial tuning parameters
//PID heading_PID(&current_heading, &motor_turning_coeff, &expected_heading, 2,.1,0, DIRECT);
PID  heading_PID(&compass_reading, &motor_turning_coeff, &current_target_heading, 2,.1,0, DIRECT);
PID speed_PID  (&current_speed, &motor_speed_coeff, &expected_speed, 2,5,1, DIRECT);



void setup() {
    //start in start mode
    next_state=START;

    //setup antenna
     pinMode(ANTENNA_PIN, INPUT);
     
     
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
    Serial.print("Compass status: ");
    Serial.println(compass_status);
    compass.enableDefault();

#ifdef HOME_CAL
    Serial.println("Calibrating compass for home mode");
    compass.m_min = (LSM303::vector<int16_t>) {
        -1028,   -784,  +2487
    };
    compass.m_max = (LSM303::vector<int16_t>) {
        +1121,  +1215,  +3276
    };
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
    compass.m_min = (LSM303::vector<int16_t>) {
        -1598,  -2160,  +1195
    } ;
    compass.m_max = (LSM303::vector<int16_t>) {
        +1755,  +1167,  +1938
    };
#endif


    //timer interrupt for GPS
    Timer.getAvailable().attachInterrupt(gps_interrupt).start(1000); // Calls every 50ms

    //initialize PIDs
    heading_PID.SetMode(AUTOMATIC);
    heading_PID.SetOutputLimits(-7,7);

    speed_PID.SetMode(AUTOMATIC);
    speed_PID.SetOutputLimits(0,50);

    //ultras
    pinMode(trigPinL, OUTPUT);                           // Trigger pin set to output
    pinMode(trigPinR, OUTPUT);
    pinMode(trigPinC, OUTPUT);
    pinMode(echoPinL, INPUT);
    pinMode(echoPinR, INPUT);
    pinMode(echoPinC, INPUT); // Echo pin set to input
    Timer6.attachInterrupt(trigger_pulse).start(250000);     //sends a pulse every 250 ms and lasts 50 us

    attachInterrupt(echoPinL, echo_interruptL, CHANGE);  // Attach interrupt to the sensor echo input
    attachInterrupt(echoPinR, echo_interruptR, CHANGE);  // Attach interrupt to the sensor echo input
    attachInterrupt(echoPinC, echo_interruptC, CHANGE);  // Attach interrupt to the sensor echo input


    //interrupt for beacon sensor
    Timer.getAvailable().attachInterrupt(antenna_interrupt).start(50000); //50ms

}

int iterations=0;
unsigned long running_duration;
void loop() {

    //next state logic
    state=next_state;

    if (state==START) 		start_routine();
    else if(state==SEARCH) 	search_routine();
    else if(state==BEACON) 	beacon_routine();
    else if(state==ESCAPE)	escape_routine();
    else if(state==END)   end_routine();

  iterations++;
  running_duration = millis() / iterations;
}//end loop

bool mag_ready=false;
bool gps_ready=false;
bool shark_flag=false;
void start_routine()
{
    //initialize the motors to 0
    ST.drive(0);
    mag_ready=false;
    gps_ready=false;
    shark_flag=false;
//    digitalWrite(MAG_LED, LOW);
//    digitalWrite(GPS_LED, LOW);

    //wait for Mag. to initialize
    compass.read();
    compass_reading=compass.heading();
    current_heading = aconv(compass_reading);
    if (current_heading!=current_heading) //wait for magnetometer reading, nan != nan is always true
    {
        Serial.println("Magnetometer is not set up...Please unplug and try again");
        digitalWrite(MAG_LED, LOW);
    }
    else //mag is ready
    {
        mag_ready=true;
        digitalWrite(MAG_LED, HIGH); //turn Yellow LED on
    }

    //GPS Check
    if (!GPS.fix)
    {
        if (GPS.newNMEAreceived())
            if (!GPS.parse(GPS.lastNMEA()))

                Serial.println("No GPS Fix...Please Hold");
        digitalWrite(GPS_LED, LOW);
    }
    else //GPS fix initiated
    {
        gps_ready=true;
        digitalWrite(GPS_LED, HIGH); //Turn Red LED on
    }


    //Button check
    if(digitalRead(BTN_PIN)==HIGH && shark_flag==false && gps_ready && mag_ready) //btn is pressed
    {
        Serial.println("I'm waiting for you to push the button!");

    }
    else if(digitalRead(BTN_PIN)==LOW)
    {
        shark_flag=true;

    }

    if(shark_flag && mag_ready && gps_ready)
    {
        Serial.println("Starting sharknado...");
        //why not leave them on?
        //digitalWrite(GPS_LED, LOW); //turn LEDs off
        //digitalWrite(MAG_LED, LOW);

        next_state=SEARCH;
        delay(1000); //wait a second then GO!
    }

}

unsigned long compass_start_time;
unsigned long compass_end_time;
unsigned long compass_last_time;
unsigned long compass_running_time;
unsigned int compass_sample_count=0;
unsigned long compass_running_average=0;
void search_routine()
{
    if(check_collision()) 
	{
		next_state = ESCAPE;
		return;
	}
    else if(beacon_range()) 
	{
		next_state = BEACON;
		return;
	}
	
	
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
        Serial.println("no fix on the gps, transitioning back to start state");
        next_state=START;
        return;

    }

    //compute expected shark speed every cycle
    update_expected_speed();

    if(current_target_distance < 2) next_state=BEACON; //hack until we get beacon RF sensor

    Serial.print(current_latlng.lat, 9);
    Serial.print(", ");
    Serial.print(current_latlng.lng, 9);
    Serial.print(", ");
    Serial.print("distance: ");
    Serial.print(current_target_distance);
    Serial.print(", ");
    Serial.print("GPS heading: ");
    Serial.print(current_target_heading);

    //heading PID update
    compass_start_time=millis();
    compass.read();
    float compass_sample = compass.heading();
    compass_median.addValue(compass_sample);
    compass_reading=compass_median.getMedian(); //filtered compass reading
    compass_end_time=millis();
    unsigned long compass_elapsed=compass_end_time-compass_start_time;
    compass_running_time = compass_running_time + compass_elapsed;
    compass_sample_count++;
    compass_running_average = compass_running_time / compass_sample_count;
    
    //compass error checking
    if (compass_reading!=compass_reading) //wait for magnetometer reading, nan != nan is always true
    {
        Serial.println("Magnetometer is broken, transitioning back to START STATE");
        next_state=START;
        return;
    }
    if (northflag) {
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
    Serial.print("current heading: ");
    Serial.print(compass_reading);
    Serial.print(", expected speed: ");
    Serial.print(expected_speed);
    Serial.print(", loop time: ");Serial.print(running_duration);
    Serial.print(", mag avg: ");Serial.println(compass_running_average);



}

void beacon_routine()
{
    //Drive forward slowly until RSSI is > 90
    //then drop ball
    ST.turn(0);
    ST.drive(SLOW_SPEED);

    //drop golf ball at some point
    //this is hacked to return random number between 50 and 100
    int rssi = antenna_median.getMedian();
    Serial.print("RSSI: ");
    Serial.println(rssi);

    if(rssi>256)
    {
        ST.drive(0); //stop
        payload.dump(); //payload lib

        //set the next target 	//set next state
        target_index++;
        if( (target_index % TARGET_COUNT) == 0) next_state = END; //you are home, stop
        else next_state = SEARCH; //search for next target
    }
}

void escape_routine()
{
  Serial.print("ESCAPE");
    if(!check_collision()) 
	{
		next_state = SEARCH;
		return;
	}
	
    ST.drive(MAX_SPEED);

    int power=MAX_SPEED/2;

    bool left, center, right;
    left = (left_dist < 100) ? true :false;
    center = (center_dist < 100) ? true :false;
    right = (right_dist < 100) ? true :false;


    if(left && !center && !right)
    {
        //go right
        ST.drive(power);
        ST.turn(power);
    }
    else if(!left && center && !right)
    {
        //stop, pivot right
        ST.drive(power);
        ST.turn(power);
    }
    else if(!left && !center && right)
    {
        //go left
        ST.drive(power);
        ST.turn(-power);
    }
    else if(left && center && !right)
    {
        // go right
        ST.drive(power);
        ST.turn(power);
    }
    else if(!left && center && right)
    {
        //go left
        ST.drive(power);
        ST.turn(-power);
    }
    else if(left && center && right)
    {
        //stop, pivot right
        ST.drive(power);
        ST.turn(power);
    }
    else if(left && !center && right)
    {
        //same as center case
        //stop, pivot randomly
        ST.drive(power);
        ST.turn(power );
    }

}

//this is just the busy wait
//the robot shouldn't do anything
void end_routine()
{
    ST.drive(0);
    Serial.println("finished work, home");
    return;
}

void update_dist_and_heading_to_target()
{
    float results [3];
    loc.computeDistanceAndBearing(current_latlng.lat, current_latlng.lng, targets[target_index].lat, targets[target_index].lng, results);
    current_target_distance = results[0];
    current_target_heading = results[1];

    //Checks if heading is in the southern hemisphere
    if (90 < current_target_heading && current_target_heading < 270)  northflag=false; // if it is then north flag is false
    else {
        compass_reading = aconv(compass_reading);//convert to northern heading or -180 to 180
        current_target_heading = aconv(current_target_heading);
        northflag=true;
    }
}

void update_expected_speed()
{
    if(current_target_distance > 9)       expected_speed=MAX_SPEED; //full speed for up to 20 meters to beacon
    else if(current_target_distance > 3)  expected_speed=SLOW_SPEED; //go slow for 17 meters
    else if(current_target_distance > 0)  expected_speed=0; //this percission is questionable
    else if(current_target_distance <=0)  expected_speed=0; //stop, we passed beacon.
}


void gps_interrupt()
{
    GPS.read();
}


//this scales a 0-360 heading to -180-180 for the PID library input
float aconv(float theta)
{
    return fmod((theta + 180.0f), 360.0f) - 180.0f;
}


void trigger_pulse()
{
    //sets all pins to high or triggers the ultrasonic
    digitalWrite(trigPinL, HIGH);
    digitalWrite(trigPinR, HIGH);
    digitalWrite(trigPinC, HIGH);
    Timer7.attachInterrupt(goLow).start(50);     //sets a  timer that will make the pulse go low after 50 us

}

void goLow()
{
    //sets all the pins to low
    //Serial.println("pin goes low");
    digitalWrite(trigPinL, LOW);
    digitalWrite(trigPinR, LOW);
    digitalWrite(trigPinC, LOW);
    Timer7.detachInterrupt();                        //detachs the interupt so that it won't go off again until initialized

}

const int threshold=100;
volatile int left_collision;
void echo_interruptL()
{
    switch (digitalRead(echoPinL))                     // Test to see if the signal is high or low
    {
    case HIGH:                                      // High so must be the start of the echo pulse
        echo_endL = 0;                                 // Clear the end time
        echo_startL = micros();                        // Save the start time
        break;

    case LOW:                                       // Low so must be the end of the echo pulse
        echo_endL = micros();                          // Save the end time
        echo_durationL = echo_endL - echo_startL;        // Calculate the pulse duration
         
        left_median.addValue(echo_durationL); // adds a value
        unsigned int median = left_median.getMedian(); // retieves the median
        
       
        left_dist = median / 58;

        if(left_dist < threshold) left_collision=true;
        else left_collision=false;

        break;
    }
}

volatile int right_collision;
void echo_interruptR()
{
    switch (digitalRead(echoPinR))                     // Test to see if the signal is high or low
    {
    case HIGH:                                      // High so must be the start of the echo pulse
        echo_endR = 0;                                 // Clear the end time
        echo_startR = micros();                        // Save the start time
        break;

    case LOW:                                       // Low so must be the end of hte echo pulse
        echo_endR = micros();                          // Save the end time
        echo_durationR = echo_endR - echo_startR;        // Calculate the pulse duration
        
        right_median.addValue(echo_durationR); // adds a value
        unsigned int median = right_median.getMedian(); // retieves the median
        
        
        right_dist=median / 58;
        if(right_dist < threshold) right_collision=true;
        else right_collision=false;
        break;
    }
}

volatile int center_collision;
void echo_interruptC()
{
    switch (digitalRead(echoPinC))                     // Test to see if the signal is high or low
    {
    case HIGH:                                      // High so must be the start of the echo pulse
        echo_endC = 0;                                 // Clear the end time
        echo_startC = micros();                        // Save the start time
        break;

    case LOW:                                       // Low so must be the end of hte echo pulse
        echo_endC = micros();                          // Save the end time
        echo_durationC = echo_endC - echo_startC;        // Calculate the pulse duration
        
        center_median.addValue(echo_durationC); // adds a value
        unsigned int median = center_median.getMedian(); // retieves the median
        
        center_dist = median / 58;
        if(center_dist < threshold) center_collision=true;
        else center_collision=false;
        break;
    }
}

bool check_collision()
{
    Serial.print( "L= ");
    Serial.print(left_dist);
    Serial.print(" C= ");
    Serial.print(center_dist);
    Serial.print(" R= ");
    Serial.println(right_dist);

    if(left_collision || center_collision || right_collision) return true;
    return false;


}

bool beacon_range()
{
    return false;
}

void antenna_interrupt()
{
    int sample=analogRead(ANTENNA_PIN);
}




