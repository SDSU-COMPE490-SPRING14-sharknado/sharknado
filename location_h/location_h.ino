#include <Adafruit_GPS.h>
#include "location.h"
#include <string>
#include <Sabertooth.h>
#include <LSM303.h>
#include <Wire.h>
#define LAB_CAL //calibrate the compass for home operation
#include <PID_v1.h>



LSM303 compass;

#define MAX_SPEED 70
#define SLOW_SPEED 1

#define encoder0PinA  3
#define encoder0PinB  4

sharknado::Location loc;
Sabertooth ST(128);

volatile long encoder0Pos=0;

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
struct latlng
{
    float lat;
    float lng;
} ;

latlng target1= {32.744899f, -117.137991f};

//distance and heading to target
float current_target_distance=0;
float current_target_heading=0;

//trusted lat lng
latlng current_latlng;
latlng current_gps_latlng;

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


//Specify the links and initial tuning parameters
PID heading_PID(&current_heading, &motor_turning_coeff, &expected_heading, 2,.1,0, DIRECT);
PID speed_PID  (&current_speed, &motor_speed_coeff, &expected_speed, 2,5,1, DIRECT);


void setup() {
    // put your setup code here, to run once:

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    Serial.begin(9600);
    Serial.println("Sharknado GPS naviation routine!");
    GPS.begin(9600);
    mySerial.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);


    delay(1000);
    // Ask for firmware version
    mySerial.println(PMTK_Q_RELEASE);

    Serial.begin(9600);

    // pinMode(encoder0PinA, INPUT);
    // pinMode(encoder0PinB, INPUT);
	//encoder interrupts
    //attachInterrupt(encoder0PinA, doEncoder, RISING);  // encoDER ON PIN 2
    //attachInterrupt(encoder1PinA, doEncoder1, RISING);
	

    SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    ST.autobaud();
    ST.drive(0); // The Sabertooth won't act on mixed mode until
    ST.turn(0);


    //setup magnotometer
	//interrupts for magnetometer
	//attachInterrupt(22, handle_compass, RISING);
    Wire.begin();
    bool compass_status=compass.init();
    compass.enableDefault();

#ifdef HOME_CAL
    Serial.println("Calibrating compass for home mode");
    compass.m_min = (LSM303::vector<int16_t>) { -1028,   -784,  +2487};
    compass.m_max = (LSM303::vector<int16_t>) { +1121,  +1215,  +3276};
#else
    //min: {  -617,  -1118,   +988}    max: { +1495,  +1025,  +1155} lab
    //min: {  -752,  -1401,  +2073}    max: { +1224,   +743,  +2942}
    //min: { -1970,  -2568,  +1301}    max: { +1620,   +912,  +1950} field
    //min: { -1563,  -1675,  +1059}    max: { +1797,  +1509,  +1723} outside parking
    //min: { -1028,   -784,  +2487}    max: { +1121,  +1215,  +3276} home
    //min: { -2024,  -1198,   +522}    max: {  +415,  +1047,  +1369} lab friday

    Serial.println("Calibrating compass for lab mode");
    compass.m_min = (LSM303::vector<int16_t>) { -2024,  -1198,   +522} ;
    compass.m_max = (LSM303::vector<int16_t>) {  +415,  +1047,  +1369};
#endif

    heading_PID.SetMode(AUTOMATIC);
    heading_PID.SetOutputLimits(-3,3);

    speed_PID.SetMode(AUTOMATIC);
    speed_PID.SetOutputLimits(0,127);

    //HACK: hard code an initial drive power
    ST.drive(15);

}

uint32_t timer = millis(); //get timestamp

void loop() {
    GPS.read(); //required
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
            return;
    }

    if (timer > millis())  timer = millis(); // if millis() or timer wraps around, we'll just reset it

    if (millis() - timer > 2000) { //every 2 seconds
        timer = millis(); // reset the timer
        if (GPS.fix) {
            float lat = GPS.latitude;
            float lon = GPS.longitude;
            current_latlng.lat=loc.convertDegMinToDecDeg(lat);
            current_latlng.lng=-loc.convertDegMinToDecDeg(lon);
            current_gps_heading=GPS.angle;
            current_gps_speed=GPS.speed;

            update_dist_and_heading_to_target();

        }

        //compute expected shark speed every cycle
        update_expected_speed();



        //Serial.print(current_latlng.lat, 9); Serial.print(", ");
        //Serial.print(current_latlng.lng, 9); Serial.print(", ");
        //Serial.print("distance: "); Serial.print(current_target_distance);Serial.print(", ");
        //Serial.print("initial heading: "); Serial.println(current_target_heading);


    }//end timer


    //heading PID update
    compass.read();
    double compass_reading=compass.heading();
    current_heading = aconv(compass_reading);
    heading_PID.Compute();
    double turning=-motor_turning_coeff; //negate to steer correctly
    ST.turn(turning);
	
    //speed PID update
    speed_PID.Compute();
    ST.drive(motor_speed_coeff);


	//print heading info
    Serial.print("motor_turning_coeff:\t");
    Serial.print(turning);
    Serial.print(", ");
    Serial.print("current heading:\t");
    Serial.println(current_heading);
	
	//print speed info
//    Serial.print("Expected Speed: ");
//    Serial.print(expected_speed);
//    Serial.print(", Actual Speed: ");
//    Serial.println(current_speed);
//    Serial.print("motor_speed_coeff: ");
//    Serial.println(motor_speed_coeff);


}//end loop

void update_dist_and_heading_to_target()
{
    float results [3];
    loc.computeDistanceAndBearing(current_latlng.lat, current_latlng.lng, target1.lat, target1.lng, results);
    current_target_distance = results[0];
    current_target_heading = results[1];

    //just go north heading hack
    expected_heading = 180;//current_target_heading;
}

void update_expected_speed()
{
    if(current_target_distance > 19)       expected_speed=MAX_SPEED; //full speed for up to 20 meters to beacon
    else if(current_target_distance > 3)   expected_speed=SLOW_SPEED; //go slow for 17 meters
    else if(current_target_distance > 0)   expected_speed=1; //this percission is questionable
    else if(current_target_distance <=0)   expected_speed=0; //stop, we passed beacon.
}



void doEncoder()
{
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
        encoder0Pos++;
    } else {
        encoder0Pos--;
    }
}

void handle_compass()
{
	//I dont' know if this is a bad interrupt
	//b/c sometimes the calls to mag
	//can timeout -- take long time
    compass.read();
    double compass_reading=compass.heading();
    current_heading = aconv(compass_reading);
}


//this scales a 0-360 heading to -180-180 for the PID library input
float aconv(float theta)
{
    return fmod((theta + 180.0f), 360.0f) - 180.0f;
}
