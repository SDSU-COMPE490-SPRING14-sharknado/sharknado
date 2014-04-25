#include <Adafruit_GPS.h>
#include "location.h"
#include <string>
#include <Sabertooth.h>
#include <LSM303.h>
#include <Wire.h>
#define LAB_CAL //calibrate the compass for home operation
#include <PID_v1.h>
#include <DueTimer.h>



LSM303 compass;

#define MAX_SPEED 40
#define SLOW_SPEED 10

sharknado::Location loc;
Sabertooth ST(128);

volatile int counter=0;


#define mySerial Serial2 //had to hack adafruit lib to use Serial2
Adafruit_GPS GPS(&mySerial);
struct latlng
{
    float lat;
    float lng;
} ;

latlng target1= {32.773467f, -117.073507f};

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



    SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    ST.autobaud();
    ST.drive(0); // The Sabertooth won't act on mixed mode until
    ST.turn(0);


    Wire.begin();
    bool compass_status=compass.init();
    Serial.print("Compass status: ");Serial.println(compass_status);
    compass.enableDefault();
//    //min: {  -617,  -1118,   +988}    max: { +1495,  +1025,  +1155} lab
//    //min: {  -752,  -1401,  +2073}    max: { +1224,   +743,  +2942}
//    //min: { -1970,  -2568,  +1301}    max: { +1620,   +912,  +1950} field
//    //min: { -1563,  -1675,  +1059}    max: { +1797,  +1509,  +1723} outside parking
//    //min: { -1028,   -784,  +2487}    max: { +1121,  +1215,  +3276} home
//    //min: { -2024,  -1198,   +522}    max: {  +415,  +1047,  +1369} lab friday
//    //min: { -2068,  -2272,   +360}    max: { +1358,   +783,  +1189}  outsite friday
//      min: { -1809,  -1856,   +673}    max: { +1819,  +1571,  +1359}  field wed
#ifdef HOME_CAL
    Serial.println("Calibrating compass for home mode");
    compass.m_min = (LSM303::vector<int16_t>) { -1028,   -784,  +2487};
    compass.m_max = (LSM303::vector<int16_t>) { +1121,  +1215,  +3276};
#else
    Serial.println("Calibrating compass for lab mode");
    compass.m_min = (LSM303::vector<int16_t>) { -1809,  -1856,   +673} ;
    compass.m_max = (LSM303::vector<int16_t>) { +1819,  +1571,  +1359};
#endif

    heading_PID.SetMode(AUTOMATIC);
    heading_PID.SetOutputLimits(-7,7);

    speed_PID.SetMode(AUTOMATIC);
    speed_PID.SetOutputLimits(0,127);

    //HACK: hard code an initial drive power
    //ST.drive(10);
    
    
    //timer interrupt
    Timer3.attachInterrupt(gps_interrupt);
    Timer3.start(1000); // Calls every 1ms

}


void loop() {
  

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

    //compute expected shark speed every cycle
    update_expected_speed();



    Serial.print(current_latlng.lat, 9); Serial.print(", ");
    Serial.print(current_latlng.lng, 9); Serial.print(", ");
    Serial.print("distance: "); Serial.print(current_target_distance);Serial.print(", ");
    Serial.print("GPS heading: "); Serial.print(current_target_heading);

    //heading PID update
    compass.read();
    double compass_reading=compass.heading();
    current_heading = aconv(compass_reading);
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
    Serial.print(current_heading);
    Serial.print(", expected speed: \t");Serial.println(expected_speed);
    
    //Serial.print("1: "); Serial.print(encoder0Pos); Serial.print(", 2: "); Serial.println(encoder1Pos);	
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

    expected_heading = aconv(current_target_heading);
    
}

void update_expected_speed()
{
    if(current_target_distance > 19)       expected_speed=MAX_SPEED; //full speed for up to 20 meters to beacon
    else if(current_target_distance > 3)   expected_speed=SLOW_SPEED; //go slow for 17 meters
    else if(current_target_distance > 0)   expected_speed=0; //this percission is questionable
    else if(current_target_distance <=0)   expected_speed=0; //stop, we passed beacon.
}


//timer interrupt
void gps_interrupt()
{
  counter++;
  GPS.read();
}

//this scales a 0-360 heading to -180-180 for the PID library input
float aconv(float theta)
{
    return fmod((theta + 180.0f), 360.0f) - 180.0f;
}
