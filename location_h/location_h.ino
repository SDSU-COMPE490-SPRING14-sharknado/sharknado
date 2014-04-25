#include <Adafruit_GPS.h>
#include "location.h"
#include <string>
#include <Sabertooth.h>
#include <LSM303.h>
#include <Wire.h>
#define LAB_CAL //calibrate the compass for home operation
#include <PID_v1.h>
#include <DueTimer.h>
#include <Servo.h> 


LSM303 compass;

#define MAX_SPEED 40
#define SLOW_SPEED 20
#define GPS_SERIAL Serial2 //had to hack adafruit lib to use Serial2
#define GPS_LED 53
#define MAG_LED 51
#define BTN_PIN 49

sharknado::Location loc;
Sabertooth ST(128);
volatile int counter=0;


Adafruit_GPS GPS(&GPS_SERIAL);
struct latlng
{
    float lat;
    float lng;
} ;

latlng target1= {32.773584f, -117.073308f};
latlng target2= {32.773834f, -117.073419f};
latlng target3= {32.773660f, -117.072823f};
latlng finish= {32.773892f, -117.072835f};
latlng GPScord[] = {target1,target2,target3,finish};
volatile int GPScounter=1;

//distance and heading to target
float current_target_distance=0;
double current_target_heading=0;
double compass_reading = 0;

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
bool northflag = false;


//Specify the links and initial tuning parameters
//PID heading_PID(&current_heading, &motor_turning_coeff, &expected_heading, 2,.1,0, DIRECT);
PID  heading_PID(&compass_reading, &motor_turning_coeff, &current_target_heading, 2,.1,0, DIRECT);
PID speed_PID  (&current_speed, &motor_speed_coeff, &expected_speed, 2,5,1, DIRECT);

//Sevo Code
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
                
void setup() {

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



    //HACK: hard code an initial drive power
    //ST.drive(10);
    
    
    //timer interrupt 
    //Must uncomment line in header file to allow for use with servo.h
    Timer6.attachInterrupt(gps_interrupt); // have to change to timer 6 in order for it to work with servo
    Timer6.start(1000); // Calls every 50ms
    
    //servo pin attachment
       myservo.attach(8);  // attaches the servo on pin 8 to the servo object 
       compass.read();
       compass_reading=compass.heading();
       current_heading = aconv(compass_reading);
    
    // loop to see if gps and navigation are set up
        myservo.write(180); // lock the payload?
		
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
    else if(current_target_distance > 0)   
      {
        expected_speed=0; //this percission is questionable
        dispense_ball(); // calls the ball dropping function
      }
    else if(current_target_distance <=0)   expected_speed=0; //stop, we passed beacon.
}


void gps_interrupt()
{
  GPS.read();
}

void dispense_ball()
{
    Serial.println("Dropping Ball");
    int n;
    ST.drive(0);
    ST.turn(0);
    myservo.write(0);
    for (n=0; n<5;n+=1)
    {
      myservo.write(180);
      delay(500); 
      myservo.write(150);
      delay(500); 
    }
    myservo.write(0);
    delay(3000); 
    for (n=0; n<5;n+=1)
    {
      myservo.write(30);
      delay(500); 
      myservo.write(0);
      delay(500); 
    }
    
    myservo.write(180);
    if (GPScounter<4){
      target1 = GPScord[GPScounter];
      GPScounter++;
    }
    else
    {
       while(1)
      {
         ST.drive(0); // if at home location then do nothing and turn LEDS on
         ST.turn(0);
         digitalWrite(GPS_LED, HIGH);
         digitalWrite(MAG_LED, HIGH);
      } 
    }
}

//this scales a 0-360 heading to -180-180 for the PID library input
float aconv(float theta)
{
    return fmod((theta + 180.0f), 360.0f) - 180.0f;
}
