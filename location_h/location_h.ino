#include <Adafruit_GPS.h>
#include "location.h"
#include <string>

#define MAX_SPEED 3
#define SLOW_SPEED 1

sharknado::Location loc;
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
struct latlng
{
	float lat;
	float lng;
} ;

latlng target1={32.744899f, -117.137991f};

//distance and heading to target
float current_target_distance=0;
float current_target_heading;

//trusted lat lng
latlng current_latlng;
latlng current_gps_latlng;

//this is the trusted heading that stearing will rely upon. It should be the output of some (Kalman) filter
//which will be a function of magnetometer and GPS
float current_heading=0;
int motor_turning_coeff=0; //current val of motor steering interface
float current_gps_heading;
float current_magnetometer_heading;
int expected_heading;

//trusted speed -- function of encoders and GPS
float current_speed=0;
int motor_speed_coeff=0; //current val of motor speed interface
float current_gps_speed;
float current_encoder_speed;
//this is the speed we WANT to go
int expected_speed=0;





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
   control_speed();
   control_heading();
   
   //now update motors based on new estimates:
   //ST.drive(motor_speed_coeff)
   //ST.turn(motor_turning_coeff)
   
   Serial.print(current_latlng.lat, 9); Serial.print(", ");
   Serial.print(current_latlng.lng, 9); Serial.print(", ");
   Serial.print("distance: "); Serial.print(current_target_distance);Serial.print(", ");
   Serial.print("initial heading: "); Serial.println(current_target_heading);
   Serial.print("Expected Speed: "); Serial.print(expected_speed); Serial.print(", Actual Speed: "); Serial.println(current_speed);
   Serial.print("motor_speed_coeff: ");Serial.println(motor_speed_coeff);
   Serial.print("motor_turning_coeff");Serial.println(motor_turning_coeff);
    }//end timer
}//end loop

void update_dist_and_heading_to_target()
{
    float results [3];
    loc.computeDistanceAndBearing(current_latlng.lat, current_latlng.lng, target1.lat, target1.lng, results);
    current_target_distance = results[0];
    current_target_heading = results[1];
    expected_heading = current_target_heading;
}

void update_expected_speed()
{
  if(current_target_distance > 19)       expected_speed=MAX_SPEED; //full speed for up to 20 meters to beacon
  else if(current_target_distance > 3)   expected_speed=SLOW_SPEED; //go slow for 17 meters
  else if(current_target_distance > 0)   expected_speed=1; //this percission is questionable
  else if(current_target_distance <=0)   expected_speed=0; //stop, we passed beacon.
}

void control_speed()
{
  int diff=current_speed-expected_speed;
  
  //increment/decrement motor speed by factor of 1 until speed stabalizes
  (diff < 0) ? motor_speed_coeff++ : motor_speed_coeff-- ;
  
  Serial.print("Speed Adjustment: "); Serial.println(diff);
}


void control_heading()
{
  //need to revisit this
  //i forget the range of the turning interface
  //for example of heading is on left I think I need to -- and on right I need to ++
  //but the compass will only return pos 0-360 vals.
  int diff=current_heading - expected_heading;
  (diff < 0) ? motor_turning_coeff++ : motor_turning_coeff-- ;

}
