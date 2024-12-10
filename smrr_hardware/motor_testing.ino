#include "src/HubWheel/HubWheel.h"
#include "src/CustomServo/CustomServo.hpp"
#include <NewPing.h>

#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define S1_TRIG 48
#define S1_ECHO 49
#define S2_TRIG 53
#define S2_ECHO 52
#define S3_TRIG 50
#define S3_ECHO 51

#define front_US 0
#define left_US 1
#define right_US 2

// Pins for the right motor
#define right_SIGNAL 3 
#define right_VR 13    
#define right_ZF 12
#define right_EL 11

// Pins for the left motor
#define left_SIGNAL 2
#define left_VR 10 
#define left_ZF 9
#define left_EL 8

// wheel indexes
#define RIGHT_WHEEL 0
#define LEFT_WHEEL 1

// timeout
#define TIME_OUT 500
#define FEEDBACK_TIME_OUT 100

// Servo motors
#define head_servo_PIN 46
#define STEP 1
#define STEP_DELAY 50

double timeout_t = 0;
double feedback_timeout_t = 0;

// Power state variable
int power_state = 0;

// Head servo angle
int head_servo_angle = 90;
int previous_head_servo_angle = 90;

//wheel objects
HubWheel right_wheel(right_SIGNAL,right_ZF,right_VR,right_EL,RIGHT_WHEEL);
HubWheel left_wheel(left_SIGNAL,left_ZF,left_VR,left_EL,LEFT_WHEEL);

//Head servo object
CustomServo headServo;

//Ultrasonic sensor array
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(S1_TRIG, S1_ECHO, MAX_DISTANCE),
  NewPing(S2_TRIG, S2_ECHO, MAX_DISTANCE),
  NewPing(S3_TRIG, S3_ECHO, MAX_DISTANCE)
  
};
int isCliff[3] = {0,0,0};
int isCliffSum = 0;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

void right_encoder(){

    /*
        interrupt function for right motor encoder
    */

    if(right_wheel.direction == FORWARD){right_wheel.count += 1;}
    else if(right_wheel.direction == BACKWARD){right_wheel.count -= 1;}

    //right_wheel.velocity = (right_wheel.count - right_wheel.pre_count)*MILIMETERS_PER_TICK/(millis()-right_wheel.t);
    
    if(abs(right_wheel.count)>MAX_COUNT){
        right_wheel.count = 0;
    }
    //Serial.write("Right encoder");
    
    //right_wheel.pre_count = right_wheel.count;
    //right_wheel.t = millis();
}

void left_encoder(){

    /*
        interrupt function for left motor encoder
    */

    if(left_wheel.direction == FORWARD){left_wheel.count += 1;}
    else if (left_wheel.direction == BACKWARD){left_wheel.count -= 1;}

    // left_wheel.velocity = (left_wheel.count - left_wheel.pre_count)*MILIMETERS_PER_TICK/(millis()-left_wheel.t);
    
    if(abs(left_wheel.count)>MAX_COUNT){
        left_wheel.count = 0;
    }
    //Serial.write("Left encoder");
    
    // left_wheel.pre_count = left_wheel.count;
    // left_wheel.t = millis();
}

void moveServo(){
  if(head_servo_angle != previous_head_servo_angle){
        while(headServo.move(head_servo_angle,STEP_DELAY,STEP));
        previous_head_servo_angle = head_servo_angle;
    }
}

void readMsg(){
  /*
   * This function reads velocities, head servo angle and power state from serial when available
   * Message is sent as (float,float,int,int)=>(left_wheel_vel, right_wheel_vel,head_servo_angle,power_state)
  */
  if(Serial.available()>0){
    String data = Serial.readStringUntil('\n');

    // Parse the data using strtok function
    char *ptr = strtok(const_cast<char *>(data.c_str()), ",");

    // Loop through the tokens and convert them 
    for (int i = 0; i < 4 && ptr != NULL; i++){
      if(i==0){left_wheel.target_velocity = atof(ptr);}
      if(i==1){right_wheel.target_velocity = atof(ptr);}   
      if(i==2){head_servo_angle = atoi(ptr);}
      if(i==3){power_state = atoi(ptr);}

      ptr = strtok(NULL,",");
   }

   //Serial.println(String(left_wheel.count)+" "+String(right_wheel.count));
    Serial.println(String(left_wheel.count)+" "+String(right_wheel.count)+" "+String(cm[0])+" "+String(cm[1])+" "+String(cm[2]));

   
   timeout_t = millis();
  }  
}

// void readPWM(){
//   /*
//    * Can be used for testings
//    * This function reads pwm values from serial when available
//    * pwm values are sent as (int,int)=>(left_wheel_pwm, right_wheel_pwm)
//   */
//   if(Serial.available()>0){
//     String data = Serial.readStringUntil('\n');

//     // Parse the data using strtok function
//     char *ptr = strtok(const_cast<char *>(data.c_str()), ",");

//     // Loop through the tokens and convert them to integers
//     for (int i = 0; i < 2 && ptr != NULL; i++){
//       if(i==1){right_wheel.pwm_vel = atoi(ptr);}
//       if(i==0){left_wheel.pwm_vel = atoi(ptr);}
// //      Serial.println(velocity[i]);
//       ptr = strtok(NULL,",");
//    }
//    Serial.println(String(left_wheel.count)+" "+String(right_wheel.count));//delay(5);
//   }  
// }

//void echoCheck() { // If ping received, set the sensor distance to array.
//  if (sonar[currentSensor].check_timer())
//    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
//}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  // Interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(right_SIGNAL),right_encoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_SIGNAL),left_encoder,CHANGE);

  // attach head servo
  headServo.attach(head_servo_PIN);
  moveServo();

  // motor stopping
  right_wheel.t = millis();
  left_wheel.t = millis();
  right_wheel.direction = STOP;
  left_wheel.direction = STOP;

  // sonar setup
//  0.4Timer[i] = pingTimer[i - 1] + PING_INTERVAL;}
  
  
}

void loop() {



    readMsg();
    // readPWM();
    
    

    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      isCliffSum = 0;
      delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between     
      cm[i]=sonar[i].ping_cm();
      if (cm[i]>12){
        isCliff[i] = 1;
        isCliffSum += 1;
      }
      else{
        isCliff[i]=0;
      }
    }

    if(isCliffSum>0){
      right_wheel.target_velocity = 0;
      left_wheel.target_velocity = 0;
    }
    else{
      right_wheel.calPWM(); 
      
      left_wheel.calPWM();
      
    }

    right_wheel.drive();
    delay(1);
    left_wheel.drive(); 
    
//    Serial.println(String(cm[0])+" "+String(cm[1])+" "+String(cm[2]));
//    delay(500);
  
    
    moveServo();

//    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
//    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
//      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
//      //if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
//      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
//      currentSensor = i;                          // Sensor being accessed.
//      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
//      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
//      }
//    }
   
   if(millis()-feedback_timeout_t>= FEEDBACK_TIME_OUT)
   {
    feedback_timeout_t = millis();

    String message = String(left_wheel.count) + " " + 
                 String(right_wheel.count) + " " + 
                 String(cm[0]) + " " + 
                 String(cm[1]) + " " + 
                 String(cm[2]) + "\n";
      
    Serial.write(message.c_str());
   }
    
   if((millis()-timeout_t) >= TIME_OUT){
     right_wheel.target_velocity = 0;
     left_wheel.target_velocity = 0;
   }


    // debugging serial output
    //Serial.println("right = "+String(right_wheel.velocity)+", dir = "+String(right_wheel.direction)+", left = "+String(left_wheel.velocity)+", dir = "+String(left_wheel.direction));
  
}
