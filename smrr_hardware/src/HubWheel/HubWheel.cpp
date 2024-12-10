#include "HubWheel.h"

HubWheel::HubWheel(uint8_t SIGNAL_, uint8_t ZF_, uint8_t VR_, uint8_t EL_, uint8_t wheel_type_){
    SIGNAL = SIGNAL_;
    ZF = ZF_;
    VR = VR_;
    EL = EL_;
    wheel_type = wheel_type_;

    pinMode(EL,OUTPUT);
    pinMode(SIGNAL,INPUT);
    pinMode(VR,OUTPUT);
    pinMode(ZF,OUTPUT);
}



void HubWheel::drive(){
    if(pre_pwm_vel != pwm_vel){
      
      if(pwm_vel>0){

        if(direction == BACKWARD || direction == STOP){
          digitalWrite(EL,LOW);
          direction = FORWARD;
        }  

        analogWrite(VR, pwm_vel);  
        delay(10);
        digitalWrite(ZF,(wheel_type)?LEFT_FORWARD:RIGHT_FORWARD);
        delay(10);
        digitalWrite(EL,HIGH);

        // Serial.println("Forward right");
      }

      else if(pwm_vel<0){

        if(direction == FORWARD || direction == STOP){
          digitalWrite(EL,LOW);
          direction = BACKWARD;
        } 

        analogWrite(VR, abs(pwm_vel));  
        delay(10);
        digitalWrite(ZF,(wheel_type)?LEFT_BACKWARD:RIGHT_BACKWARD);
        delay(10);
        digitalWrite(EL,HIGH);

        // Serial.println("Backward right");
      }

      else if(pwm_vel == 0){
        digitalWrite(EL,LOW);
        direction = STOP;
        sum_of_errors = 0;
        velocity = 0;
        // count = 0;
        // pre_count = 0;
        // Serial.println("Stopping right");
        analogWrite(VR, 0);
        digitalWrite(EL,HIGH);

      }
      pre_pwm_vel = pwm_vel;
  }
}

void HubWheel:: calPWM(){

  float error = target_velocity - velocity;
  
  //sum_of_errors += error;
  //pwm_vel = (int)(error*KP) + pwm_vel;
  
  //The following speed calculation linear curve is for 12V operation
  //Need to calibrate if the driving voltage is changed
  //if you wish to use PID driving, uncomment the very first two lines and comment the below code

   if(wheel_type==RIGHT_WHEEL){    
     if(target_velocity>0){pwm_vel = int(25.16+134*target_velocity);}
     else{pwm_vel = int(-24.75+133*target_velocity);}
   }
   else if(wheel_type==LEFT_WHEEL){
     if(target_velocity>0){pwm_vel = int(21.89 + 140.84*target_velocity);}
     else{pwm_vel = int(-20.857+141.1*target_velocity);}
    // else{pwm_vel = int(30+141.1*target_velocity);}
   }

  if(target_velocity == 0){pwm_vel = 0;}

  if(abs(pwm_vel)>MAX_PWM){
    if(pwm_vel>0){pwm_vel = MAX_PWM;}
    else if(pwm_vel<0){pwm_vel = -MAX_PWM;}
  }
  // Serial.println(String(wheel_type)+":"+String(pwm_vel)+", error:"+String(error));
}