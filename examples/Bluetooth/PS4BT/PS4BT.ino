#include <SPI.h>
#include <PS4BT.h>
#include <usbhub.h>
#include <PID_v1.h>
#include <Ewma.h>
#include <Wire.h>
#include <MPU6050.h>
//Left motor pin (short pin 8 to 19 for interrupt)
#define PWM_L 2
#define leftdir 7
#define leftbrake 9
//Right motor pin (short pin 5 to 18 for interrupt)
#define PWM_R 3
#define rightdir 6
#define rightbrake 4
//May need to change to digital pin-------------------------------------
#define dirPin1 A9 //M4M2 Dir
#define stepPin1 A12 //M4M1 Step
#define dirPin2 A13 //M3M1 Dir 
#define stepPin2 A10 //M3M2 Step
#define enPin A8
#define stepPin3 A6
#define dirPin3 A7
//----------------------------------------------------------------------
#define legrestup 35
#define legrestdown 33
#define backrestup 25
#define backrestdown 27
#define strutup 29
#define strutdown 31
#define transferOut 39
#define transferIn 37
#define LeftBrake 16
#define RightBrake 17
#define selector A11 //Short pin 48 to A11 for selector //JL: A11 -> joystick selector ON or else it will be Bluetooth PS4
#define joybutton 46
#define joystickX A15
#define joystickY A14
#define limitswitch 13 

char data;  
int PS4priority = 2, controllerpriority =1;
int ldir = 1, rdir = 1;
int pwm = 0;
int dstate = 0, tstate = 0, error_count = 0, state = 0, Pstate = 0, x = 0, step1 = 0, step2 = 0;
int leftPwm = 0, left = 0;
int rightPwm = 0, right = 0;
float leftY = 0 , rightX = 0;
int selector_data = 0, x_data = 0, y_data = 0;
bool button_state = false, pressed = false;
float x_vel = 0, y_vel = 0;
double lspeed = 0, rspeed = 0, lavg = 0, ravg =0;
double spd = 0, spdpwm = 0, sp = 0;
double angvel = 0, angpwm = 0, ang_sp = 0, angavg = 0;
unsigned long count = 0, current_time = 0, timer = 0, timediff = 0;
int Leftcount = 0, Rightcount = 0, oldLeft = 0, oldRight = 0;
float f1 =0, f2 = 0;
float Kp = 0, Ki = 0, Kd = 0;
float lscale = 0.5, rscale = 0.5;
USB Usb;
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS4BT PS4(&Btd);
Ewma LFilter(0.1);
Ewma RFilter(0.1);
Ewma AngFilter(0.1);
MPU6050 mpu;
PID speedPID(&spd, &spdpwm, &sp, 2, 0, 0.07, DIRECT);
PID anglePID(&angvel, &angpwm, &ang_sp, 1.72, 0.005, 0.06, DIRECT);

void setup() {
  Serial.begin(115200);
   // Initialize MPU6050
   /*while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);*/
  attachInterrupt(digitalPinToInterrupt(18), Leftinterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), Rightinterrupt, CHANGE);
  TCCR3B = TCCR3B & B11111000 | B00000010;
  speedPID.SetMode(AUTOMATIC);
  anglePID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(50);
  anglePID.SetSampleTime(50); 
  TCCR3B = TCCR3B & 0b11111000 | 0x01;
  speedPID.SetOutputLimits(0,160);
  anglePID.SetOutputLimits(-100,100);
  pinMode(PWM_L,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(enPin,OUTPUT);
  pinMode(stepPin1,OUTPUT);
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin2,OUTPUT);
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin3,OUTPUT);
  pinMode(dirPin3,OUTPUT);
  pinMode(legrestup,OUTPUT);
  pinMode(legrestdown,OUTPUT);
  pinMode(backrestup,OUTPUT);
  pinMode(backrestdown,OUTPUT);
  pinMode(strutup,OUTPUT);
  pinMode(strutdown,OUTPUT);
  pinMode(leftdir,OUTPUT);
  pinMode(rightdir,OUTPUT);
  pinMode(transferIn,OUTPUT);
  pinMode(transferOut,OUTPUT);
  pinMode(selector,INPUT);
  pinMode(joybutton,INPUT);
  pinMode(joystickX,INPUT);
  pinMode(joystickY,INPUT);
  pinMode(limitswitch, INPUT);
  
  analogWrite(PWM_L,0);
  analogWrite(PWM_R,0);
  digitalWrite(legrestup,HIGH);
  digitalWrite(legrestdown,HIGH);
  digitalWrite(backrestup,HIGH);
  digitalWrite(backrestdown,HIGH);
  digitalWrite(strutup,HIGH);
  digitalWrite(strutdown,HIGH);
  pinMode(leftbrake,OUTPUT);
  pinMode(rightbrake,OUTPUT);
  
  
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  
  digitalWrite(leftbrake,HIGH);
  digitalWrite(rightbrake,HIGH);
  timediff = millis();
  current_time = millis();
}

void Leftinterrupt(){
  if(ldir == 1){
    Leftcount += 1;
  }
  else{
    Leftcount -= 1;
  }
}

void Rightinterrupt(){
  if(rdir == 1){
    Rightcount += 1;
  }
  else{
    Rightcount -= 1;
  }
}

void Idle(){        
    digitalWrite(dirPin1,LOW);
    digitalWrite(dirPin2,LOW);
    digitalWrite(dirPin3,LOW);
    digitalWrite(legrestup,HIGH);
    digitalWrite(legrestdown,HIGH);
    digitalWrite(backrestup,HIGH);
    digitalWrite(backrestdown,HIGH);
    digitalWrite(strutup,HIGH);
    digitalWrite(strutdown,HIGH);
    digitalWrite(transferIn,HIGH);
    digitalWrite(transferOut,HIGH);
    spd = 0;
    spdpwm = 0; 
    sp = 0;
    x_vel = 0;
    angvel = 0;
    angpwm = 0;
    ang_sp = 0;
    analogWrite(PWM_R,0);
    analogWrite(PWM_L,0);  
    digitalWrite(leftbrake,HIGH);
    digitalWrite(rightbrake,HIGH);
}
void Selector(){
  if (selector_data < 100) {
    state = 4;
    digitalWrite(leftbrake,LOW);
    digitalWrite(rightbrake,LOW);
    speedPID.SetMode(AUTOMATIC);
    anglePID.SetMode(AUTOMATIC);
  }
  else{
    speedPID.SetMode(MANUAL);
    anglePID.SetMode(MANUAL);  
    digitalWrite(leftbrake,HIGH);
    digitalWrite(rightbrake,HIGH);
    if (selector_data < 400) {
      state = 0;
      spd = 0;
      spdpwm = 0; 
      sp = 0;
      angvel = 0;
      angpwm = 0;
      ang_sp = 0;
      analogWrite(PWM_R,0);
      analogWrite(PWM_L,0);
    }
    else if (selector_data < 650) {
      state = 1;
      spd = 0;
      spdpwm = 0; 
      sp = 0;
      angvel = 0;
      angpwm = 0;
      ang_sp = 0;
      analogWrite(PWM_R,0);
      analogWrite(PWM_L,0);
    }
    else if (selector_data < 900) {
      state = 2;
      spd = 0;
      spdpwm = 0; 
      sp = 0;
      angvel = 0;
      angpwm = 0;
      ang_sp = 0;
      analogWrite(PWM_R,0);
      analogWrite(PWM_L,0);
    }
    else{
      state = 3;
      spd = 0;
      spdpwm = 0; 
      sp = 0;
      angvel = 0;
      angpwm = 0;
      ang_sp = 0;
      analogWrite(PWM_R,0);
      analogWrite(PWM_L,0);
    }
  }
}
void SetVelocity(int x, int y){
    int offsetx = 0;
    double mx = 0;
    int offsety = 0;
    double my = 0;
    if(x <120){
      //Forward
      offsetx = 160;
      mx = -4.0/3.0;
      dstate = 1;
    }
    else if(x >135){
      //Reverse
      offsetx = -180;
      mx = 4.0/3.0;
      dstate = -1;
    }
    else{
      //Drive speed 0
      offsetx = 0;
      mx = 0;
    }
    if(y > 120 && y < 135){
      //Don't turn
      if(tstate == 1){
        Leftcount = 0;
        Rightcount = 0;
        oldLeft = 0;
        oldRight = 0;
        tstate = 0;
      }
      offsety = 0;
      my = 0;
    }
    else{
      //Turn
      offsety = 160;
      my = -5.0/4.0;
      tstate = 1;
    }
    x_vel = (mx * x) + offsetx;
    y_vel = (my * y) + offsety;
   }

void RelayTrigger(int a, int b, int state){
  if(state == 1){
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }
  else{
    digitalWrite(a, HIGH);
    digitalWrite(b, HIGH);
  }
}
void PS4controller(){
      switch(Pstate){
          case 0:
          //Chair going out
           if (PS4.getAnalogButton(L2)>100) {
            if(step1 < 2200){
              digitalWrite(dirPin1,HIGH);
              digitalWrite(dirPin2,LOW);
              digitalWrite(stepPin1,HIGH);
              delayMicroseconds(500); 
              digitalWrite(stepPin1,LOW); 
              delayMicroseconds(500);
              step1++;
            }
            else{
              RelayTrigger(transferIn, transferOut, 1);
            }
          }
          //Chair going in
          else if (PS4.getAnalogButton(R2)>100){
           if (digitalRead(limitswitch) == HIGH ){
              RelayTrigger(transferOut, transferIn, 1);
            }
            else{
              if(step1 > 0){
              digitalWrite(dirPin1,LOW);
              digitalWrite(dirPin2,HIGH);
              digitalWrite(stepPin1,HIGH);  
              delayMicroseconds(500); 
              digitalWrite(stepPin1,LOW); 
              delayMicroseconds(500);
              step1--;
            }
            }
          }
            break;
          case 1:
            if (PS4.getAnalogButton(L2)>100) {           
              RelayTrigger(legrestdown, legrestup, 1);
            }
            else if (PS4.getAnalogButton(R2)>100){
              RelayTrigger(legrestup, legrestdown, 1);
            }
            break;
          case 2:
            if (PS4.getAnalogButton(L2)>100) {              
              RelayTrigger(backrestup, backrestdown, 1);
            }
            else if (PS4.getAnalogButton(R2)>100){
              RelayTrigger(backrestdown, backrestup, 1);
            }
            break;
          case 3:
            if (PS4.getAnalogButton(L2)>100) {              
              RelayTrigger(strutdown, strutup, 1);
            }
            else if (PS4.getAnalogButton(R2)>100){
              RelayTrigger(strutup, strutdown, 1);
            }
            break;
          }
          if (PS4.getAnalogButton(L2)<100 && PS4.getAnalogButton(R2)<100){
            RelayTrigger(strutup, strutdown, 0);
            RelayTrigger(backrestup, backrestdown, 0);
            RelayTrigger(legrestup, legrestdown, 0);
            RelayTrigger(transferIn, transferOut, 0);
          }
    if (PS4.getButtonClick(L1)) {
      //Serial.println("LOW BRAKE");
      digitalWrite(LeftBrake,LOW);
      digitalWrite(RightBrake,LOW);
    }
    if (PS4.getButtonClick(R1)) {
      //Serial.println("HIGH BRAKE");
      digitalWrite(LeftBrake,HIGH);
      digitalWrite(RightBrake,HIGH);
    }
    if (PS4.getButtonClick(TRIANGLE)) {
      //Transfer
      Pstate = 0;
    }
    if (PS4.getButtonClick(CIRCLE)) {
      //leg
      Pstate = 1;
    }
    if (PS4.getButtonClick(SQUARE)) {
      //back
      Pstate = 2;
    }
    if (PS4.getButtonClick(CROSS)) {
      //strut
      Pstate = 3;
    }
    SetVelocity(PS4.getAnalogHat(LeftHatY),PS4.getAnalogHat(RightHatX));
    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS4.disconnect();
    }
    Serial.print(PS4.getAnalogButton(L2));
}

void loop(){ 
  speedPID.Compute();
  anglePID.Compute(); 
  Usb.Task();
  selector_data = analogRead(selector);
  x_data = analogRead(joystickX);
  y_data = analogRead(joystickY);
  if (digitalRead(joybutton) == 1 && !(pressed)){
    button_state = !button_state;
    pressed = true;
  }
  if(digitalRead(joybutton) == 0){ 
    pressed = false;
  }
    if(button_state || PS4.connected()){
      Serial.println("here");
      digitalWrite(enPin,LOW);
      if(millis() - current_time > 45){
        /*Serial.print(" state: ");
        Serial.print(state);
        Serial.print(" Limit Switch State: ");
        Serial.print(digitalRead(limitswitch));
        Serial.print(" Sp: ");
        Serial.print(sp);
        Serial.print(" AngSp:");
        Serial.print(ang_sp);
        Serial.print(" SpdPwm:");
        Serial.print(spdpwm);*/
        Serial.print(" Leftcount:");
        Serial.print(Leftcount);
        Serial.print(" Rightcount:");
        Serial.print(Rightcount);
        /*Serial.print(" x_vel:");
        Serial.print(x_vel);
        Serial.print(" y_vel:");
        Serial.print(y_vel);*/
        Serial.print("  STATE:");
        Serial.print(state);
        Serial.print("  RightPwm:");
        Serial.print(rightPwm*rdir);
        Serial.print("  LeftPwm:");
        Serial.print(leftPwm*ldir);
        Serial.println(" ");
        lspeed = (((Leftcount - oldLeft)*3.14*16/90)/(millis() - current_time))*1000;
        rspeed = (((Rightcount - oldRight)*3.14*16/90)/(millis() - current_time))*1000;
        ravg = RFilter.filter(rspeed);
        lavg = LFilter.filter(lspeed);
        /*if(tstate != 0){
          //Vector norm = mpu.readNormalizeGyro();
          //angvel = norm.ZAxis;
          if(dstate == 0){
            if(abs(lspeed) > abs(rspeed)){
              lscale -= 0.01;
              rscale += 0.01;
            }
            if(abs(lspeed) < abs(rspeed)){
              lscale += 0.01;
              rscale -= 0.01;
            }
          }
          else{
            lscale = 0.5;
            rscale = 0.5;
          }        
          lscale = constrain(lscale,0,1);
          rscale = constrain(rscale,0,1);
        }
        else{
          angvel = (((Rightcount - Leftcount)*3.14*16/90)/(millis() - current_time))*1000;;  
        }*/
        current_time = millis();
        oldLeft = Leftcount;
        oldRight = Rightcount;
        spd = abs((lavg + ravg)/2);
      }

      //If controller is ON
      if(button_state){
        if(x_data < 640 && x_data > 384 && y_data < 640 && y_data > 384){
          
          if(PS4.connected()){
          PS4controller();
          }
          else{
            Idle();
          }
        }
        else{
         /* Serial.print(x_data);
          Serial.print(" ");
          Serial.println(y_data);*/
          switch(state){
            case 0:
            Serial.print("Step : ");
            Serial.println(step1);
            //Chair going out
             if (x_data > 640) {
              if(step1 < 2200){
                digitalWrite(dirPin1,HIGH);
                digitalWrite(dirPin2,LOW);
                digitalWrite(stepPin1,HIGH);
                delayMicroseconds(500); 
                digitalWrite(stepPin1,LOW); 
                delayMicroseconds(500);
                step1++;
              }
              else{
                RelayTrigger(transferIn, transferOut, 1);
                Serial.println("Out");
              }
            }
            //Chair going in
            else if (x_data < 384){
             if (digitalRead(limitswitch) == HIGH ){
                RelayTrigger(transferOut, transferIn, 1);
                Serial.println("In");
              }
              else{
                if(step1 > 0){
                  digitalWrite(dirPin1,LOW);
                  digitalWrite(dirPin2,HIGH);
                  digitalWrite(stepPin1,HIGH);  
                  delayMicroseconds(500); 
                  digitalWrite(stepPin1,LOW); 
                  delayMicroseconds(500);
                  step1--;
                }
              }
            }
              break;
            case 1:
              if (x_data > 640) {           
                RelayTrigger(legrestdown, legrestup, 1);
              }
              else if (x_data < 384){
                RelayTrigger(legrestup, legrestdown, 1);
              }
              break;
            case 2:
              if (x_data > 640) {              
                RelayTrigger(backrestup, backrestdown, 1);
              }
              else if (x_data < 384){
                RelayTrigger(backrestdown, backrestup, 1);
              }
              break;
            case 3:
              if (x_data > 640) {              
                RelayTrigger(strutdown, strutup, 1);
              }
              else if (x_data < 384){
                RelayTrigger(strutup, strutdown, 1);
              }
              break;
            case 4:    
                SetVelocity(x_data/4, y_data/4);                                                                                                 
              break;
          }
        }
      Selector(); 
      }
      else{
        PS4controller();
      }
    if(x_vel == 0){
      sp = 0;
      anglePID.SetOutputLimits(-250,250);
    }
    else{
      anglePID.SetOutputLimits(-100,100);
      if(sp < x_vel){
          sp += 0.1;
      }
      else if(sp > x_vel){
          sp -= 0.1;
      }
    }
    if(y_vel == 0){
      ang_sp = 0;
    }
    else if(ang_sp < y_vel){
        ang_sp += 0.5;
    }
    else if(ang_sp > y_vel){
        ang_sp -= 0.5;
    }
    leftPwm = (spdpwm*dstate) - (angpwm*lscale);
    rightPwm = (spdpwm*dstate) + (angpwm*rscale);
    if(leftPwm < -5){
        ldir = -1;
        digitalWrite(leftdir,HIGH);
    }
    else if(leftPwm > 5){
        ldir = 1;
        digitalWrite(leftdir, LOW);
    }
    if(rightPwm < -5){      
        rdir = -1;
        digitalWrite(rightdir,HIGH);
    }
    else if(rightPwm > 5){
        rdir = 1;
        digitalWrite(rightdir, LOW);
    }
    leftPwm = abs(leftPwm);
    rightPwm = abs(rightPwm);
    leftPwm = constrain(leftPwm,0,160);
    rightPwm = constrain(rightPwm,0,160);
    analogWrite(PWM_R,rightPwm);
    analogWrite(PWM_L,leftPwm);
    
}
  else{
    Serial.println("false");
    digitalWrite(enPin,HIGH);
    Idle();
  }
}
