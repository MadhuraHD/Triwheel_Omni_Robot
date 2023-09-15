#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include "lf310.h"
#include <Sabertooth.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include "I2Cdev.h"
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

//  Yaw values

float yaw = 0;

float Kp = 6.0;                                //5  // 8
float Ki = -0.0065;                            //0.1
float Kd = 3.0;                             //0.1 // 6.5
float prev_time = 0;
float old_yaw = 0;
float i_term = 0;
float new_yaw;
float goal = 0;
float dt = 0;
float error_yaw = 0;
//float i_term = 0;
float d_term = 0;
float result = 0;

MPU6050 mpu;

Sabertooth ST1(128, Serial1);
Sabertooth ST2(128, Serial2);

USB Usb;
LF310 lf310(&Usb);

void setup() {


#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Starting Logitech F310 gamepad");
  digitalWrite(4, 0);

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  pinMode(4, OUTPUT);
  digitalWrite(4, 1);

  pinMode(12, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(19200);
  Serial2.begin(9600);
  Sabertooth::autobaud(Serial1); // Autobaud is for the whole serial line -- you don't need to do
  Sabertooth::autobaud(Serial2); // Autobaud is for the whole serial line -- you don't need to do

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.calibrateGyro();
  delay(100);
  mpu.setThreshold(1);
  delay(100);
  delay(700);
}

uint8_t oldX = 128;
uint8_t oldY = 128;
uint8_t oldZ = 128;
uint8_t oldRz = 128;
unsigned long counter = 0;

void loop() {

 digitalWrite(12, HIGH);   
//  delay(1000);                       
//  digitalWrite(12, LOW);    
//  delay(1000);                      

  ++counter;
  if (counter % 80 == 1)
    digitalWrite(4, 1);
  else
    digitalWrite(4, 0);

  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Yaw
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Yaw : ");
  Serial.print(yaw);

  // Wait to full timeStep period
  delay((timeStep * 1000) - (millis() - timer));

  new_yaw = yaw;
  Serial.print("\t new_yaw");
  Serial.print(new_yaw);

  Serial.println("Calculate dt and print it...........");

  float curr_time = millis();
  dt = prev_time - curr_time;
  Serial.print("\t dt : ");
  Serial.print(dt);

  Serial.println("dt is calculated and printed...........");

  prev_time = curr_time;

  error_yaw  = goal - new_yaw;
  Serial.print("\t error : ");
  Serial.print(error_yaw);

  i_term = i_term + (error_yaw);
  Serial.print("\t i : ");
  Serial.print(i_term);

  d_term = (old_yaw - new_yaw) / dt;

  Serial.print("\t d");
  Serial.print(d_term);

  float result = (error_yaw * Kp) + (i_term * Ki) + (d_term * Kd);

  old_yaw = new_yaw;

  Serial.print("\t old_yaw : ");
  Serial.print(old_yaw);


  Usb.Task();
  if (lf310.connected()) {
    Serial.print("connected: ");
    //    Serial.print("x: ");
    //    Serial.println(lf310.lf310Data.X);
    if (lf310.lf310Data.X != oldX) {
      Serial.print("\tLeft Joystick X: ");
      Serial.print(lf310.lf310Data.X);
      oldX = lf310.lf310Data.X;
    }

    if (lf310.lf310Data.Y != oldY) {
      Serial.print("\tLeft Joystick Y: ");
      Serial.print(lf310.lf310Data.Y);
      oldY = lf310.lf310Data.Y;
    }

    if (lf310.lf310Data.Z != oldZ) {
      Serial.print("\tRight Joystick X: ");
      Serial.print(lf310.lf310Data.Z);
      oldZ = lf310.lf310Data.Z;
    }

    if (lf310.lf310Data.Rz != oldRz) {
      Serial.print("\tRight Joystick Y: ");
      Serial.println(lf310.lf310Data.Rz);
      oldRz = lf310.lf310Data.Rz;
    }


    if (lf310.buttonClickState.Xbutton) {
      lf310.buttonClickState.Xbutton = 0; // Clear event
      Serial.println(F("X button"));
    }

    if (lf310.buttonClickState.Abutton) {
      lf310.buttonClickState.Abutton = 0; // Clear event
      Serial.println(F("A button"));
    }

    if (lf310.buttonClickState.Bbutton) {
      lf310.buttonClickState.Bbutton = 0; // Clear event
      Serial.println(F("B button"));
    }

    if (lf310.buttonClickState.Ybutton) {
      lf310.buttonClickState.Ybutton = 0; // Clear event
      Serial.println(F("Y button"));
    }

    if (lf310.buttonClickState.LBbutton) {
      lf310.buttonClickState.LBbutton = 0; // Clear event
      Serial.println(F("LB button"));
    }

    if (lf310.buttonClickState.RBbutton) {
      lf310.buttonClickState.RBbutton = 0; // Clear event
      Serial.println(F("RB button"));
    }

    if (lf310.buttonClickState.LTbutton) {
      lf310.buttonClickState.LTbutton = 0; // Clear event
      Serial.println(F("LT button"));
    }

    if (lf310.buttonClickState.RTbutton) {
      lf310.buttonClickState.RTbutton = 0; // Clear event
      Serial.println(F("RT button"));
    }

    if (lf310.buttonClickState.Backbutton) {
      lf310.buttonClickState.Backbutton = 0; // Clear event
      Serial.println(F("Back button"));
    }

    if (lf310.buttonClickState.Startbutton) {
      lf310.buttonClickState.Startbutton = 0; // Clear event
      Serial.println(F("Start button"));
    }

    if (lf310.buttonClickState.LJSP) {
      lf310.buttonClickState.LJSP = 0; // Clear event
      Serial.println(F("Left Joystick deprressed"));
    }

    if (lf310.buttonClickState.RJSP) {
      lf310.buttonClickState.RJSP = 0; // Clear event
      Serial.println(F("Right Joystick deprressed"));
    }

    switch (lf310.lf310Data.btn.dPad) {
      case DPAD_UP:
        Serial.println(F("Up"));
        break;
      case DPAD_RIGHT:
        Serial.println(F("Right"));
        break;
      case DPAD_DOWN:
        Serial.println(F("Down"));
        break;
      case DPAD_LEFT:
        Serial.println(F("Left"));
        break;
      case DPAD_OFF:
        break;
      default:
        Serial.print(F("Unknown state: "));
        PrintHex<uint8_t > (lf310.lf310Data.btn.dPad, 0x80);
        Serial.println();
        break;
    }

    int maplx =  map(lf310.lf310Data.X, 0, 255, -100, 100);     //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f); JoyEvents.lx;
    int maply =  map(lf310.lf310Data.Y, 0, 255, 100, -100);      // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);  JoyEvents.ly;

    float theta = atan2(maply, maplx);

    float v = sqrt((maplx * maplx) + (maply * maply));


    float Vx = v * cos(theta);
    float Vy = v * sin(theta);

    float VA = -Vx;
    float VB = (Vx * sin(0.5236)) - (Vy * cos(0.5236));
    float VC = (Vx * sin(0.5236)) + (Vy * cos(0.5236));

    int V1 =  map( VA, -100, 100, -85, 85);
    int V2 =  map(VB, -100, 100, -85, 85);
    int V3 =  map(VC, -100, 100, -85, 85);

    Serial.print("\t V1 : ");
    Serial.print(V1);
    Serial.print("\t V2 : ");
    Serial.print(V2);
    Serial.print("\t V3 : ");
    Serial.println(V3);


    //Serial.println(V1);
    //Serial.println(V2);
    //Serial.println(V3);

    Serial.print("\t result : ");
    Serial.print(result);

    int final_V1 = V1 + result;
    int final_V2 = V2 + result;
    int final_V3 = V3 + result;

    ST1.motor(1, final_V1);    ///Applying PID
    ST2.motor(2, final_V2);
    ST2.motor(1, final_V3);

    Serial.print("\t final_V1 : ");
    Serial.print(final_V1);
    Serial.print("\t final_V2 : ");
    Serial.print(final_V2);
    Serial.print("\t final_V3 : ");
    Serial.println(final_V3);

    //Serial2.println("sabertooth");

    if (lf310.buttonClickState.Bbutton == true) {
      ST1.motor(1, 0);    //Rotations
      ST2.motor(2, 0);
      ST2.motor(1, 0);
    }
  }
}
