// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <math.h>
#include <SPI.h>
#include "lf310.h"
#include <Sabertooth.h>

Sabertooth ST1(128, Serial1);
Sabertooth ST2(128, Serial2);


//int theta =60;

USB Usb;
LF310 lf310(&Usb);

void setup() {

  Serial.begin(115200);
  //SabertoothTXPinSerial.begin(9600);
  Serial1.begin(19200);
  Sabertooth::autobaud(Serial1); // Autobaud is for the whole serial line -- you don't need to do

  Serial2.begin(19200);
  Sabertooth::autobaud(Serial2); // Autobaud is for the whole serial line -- you don't need to do

       // Serial.begin(115200);
#if !defined(__MIPSEL__)
        while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
        Serial.println("Starting Logitech F310 gamepad");

        if (Usb.Init() == -1)
                Serial.println("OSC did not start.");
               
        // Set this to higher values to enable more debug information
        // minimum 0x00, maximum 0xff, default 0x80
        // UsbDEBUGlvl = 0xff;

        delay(200);
}

   uint8_t oldX = 128;
   uint8_t oldY = 128;
   uint8_t oldZ = 128;
   uint8_t oldRz = 128;
 
void loop() {
    /*
     * These four variable hold the "old" values of the joysticks so that action can be taken
     * only if they change.
     */
    Usb.Task();
    if (lf310.connected()) {
//   Serial.print("connected: ");
//   Serial.print("x: ");
   Serial.println(lf310.lf310Data.X);
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
            lf310.buttonClickState.Xbutton= 0; // Clear event
            Serial.println(F("X button"));
        }
     
        if (lf310.buttonClickState.Abutton) {
            lf310.buttonClickState.Abutton= 0; // Clear event
            Serial.println(F("A button"));
        }
     
        if (lf310.buttonClickState.Bbutton) {
//            lf310.buttonClickState.Bbutton= 0; // Clear event
            Serial.println(F("B button"));
        }
     
        if (lf310.buttonClickState.Ybutton) {
            lf310.buttonClickState.Ybutton= 0; // Clear event
            Serial.println(F("Y button"));
        }
     
        if (lf310.buttonClickState.LBbutton) {
            lf310.buttonClickState.LBbutton= 0; // Clear event
            Serial.println(F("LB button"));
        }
     
        if (lf310.buttonClickState.RBbutton) {
            lf310.buttonClickState.RBbutton= 0; // Clear event
            Serial.println(F("RB button"));
        }
     
        if (lf310.buttonClickState.LTbutton) {
            lf310.buttonClickState.LTbutton= 0; // Clear event
            Serial.println(F("LT button"));
        }
     
        if (lf310.buttonClickState.RTbutton) {
            lf310.buttonClickState.RTbutton= 0; // Clear event
            Serial.println(F("RT button"));
        }
     
        if (lf310.buttonClickState.Backbutton) {
            lf310.buttonClickState.Backbutton= 0; // Clear event
            Serial.println(F("Back button"));
        }
     
        if (lf310.buttonClickState.Startbutton) {
            lf310.buttonClickState.Startbutton= 0; // Clear event
            Serial.println(F("Start button"));
        }
     
        if (lf310.buttonClickState.LJSP) {
            lf310.buttonClickState.LJSP= 0; // Clear event
            Serial.println(F("Left Joystick deprressed"));
        }
     
        if (lf310.buttonClickState.RJSP) {
            lf310.buttonClickState.RJSP= 0; // Clear event
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

//float d_theta = 30;
//float theta = d_theta*M_PI/180;
//////float theta = 1.0;
//float v = 50;

int maplx =  map(lf310.lf310Data.X, 0, 255, -100, 100);     //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f); JoyEvents.lx;
int maply =  map(lf310.lf310Data.Y, 0, 255, 100, -100);      // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);  JoyEvents.ly;

float theta = atan2(maply,maplx);

float v = sqrt((maplx*maplx)+(maply*maply));


float Vx = v*cos(theta);
float Vy = v*sin(theta);

float VA = -Vx;
float VB = (Vx*sin(0.5236))-(Vy*cos(0.5236));
float VC = (Vx*sin(0.5236))+(Vy*cos(0.5236));

int V1 =  map( VA, -100, 100, -30, 30);   
int V2 =  map(VB, -100, 100, -30, 30); 
int V3 =  map(VC, -100, 100, -30, 30);

Serial.println(V1);

Serial.println(V2);

Serial.println(V3);

ST1.motor(1, V1);    //Rotations    
ST2.motor(2, V2);          
ST2.motor(1, V3);
    }
}
