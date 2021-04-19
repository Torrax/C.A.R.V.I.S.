//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   Author: Carson Morris                                                  //
//                                          Class: ELE8941 Robotics and Controls                                            //
//                                                        Section: 010                                                      //
//                                                      Date: 30/01/2020                                                    //
//                                                         Lab: Robot                                                       //
//                                                          Rev: 2.0                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                          //
//                                                       Material Data                                                      //
//                                                                                                                          //
//         Sensor                Port          Datasheet                                                                    //
//                                                                                                                          //
//          HC-05                              http://www.electronicaestudio.com/docs/istd016A.pdf                          //
//                                                                                                                          //
//         HC-SR04                             https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf             //
//                                                                                                                          //
//         GP2Y0A02                            https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf        //
//                                                                                                                          //
//        IR Sensor                            https://cdn.instructables.com/ORIG/FW9/SBS0/J3EPQTB8/FW9SBS0J3EPQTB8.pdf     //
//                                                                                                                          //
//          Servo                              N/A                                                                          //
//                                                                                                                          //
//           LED                               N/A                                                                          //
//                                                                                                                          //
//         Buzzer                              N/A                                                                          //
//                                                                                                                          //
//         Motors                              N/A                                                                          //
//                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////// INCLUDES //////////////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>  //Software Serial needed for Bluetooth controller (Creates extra RX, TX pins which are not busy while uploading code)
#include <Servo.h> // Servo Library Needed for Servo for Ultrasonic Sensor
#include <SharpIR.h> //Needed for Sharp Distance Sensors
#include <U8glib.h> // Needed for LCD Display
#include <math.h> // Needed for Math Equations

//////////////////////////////////////////////////////////////////// TIMER SETUP //////////////////////////////////////////////////////////////////////////
uint32_t currentMillis; // Variable for Milliseconds
int currentSec; // Variable for Seconds
int currentMin; // Variable for Minutes
unsigned long updatePreviousMillis = 0; // Reset Variable for Refresh Rate of main loop functions

//////////////////////////////////////////////////////////////////// SPEED ADJUSTER //////////////////////////////////////////////////////////////////////////
#define speedPot A12 // Sets Potentiometer input to Pin 12

//////////////////////////////////////////////////////////////////// WHEEL SETUP //////////////////////////////////////////////////////////////////////////
# define motorBR_Forward 49 // Sets Front Left Motor - Forward to Pin 49
# define motorBR_Backward 47 // Sets Front Left Motor - Backward to Pin 47
# define motorBL_Forward 27 // Sets Front Right Motor - Forward to Pin 27
# define motorBL_Backward 29 // Sets Front Right Motor - Backward to Pin 29
# define motorFR_Forward 53 // Sets Back Left Motor - Forward to Pin 53
# define motorFR_Backward 51 // Sets Back Left Motor - Backward to Pin 51
# define motorFL_Forward 25 // Sets Back Right Motor - Forward to Pin 25
# define motorFL_Backward 23 // Sets Back Right Motor - Backward to Pin 23
# define motorFR_EN 6 // Sets Front Right Motor - Enable to Pin 6
# define motorBL_EN 3 // Sets Back Left Motor - Enable to Pin 3
# define motorBR_EN 5 // Sets Back Right Motor - Enable to Pin 5
# define motorFL_EN 4 // Sets Front Left Motor - Enable to Pin 4

int moveSpeed = 200; // How fast the Motors will turn at full speed (PWM Value from 134 - 255)
int turnSpeed = 220; // How fast the Motors will turn at turning speed (PWM Value from 150 - 255)

int runMotor = 1000; // How long motors will run for

//////////////////////////////////////////////////////////////////// LIGHTS & HORN SETUP //////////////////////////////////////////////////////////////////////////
#define rightHeadlight 34 // Sets Right Headlight to Pin 34
#define leftHeadlight 42 // Sets Left Headlight to Pin 42
#define leftBreakLight 28 // Sets Left Break Light to Pin 28
#define leftTurnLight 26 // Sets Left Turn Light to Pin 26
#define rightBreakLight 46 // Sets Right Break Light to Pin 46
#define rightTurnLight 44 // Sets Right Turn Light to Pin 44

const long blinkInterval = 400;  // Interval  that the Left/Right Turn Signals Blink (milliseconds)
int leftSignalState = LOW; //Flag variable for left Signal
unsigned long leftPreviousMillis = 0; // Time varaible for left Signal
int rightSignalState = LOW; //Flag variable for right Signal
unsigned long rightPreviousMillis = 0; // Time varaible for right Signal
unsigned long hazzardPreviousMillis = 0; // Time varaible for left Signal

#define hornPin 24 // Sets Horn to Pin 24

//////////////////////////////////////////////////////////////////// BLUETOOTH SETUP /////////////////////////////////////////////////////////////////////////////
SoftwareSerial BTSerial(10, 11); // Turns pins 10 to RX and 11 to TX for Bluetooth Serial
String BTInput; // Input from Bluetooth Module

//////////////////////////////////////////////////////////////////// IR SENSOR SETUP /////////////////////////////////////////////////////////////////////////////
#define leftIRSensor 32 // Sets Left IR Sensor as Pin 32
#define rightIRSensor 30 // Sets Right IR Sensor as Pin 30

//////////////////////////////////////////////////////////////////// PHOTORESISTOR SETUP //////////////////////////////////////////////////////////////////////////
#define leftPR A10 // Sets Left PR Sensor as Pin 10
#define rightPR A11 // Sets Right PR Sensor as Pin 11

//////////////////////////////////////////////////////////////////// DISTANCE SENSOR SETUP ////////////////////////////////////////////////////////////////////////
#define leftDistanceSensor A13  // Sets Left Distance Sensor as Pin 13
#define rightDistanceSensor A14  // Sets Right Distance Sensor as Pin 14
SharpIR leftSharpSensor (SharpIR::GP2Y0A21YK0F, leftDistanceSensor); // Create and sensor object for Left Sharp IR Sensor
SharpIR rightSharpSensor (SharpIR::GP2Y0A21YK0F, rightDistanceSensor); // Create and sensor object for Right Sharp IR Sensor
double distanceLeft; //Variable for storing distance left sensor measures (cm)
double distanceRight; //Variable for storing distance right sensor measures (cm)

////////////////////////////////////////////////////////////// ULTRASONIC SENSOR / SERVO SETUP ////////////////////////////////////////////////////////////////////
# define servoPin 52 // Sets Servo Pin as Pin 52
# define sonicTrig 50 // Sets Ultra Sonic Sensor Trigger Pin as Pin 48
# define sonicEcho 48 // Sets Ultra Sonic Echo Trigger Pin as Pin 50

Servo headServo; // Initialize servo "headServo" for Ultrasonic Sensor

double distance; // Variable for Distance Calculation

///////////////////////////////////////////////////////////////////////// LCD SETUP ///////////////////////////////////////////////////////////////////////////////
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE); //Create LCD as an U8G Object
#define LCDbutton 2 // Set LCD Front Button to Pin 2 to allow for INTERUPTS
#define leftSelButton 19  // Set LCD Top Left Button to Pin 19 to allow for INTERUPTS
#define rightSelButton 18 // Set LCD Top Right Button to Pin 18 to allow for INTERUPTS
int LCDSel = 1; // Set the LCD to start on Selection Screen 1 (Main Stats)
int mode = 1; // Sets starting program as program 1 (Bluetoth RC Mode)
int speedMode = 1; //Sets starting program to set Move Speed with Potentiometer
unsigned long LCDPreviousMillis = 0; // Time varaible for LCD button Debounce

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MAIN SETUP /////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600); // Serial for Testing
  BTSerial.begin(9600); // Serial for recieving BT data

  ///////////////// Set Motor Forward/Backward Pins as OUTPUTS ////////////////////////////////
  pinMode(motorFL_Forward, OUTPUT);
  pinMode(motorFL_Backward, OUTPUT);
  pinMode(motorFR_Forward, OUTPUT);
  pinMode(motorFR_Backward, OUTPUT);
  pinMode(motorBL_Forward, OUTPUT);
  pinMode(motorBL_Backward, OUTPUT);
  pinMode(motorBR_Forward, OUTPUT);
  pinMode(motorBR_Backward, OUTPUT);

  ///////////////// Set Horn & Light Pins as OUTPUTS ////////////////////////////////
  pinMode(leftHeadlight, OUTPUT);
  pinMode(rightHeadlight, OUTPUT);
  pinMode(leftBreakLight, OUTPUT);
  pinMode(leftTurnLight, OUTPUT);
  pinMode(rightBreakLight, OUTPUT);
  pinMode(rightTurnLight, OUTPUT);

  digitalWrite(leftHeadlight, LOW);
  digitalWrite(rightHeadlight, LOW);
  digitalWrite(leftBreakLight, LOW);
  digitalWrite(leftTurnLight, LOW);
  digitalWrite(rightBreakLight, LOW);
  digitalWrite(rightTurnLight, LOW);

  pinMode (hornPin, OUTPUT);
  digitalWrite (hornPin, LOW);

    ///////////////// IR Sensor Setup ////////////////////////////////
  pinMode(leftIRSensor,INPUT);
  pinMode(rightIRSensor,INPUT);

  ///////////////// Ultra Sonic Sensor Head ////////////////////////////////
  pinMode(sonicEcho, INPUT);  // Register echoPin for receiving input
  pinMode(sonicTrig, OUTPUT);  // Register trigPin for sending output
  pinMode(servoPin,OUTPUT); // Set Servo as an Output
  headServo.attach(servoPin); // Set Servo pin as Pin 31
  headServo.write(90); // Sets Servo back to front facing to start (0 - 180)

    //////////////////////////////////LCD Screen////////////////////////////////////
   u8g.setRot180();   // Flip Screen 180 degrees
   u8g.setFont(u8g_font_unifont); // Set font that will be printer to LCD
   pinMode(LCDbutton, INPUT_PULLUP); // Initialize LCDButton as an Input Interupt
   digitalWrite(LCDbutton, HIGH); // Pull LCDButton HIGH
   attachInterrupt(digitalPinToInterrupt(LCDbutton), selChange, FALLING); // Interupt to Trigger on FALLING edge and will tigger selChange Function

  //////////////////////////////////Selection Menu////////////////////////////////////
  pinMode(leftSelButton, INPUT_PULLUP); // Initialize leftSelButton as an Input Interupt
  digitalWrite(leftSelButton,HIGH); // Pull leftSelButton HIGH
  pinMode(rightSelButton, INPUT_PULLUP); // Initialize rightSelButton as an Input Interupt
  digitalWrite(rightSelButton,HIGH);// Pull rightSelButton HIGH
  attachInterrupt(digitalPinToInterrupt(leftSelButton), leftSelect, FALLING); // Interupt to Trigger on FALLING edge and will tigger leftSelect Function
  attachInterrupt(digitalPinToInterrupt(rightSelButton), rightSelect, FALLING); // Interupt to Trigger on FALLING edge and will tigger rightSelect Function
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MAIN LOOP //////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (millis() - updatePreviousMillis >= 500) { //If current time minus previous time is greater/equal to binkInterval
     updatePreviousMillis = millis();    // save the last time you blinked the LED
     updateTime();
     updateLCD();
     setSpeed();
     nightLights();
  }
  
  if (mode == 1) { //Bluetooth RC
   BTControl();
  } else if (mode == 2) { //Line Follower
    lineFollow();
  } else if (mode == 3) { //Avoidder
   avoider();
  } else if (mode == 4) { //Light Follower
   followLight ();
  } else if (mode == 5) { //Empty
   scanSides();
  } else if (mode == 6) { //Test Mode
   testFunction();
  }
}

  ////////////////////////////////// Timer /////////////////////////////////////
void updateTime() {
  currentMillis = millis();  // Put milliseconds into a variable
  currentSec = currentMillis/1000; // Convert Milliseconds to Seconds
  currentMin = currentSec/60; // Convert Seconds to Minutes
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MOVE FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////// SET SPEED /////////////////////////////////////
void setSpeed() {
  if (speedMode == 1){
  moveSpeed = (analogRead(speedPot)/8*1.20)+134; //Requires a minimum speed of 134 PWM on EN calculated values between 134 - 255
  if (moveSpeed > 255) {  //Make sure PWM value maxes at 255
    moveSpeed = 255;
    }
  //Serial.println(moveSpeed);
  } else  if (speedMode == 2){
  turnSpeed = (analogRead(speedPot)/8*1.20)+134; //Requires a minimum speed of 134 PWM on EN calculated values between 134 - 255
  if (turnSpeed > 255) {  //Make sure PWM value maxes at 255
    turnSpeed = 255;
    }
  //Serial.println(turnSpeed);
  }
}

void stopMotors() {
  ///////////////////////////////////// STOP ////////////////////////////////////////
  // Stop Motors
  digitalWrite(motorFL_Forward, LOW);
  digitalWrite(motorFR_Forward, LOW);
  digitalWrite(motorBL_Forward, LOW);
  digitalWrite(motorBR_Forward, LOW);
  digitalWrite(motorFL_Backward, LOW);
  digitalWrite(motorFR_Backward, LOW);
  digitalWrite(motorBL_Backward, LOW);
  digitalWrite(motorBR_Backward, LOW);
}

void driveForwards() {
  ///////////////////////////////////// MOVE FORWARD ////////////////////////////////////////
  // Front Left Motor  -------- Forwards
  analogWrite(motorFL_EN, moveSpeed);
  digitalWrite(motorFL_Forward, HIGH);
  digitalWrite(motorFL_Backward, LOW);
  // Front Right Motor  -------- Forwards
  analogWrite(motorFR_EN, moveSpeed);
  digitalWrite(motorFR_Forward, HIGH);
  digitalWrite(motorFR_Backward, LOW);
  // Back Left Motor  -------- Forwards
  analogWrite(motorBL_EN, moveSpeed);
  digitalWrite(motorBL_Forward, HIGH);
  digitalWrite(motorBL_Backward, LOW);
  // Back Right Motor  -------- Forwards
  analogWrite(motorBR_EN, moveSpeed);
  digitalWrite(motorBR_Forward, HIGH);
  digitalWrite(motorBR_Backward, LOW);

  delay(runMotor); // Turn the Motors for this long
}

void driveBackwards() {
  ///////////////////////////////////// MOVE BACKWARD ////////////////////////////////////////
  // Front Left Motor  -------- Backwards
  analogWrite(motorFL_EN, moveSpeed);
  digitalWrite(motorFL_Forward, LOW);
  digitalWrite(motorFL_Backward, HIGH);
  // Front Right Motor  -------- Backwards
  analogWrite(motorFR_EN, moveSpeed);
  digitalWrite(motorFR_Forward, LOW);
  digitalWrite(motorFR_Backward, HIGH);
  // Back Left Motor  -------- Backwards
  analogWrite(motorBL_EN, moveSpeed);
  digitalWrite(motorBL_Forward, LOW);
  digitalWrite(motorBL_Backward, HIGH);
  // Back Right Motor  -------- Backwards
  analogWrite(motorBR_EN, moveSpeed);
  digitalWrite(motorBR_Forward, LOW);
  digitalWrite(motorBR_Backward, HIGH);

  delay(runMotor); // Turn the Motors for this long   
}

void driveLeft() {
  ///////////////////////////////////// TURN LEFT ////////////////////////////////////////
  // Front Left Motor  -------- Backwards
  analogWrite(motorFL_EN, turnSpeed);
  digitalWrite(motorFL_Forward, LOW);
  digitalWrite(motorFL_Backward, HIGH);
  // Front Right Motor  -------- Forwards
  analogWrite(motorFR_EN, turnSpeed);
  digitalWrite(motorFR_Forward, HIGH);
  digitalWrite(motorFR_Backward, LOW);
  // Back Left Motor  -------- Backwards
  analogWrite(motorBL_EN, turnSpeed);
  digitalWrite(motorBL_Forward, LOW);
  digitalWrite(motorBL_Backward, HIGH);
  // Back Right Motor  -------- Forwards
  analogWrite(motorBR_EN, turnSpeed);
  digitalWrite(motorBR_Forward, HIGH);
  digitalWrite(motorBR_Backward, LOW);

  delay(runMotor); // Turn the Motors for this long
}

void driveRight() {
  ///////////////////////////////////// RIGHT TURN ////////////////////////////////////////
  // Front Left Motor  -------- Forwards
  analogWrite(motorFL_EN, turnSpeed);
  digitalWrite(motorFL_Forward, HIGH);
  digitalWrite(motorFL_Backward, LOW);
  // Front Right Motor  -------- Backwards
  analogWrite(motorFR_EN, turnSpeed);
  digitalWrite(motorFR_Forward, LOW);
  digitalWrite(motorFR_Backward, HIGH);
  // Back Left Motor  -------- Forwards
  analogWrite(motorBL_EN, turnSpeed);
  digitalWrite(motorBL_Forward, HIGH);
  digitalWrite(motorBL_Backward, LOW);
  // Back Right Motor  -------- Backwards
  analogWrite(motorBR_EN, turnSpeed);
  digitalWrite(motorBR_Forward, LOW);
  digitalWrite(motorBR_Backward, HIGH);

  delay(runMotor); // Turn the Motors for this long
}

void driveFRight() {
  ////////////////////////////////// FORWARD RIGHT TURN /////////////////////////////////////
  // Front Left Motor  -------- Forwards
  analogWrite(motorFL_EN, turnSpeed);
  digitalWrite(motorFL_Forward, HIGH);
  digitalWrite(motorFL_Backward, LOW);
  // Front Right Motor  -------- Backwards
  analogWrite(motorFR_EN, turnSpeed);
  digitalWrite(motorFR_Forward, LOW);
  digitalWrite(motorFR_Backward, LOW);
  // Back Left Motor  -------- Forwards
  analogWrite(motorBL_EN, turnSpeed);
  digitalWrite(motorBL_Forward, HIGH);
  digitalWrite(motorBL_Backward, LOW);
  // Back Right Motor  -------- Backwards
  analogWrite(motorBR_EN, turnSpeed);
  digitalWrite(motorBR_Forward, LOW);
  digitalWrite(motorBR_Backward, LOW);

  delay(runMotor); // Turn the Motors for this long
}

void driveFLeft() {
  ////////////////////////////////// FORWARD LEFT TURN /////////////////////////////////////
  // Front Left Motor  -------- Forwards
  analogWrite(motorFL_EN, turnSpeed);
  digitalWrite(motorFL_Forward, LOW);
  digitalWrite(motorFL_Backward, LOW);
  // Front Right Motor  -------- Backwards
  analogWrite(motorFR_EN, turnSpeed);
  digitalWrite(motorFR_Forward, HIGH);
  digitalWrite(motorFR_Backward, LOW);
  // Back Left Motor  -------- Forwards
  analogWrite(motorBL_EN, turnSpeed);
  digitalWrite(motorBL_Forward, LOW);
  digitalWrite(motorBL_Backward, LOW);
  // Back Right Motor  -------- Backwards
  analogWrite(motorBR_EN, turnSpeed);
  digitalWrite(motorBR_Forward, HIGH);
  digitalWrite(motorBR_Backward, LOW);

  delay(runMotor); // Turn the Motors for this long
}

void driveBRight() {
  ////////////////////////////////// BACK RIGHT TURN /////////////////////////////////////
  // Front Left Motor  -------- Forwards
  analogWrite(motorFL_EN, turnSpeed);
  digitalWrite(motorFL_Forward, LOW);
  digitalWrite(motorFL_Backward, HIGH);
  // Front Right Motor  -------- Backwards
  analogWrite(motorFR_EN, turnSpeed);
  digitalWrite(motorFR_Forward, LOW);
  digitalWrite(motorFR_Backward, LOW);
  // Back Left Motor  -------- Forwards
  analogWrite(motorBL_EN, turnSpeed);
  digitalWrite(motorBL_Forward, LOW);
  digitalWrite(motorBL_Backward, HIGH);
  // Back Right Motor  -------- Backwards
  analogWrite(motorBR_EN, turnSpeed);
  digitalWrite(motorBR_Forward, LOW);
  digitalWrite(motorBR_Backward, LOW);

  delay(runMotor); // Turn the Motors for this long
}

void driveBLeft() {
  ////////////////////////////////// BACK LEFT TURN /////////////////////////////////////
  // Front Left Motor  -------- Forwards
  analogWrite(motorFL_EN, turnSpeed);
  digitalWrite(motorFL_Forward, LOW);
  digitalWrite(motorFL_Backward, LOW);
  // Front Right Motor  -------- Backwards
  analogWrite(motorFR_EN, turnSpeed);
  digitalWrite(motorFR_Forward, LOW);
  digitalWrite(motorFR_Backward, HIGH);
  // Back Left Motor  -------- Forwards
  analogWrite(motorBL_EN, turnSpeed);
  digitalWrite(motorBL_Forward, LOW);
  digitalWrite(motorBL_Backward, LOW);
  // Back Right Motor  -------- Backwards
  analogWrite(motorBR_EN, turnSpeed);
  digitalWrite(motorBR_Forward, LOW);
  digitalWrite(motorBR_Backward, HIGH);

  delay(runMotor); // Turn the Motors for this long
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MISC FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////// HORN /////////////////////////////////////
void soundHorn() {
digitalWrite (hornPin, HIGH);
delay (250);
digitalWrite (hornPin, LOW);
delay (30);
digitalWrite (hornPin, HIGH);
delay (200);
digitalWrite (hornPin, LOW);
delay (500);
}

  ////////////////////////////////// LIGHTS /////////////////////////////////////
void headlightsON() {
  digitalWrite(rightHeadlight, HIGH);
  digitalWrite(leftHeadlight, HIGH);
}

void headlightsOFF() {
  digitalWrite(rightHeadlight, LOW);
  digitalWrite(leftHeadlight, LOW);
}

void nightLights() {
if (analogRead(leftPR) < 100) {
  headlightsON();
  } else {
  headlightsOFF();
    }
}

void breakSignalON() {
  digitalWrite(rightBreakLight, HIGH);
  digitalWrite(leftBreakLight, HIGH);
}

void breakSignalOFF() {
  digitalWrite(rightBreakLight, LOW);
  digitalWrite(leftBreakLight, LOW);
}

void rightTurnSignal() {
    digitalWrite(leftTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
    
    if (millis() - rightPreviousMillis >= blinkInterval) { //If current time minus previous time is greater/equal to binkInterval
    rightPreviousMillis = millis();     //Save the last time you blinked the LED

    //If the LED is off turn it on and vice-versa:
    if (rightSignalState == LOW) {
      rightSignalState = HIGH;
    } else {
      rightSignalState = LOW;
    }
    
    digitalWrite(rightTurnLight, rightSignalState); //Set the LED the same state as the flag
  }
}

void leftTurnSignal() {
  digitalWrite(rightTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
  if (millis() - leftPreviousMillis >= blinkInterval) { //If current time minus previous time is greater/equal to binkInterval
    leftPreviousMillis = millis();    //Save the last time you blinked the LED

    //If the LED is off turn it on and vice-versa:
    if (leftSignalState == LOW) {
      leftSignalState = HIGH;
    } else {
      leftSignalState = LOW;
    }

    digitalWrite(leftTurnLight, leftSignalState); //Set the LED the same state as the flag
  }
}

void hazzardSignal() {
  if (millis() - hazzardPreviousMillis >= blinkInterval) { //If current time minus previous time is greater/equal to binkInterval
    hazzardPreviousMillis = millis();    //Save the last time you blinked the LEDs

    //If the LED is off turn it on and vice-versa:
    if (leftSignalState == LOW) {
      leftSignalState = HIGH;
      rightSignalState = HIGH;
    } else {
      leftSignalState = LOW;
      rightSignalState = LOW;
    }

    //Set the LED with the ledState of the variable:
    digitalWrite(leftTurnLight, leftSignalState);
    digitalWrite(rightTurnLight, rightSignalState);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// PROGRAMS ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// BLUETOOTH CONTROL ////////////////////////////////////////
void BTControl() {
  if (BTSerial.available() > 0) { //If something is recieved from the BT module
    BTInput = BTSerial.read(); //Record the input to a variable
    Serial.print(BTInput); // Print the input recieved to the Test Console
  }
  if (BTInput == "70") { //If 70 is recieved from the BT module (FORWARDS)
    breakSignalOFF();
    digitalWrite(leftTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
    digitalWrite(rightTurnLight, LOW); //Turns Right Turn Signal off if it was left on
    runMotor = 200; // How long to move for
    driveForwards();
    Serial.println(" = Moving Forward");
  } else if (BTInput == "66") { //If 66 is recieved from the BT module (BACKWARDS)
    digitalWrite(leftTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
    digitalWrite(rightTurnLight, LOW); //Turns Right Turn Signal off if it was left on
    breakSignalON();
    hazzardSignal();
    runMotor = 200; // How long to move for
    driveBackwards();
    Serial.println(" = Moving Backwards");
  } else if (BTInput == "76") { //If 76 is recieved from the BT module (LEFT)
    leftTurnSignal();
    breakSignalOFF();
    runMotor = 200; // How long to move for
    driveLeft();
    Serial.println(" = Moving Left");
  } else if (BTInput == "82") { //If 82 is recieved from the BT module (RIGHT)
    rightTurnSignal();
    breakSignalOFF();
    runMotor = 200; // How long to move for
    driveRight();
    Serial.println(" = Moving Right");
  } else if (BTInput == "73") { //If 118 is recieved from the BT module (FORWARD RIGHT)
    rightTurnSignal();
    breakSignalOFF();
    runMotor = 200; // How long to move for
    driveFRight();
    Serial.println(" = Moving Front Right");
  } else if (BTInput == "71") { //If 118 is recieved from the BT module (FORWARD LEFT)
    leftTurnSignal();
    breakSignalOFF();
    runMotor = 200; // How long to move for
    driveFLeft();
    Serial.println(" = Moving Front Left");
  } else if (BTInput == "74") { //If 118 is recieved from the BT module (BACK RIGHT)
    rightTurnSignal();
    breakSignalON();
    runMotor = 200; // How long to move for
    driveBRight();
    Serial.println(" = Moving Back Right");
  } else if (BTInput == "72") { //If 118 is recieved from the BT module (BACK LEFT)
    leftTurnSignal();
    breakSignalON();
    runMotor = 200; // How long to move for
    driveBLeft();
    Serial.println(" = Moving Back Left");
  } else if (BTInput == "48") { //If 48 is recieved from the BT module (SPEED 1)
    moveSpeed = 134; // Set Speed to 105 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 1");
  } else if (BTInput == "49") { //If 49 is recieved from the BT module (SPEED 2)
    moveSpeed = 142; // Set Speed to 120 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 2");
  } else if (BTInput == "50") { //If 50 is recieved from the BT module (SPEED 3)
    moveSpeed = 155; // Set Speed to 135 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 3");
  } else if (BTInput == "51") { //If 51 is recieved from the BT module (SPEED 4)
    moveSpeed = 162; // Set Speed to 150 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 4");
  } else if (BTInput == "52") { //If 52 is recieved from the BT module (SPEED 5)
    moveSpeed = 175; // Set Speed to 180 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 5");
  } else if (BTInput == "53") { //If 53 is recieved from the BT module (SPEED 6)
    moveSpeed = 187; // Set Speed to 195 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 6");
  } else if (BTInput == "54") { //If 54 is recieved from the BT module (SPEED 7)
    moveSpeed = 200; // Set Speed to 215 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 7");
  } else if (BTInput == "55") { //If 55 is recieved from the BT module (SPEED 8)
    moveSpeed = 212; // Set Speed to 235 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 8");
  } else if (BTInput == "56") { //If 56 is recieved from the BT module (SPEED 9)
    moveSpeed = 225; // Set Speed to 255 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 9");
  } else if (BTInput == "57") { //If 57 is recieved from the BT module (SPEED 10)
    moveSpeed = 237; // Set Speed to 255 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 10");
  } else if (BTInput == "113") { //If 113 is recieved from the BT module (SPEED 11)
    moveSpeed = 255; // Set Speed to 255 (0 - 255 PWM Signal)
    Serial.println(" = Speed Set to 11");
  } else if (BTInput == "118") { //If 118 is recieved from the BT module (HORN)
    soundHorn();
    Serial.println(" = Sounding Horn");
  } else if (BTInput == "85") { //If 85 is recieved from the BT module (BREAK SIGNAL ON)
    breakSignalON();
    Serial.println(" = Turning On Break Lights");
  } else if (BTInput == "117") { //If 117 is recieved from the BT module (BREAK SIGNAL OFF)
    breakSignalOFF();
    Serial.println(" = Turning Off Break Lights");
  } else if (BTInput == "87") { //If 87 is recieved from the BT module (HEADLIGHTS ON)
    headlightsON();
    Serial.println(" = Turning On Head Lights");
  } else if (BTInput == "119") { //If 119 is recieved from the BT module (HEADLIGHTS OFF)
    headlightsOFF();
    Serial.println(" = Turning Off Head Lights");
  } else if (BTInput == "88") { //If 88 is recieved from the BT module (HAZARD SIGNAL ON)
    hazzardSignal();
    Serial.println(" = Turning On Hazzards");
  } else if (BTInput == "83") { //If 83 is recieved from the BT module (COASTING)
    digitalWrite(leftTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
    digitalWrite(rightTurnLight, LOW); //Turns Right Turn Signal off if it was left on
    breakSignalON();
    stopMotors();
    } else {
      }
}

///////////////////////////////////// LINE FOLLOW ////////////////////////////////////////
void lineFollow() {
  moveSpeed = 155; // Set how fast Robot will move forward ---------- POT CONTROLLED ------------
  turnSpeed = 220; // Set how fast Robot will turn ---------- POT CONTROLLED ------------
  if (digitalRead (leftIRSensor) == 1) { // If something is detected by the Left IR Sensor
    rightTurnSignal();
    Serial.println("Object Detected by Left IR Sensor"); //Print test message
    runMotor = 400; // How long to move for
    driveLeft(); //Drive Left
  } else   if (digitalRead (rightIRSensor) == 1) { // If something is detected by the Right IR Sensor
    leftTurnSignal();
    Serial.println("Object Detected by Right IR Sensor"); //Print test message
    runMotor = 400; // How long to move for
    driveRight();//Drive Right
  } else   if (digitalRead (rightIRSensor) == 1 and digitalRead (leftIRSensor) == 1) { // If something is detected by the Left & Right IR Sensor
    hazzardSignal();
    Serial.println("Object Detected by BOTH Sensors"); //Print test message
    runMotor = 600; // How long to move for
    driveBackwards();//Drive Backwards
  } else {
    digitalWrite(leftTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
    digitalWrite(rightTurnLight, LOW); //Turns Right Turn Signal off if it was left on
    runMotor = 5; // How long to move for
    driveForwards();//Drive Forwards
    }
}

///////////////////////////////////// AVOIDDER ////////////////////////////////////////
void avoider() {
   moveSpeed = 180; // Set how fast Robot will move forward ---------- POT CONTROLLED ------------
   turnSpeed = 230; // Set how fast Robot will move forward ---------- POT CONTROLLED ------------
  sonicScan(); //Scan Ultra Sonic Sensor in Current Direction
        Serial.println(distance);
  if (distance > 10) { // If there is more than 10 cm infront of car
    runMotor = 5; // How long to move for
    driveForwards();
    digitalWrite(leftTurnLight, LOW);  //Turns Left Turn Signal off if it was left on
    digitalWrite(rightTurnLight, LOW); //Turns Right Turn Signal off if it was left on
    breakSignalOFF();
  } else { // If there is less than 10 cm infront of car
    runMotor = 200; // How long to move for
    driveBackwards();
    stopMotors();
    breakSignalON();
    scanSides();
    if ((distanceRight > 75) and(distanceLeft > 75)) { //If there is at least 50 cm on the Left and the Right
      if (distanceRight > distanceLeft) { // If there is more room on the right than on the left
        runMotor = 1250; // How long to move for
        driveRight();
      } else { // If there is more room on the left than on the right
        runMotor = 1250; // How long to move for
        driveLeft();
      }
    } else if (distanceRight > 55) {
      runMotor = 1250; // How long to move for
      driveRight();
    } else if (distanceLeft > 55) {
      runMotor = 1250; // How long to move for
      driveLeft();
    } else {
      ///////////////////////// SCAN LEFT ///////////////////////////////
      headServo.write(0); // Sets Servo to face right
      delay(500); //Give Servo Time to turn
      
      sonicScan(); //Runs ultrasonic scan function
      distanceRight = distance; // Set Distance to Right to Sonic Scan Value
      Serial.println(distance); //Print distance to Console in cm

      ///////////////////////// SCAN RIGHT ///////////////////////////////
      headServo.write(180); // Sets Servo back to face left
      delay(500); //Give Servo Time to turn
      Serial.print("Distance to Left: ");

      sonicScan(); //Runs ultrasonic scan function
      distanceLeft = distance; // Set Distance to Left to Sonic Scan Value
      Serial.println(distance); //Print distance to Console in cm

      headServo.write(90); // Sets Servo back to front facing to start
      delay(500); //Give Servo Time to turn

      if (distanceRight > distanceLeft) { // If there is more room on the right than on the left
        runMotor = 1250; // How long to move for
        breakSignalOFF();
        driveRight();
      } else { // If there is more room on the left than on the right
        runMotor = 1250; // How long to move for
        breakSignalOFF();
        driveLeft();
      }
    }
  }
}

void sonicScan(){
  digitalWrite(sonicTrig, LOW); // Send a short low pulse to ensure a clean high one.
  delayMicroseconds(2);

  digitalWrite(sonicTrig, HIGH); // Send a ten-second high pulse.
  delayMicroseconds(10);
  digitalWrite(sonicTrig, LOW);

  const long duration = pulseIn(sonicEcho, HIGH); // Store the high pulse's duration.
  distance = microsecondsToDistance(duration); // Calculate and print the distance to the target.
}

const double microsecondsToDistance(const long microseconds) { //Converts Ping into Distance
  
// Initialize m and b to their respective values in the formula, y = mx + b.  
// y = distance       x = time (in microseconds)
const double m = 0.0171;
const double b = 0.2461;

return m * microseconds + b;

}

void scanSides(){
distanceLeft = leftSharpSensor.getDistance(); //Check current distance reading on left distance sensor
distanceRight = rightSharpSensor.getDistance(); //Check current distance reading on right distance sensor
distanceLeft = 1.1682 * distanceLeft + 18.738; // Calculate actual values based on calibration for sensor
distanceRight = 1.2074 * distanceRight + 15.74; // Calculate actual values based on calibration for sensor

//Serial.print("Left IR Sensor = ");
//Serial.print(distanceLeft);
//Serial.println(" cm");
Serial.print("Right IR Sensor = ");
Serial.print(distanceRight);
Serial.println(" cm");
delay(500);
}

  ////////////////////////////////// Light Follower /////////////////////////////////////
void followLight() {
//////////////////////////////////////////////
// DARK = LESS THAN 130
// HEADLIGHTS = LESS THAN 200
// ROOM LIGHT = 130 - 400
// 50cm = 400- 550
// 20cm = 550 - 750
// 10cm = 750 - 900
// >10cm = 900+
/////////////////////////////////////////////
  //moveSpeed = 200; // Set Speed to 200 (0 - 255 PWM Signal) ---------- POT CONTROLLED -------------
  turnSpeed = 200; // Set Speed to 220 (0 - 255 PWM Signal)
  runMotor = 20; // How long to move for

  int roomLight = 450; // Variable for how bright it is usually in the room
  
Serial.print("Left: ");
Serial.println(analogRead(leftPR));
Serial.print("Right: ");
Serial.println(analogRead(rightPR));
Serial.print("Difference: ");
Serial.println(analogRead(leftPR) - analogRead(rightPR));
delay(500);

  if ((analogRead(leftPR) > roomLight) or (analogRead(rightPR) > roomLight)) { // If the light on the Left sensor is brighter than Room Light
    if ((analogRead(leftPR) - analogRead(rightPR) < -150)) { //If the Right sensor has more light
      rightTurnSignal();
      driveRight();
    } else if ((analogRead(leftPR) - analogRead(rightPR) > 150)) { // If the Left sensor has more light
      leftTurnSignal();
      driveLeft();
    } else { // Otherwise the light is comming from infront
      driveForwards();
      digitalWrite(rightTurnLight, LOW);
      digitalWrite(leftTurnLight, LOW);
    }
  } else { //Otherwise light is just from room light
    stopMotors();
    digitalWrite(rightTurnLight, LOW);
    digitalWrite(leftTurnLight, LOW);
  }
}

  ////////////////////////////////// Test Function /////////////////////////////////////
void testFunction() {

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// LCD /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////// LCD Screen Buttons /////////////////////////////////////
void selChange() {
  if (millis() - LCDPreviousMillis >= 100) { //If current time minus previous time is greater/equal to 100 ms (DEBOUNCE)
    LCDPreviousMillis = millis(); //Save as new last time button has been pressed

    //If the LED is off turn it on and vice-versa:
    if (digitalRead(LCDbutton) == LOW) {
      if (LCDSel == 1) { //If selection is currently 1
        LCDSel = 2; //Make it 2
      } else if (LCDSel == 2) { //If selection is currently 2
        LCDSel = 1; //Make it 3
      } else if (LCDSel == 3) { //If selection is currently 3
        LCDSel = 1; //Make it 1 //////////TO BE FIXED///////////////
      } else {}
    }
  }
  updateLCD(); //Update the screen to the new menu
}

void leftSelect() {
  if (millis() - LCDPreviousMillis >= 100) { //If current time minus previous time is greater/equal to 100 ms (DEBOUNCE)
    LCDPreviousMillis = millis(); //Save as new last time button has been pressed
    //If the LED is off turn it on and vice-versa:
    if (digitalRead(leftSelButton) == LOW) {
      if (LCDSel == 1) { // If it is currently on the Stats Menu
        if (speedMode == 1) {
          speedMode = 2;
        } else if (speedMode == 2) {
          speedMode = 1;
          }
      } else if (LCDSel == 2) { // If it is currently on the Mode Menu
        mode--;
        if (mode < 1) {
          mode = 6;
        }
      } else if (LCDSel == 3) { // If it is currently on the Battery Menu

      }
    }
  }
  stopMotors(); //Stop all previous movement
  headServo.write(90); // Sets Servo back to front facing
  updateLCD(); //Update the screen to the new menu
}

void rightSelect() {
  if (millis() - LCDPreviousMillis >= 100) { //If current time minus previous time is greater/equal to 100 ms (DEBOUNCE)
    LCDPreviousMillis = millis(); //Save as new last time button has been pressed
    //If the LED is off turn it on and vice-versa:
    if (digitalRead(rightSelButton) == LOW) {
      if (LCDSel == 1) { // If it is currently on the Stats Menu
        if (speedMode == 1) {
          speedMode = 2;
        } else if (speedMode == 2) {
          speedMode = 1;
          }
      } else if (LCDSel == 2) { // If it is currently on the Mode Menu
        mode++;
        if (mode > 6) {
          mode = 1;
        }
      } else if (LCDSel == 3) { // If it is currently on the Battery Menu
      }
      }
  }
  stopMotors(); //Stop all previous movement
  headServo.write(90); // Sets Servo back to front facing
  updateLCD(); //Update the screen to the new menu
}

  ////////////////////////////////// LCD Screen Updater /////////////////////////////////////
void updateLCD() {
  u8g.firstPage();  //Print page
  do {
    if (LCDSel == 1) { // If current menu mode is Car Stats
    printStats();
    } else if (LCDSel == 2) { // If current menu mode is Run Mode
    printMode();
    } else if (LCDSel == 3) { // If current menu mode is Battery Stats
    printBattery();
    }
  } while( u8g.nextPage() ); //While there is another page
}

  ////////////////////////////////// LCD Stats Print /////////////////////////////////////
void printStats() {
  u8g.setPrintPos(0,10);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  if (speedMode == 1) {
  u8g.print("Speed: ");  // Print Speed: to LCD
  u8g.setPrintPos(50,10);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  if (moveSpeed == 134) { // If move speed is at minimum
    u8g.print("Min");  // Print Min to LCD
  } else { // If move speed is above min
    u8g.print(round((moveSpeed-134)/1.21));  // Print current percentage of max speed to LCD
    u8g.setPrintPos(75,10);  // Choose Position to start drawing from (0 - 128, 0 - 32)
    u8g.print("%");  // Print % to LCD
  }
  } else if (speedMode == 2) {
    u8g.print("Turn Speed: ");  // Print Speed: to LCD
    u8g.setPrintPos(90,10);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  if (turnSpeed == 134) { // If move speed is at minimum
    u8g.print("Min");  // Print Min to LCD
  } else { // If move speed is above min
    u8g.print(round((turnSpeed-134)/1.21));  // Print current percentage of max speed to LCD
    u8g.setPrintPos(115,10);  // Choose Position to start drawing from (0 - 128, 0 - 32)
    u8g.print("%");  // Print % to LCD
    }
    }
  u8g.setPrintPos(0,21);   // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("Rotation:");  // Print Rotation to LCD
  u8g.setPrintPos(75,21);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("90 <");  // Print 90 < to LCD
  u8g.setPrintPos(0,32);   // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("Run Time: ");  // Print Run Time: to LCD
  u8g.setPrintPos(75,32);   // Choose Position to start drawing from (0 - 128, 0 - 32)
  if (currentSec >= 60) { // If over 60 seconds have passed
      u8g.print(currentMin);  // Print current amount of minutes to LCD
      u8g.setPrintPos(95,32);   // Choose Position to start drawing from (0 - 128, 0 - 32)
      u8g.print("min");  // Print min to LCD
  } else { // If less than 60 seconds have passed
      u8g.print(currentSec);  // Print current amount of seconds to LCD
      u8g.setPrintPos(95,32);   // Choose Position to start drawing from (0 - 128, 0 - 32)
      u8g.print("sec");  // Print sec to LCD
  }
}

  ////////////////////////////////// LCD Mode Print /////////////////////////////////////
void printMode() {
  u8g.setPrintPos(0,15);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("Current Mode:");  // Print Current Mode: to LCD
  
  u8g.setPrintPos(0,28);   // Choose Position to start drawing from (0 - 128, 0 - 32)
  if (mode == 1) { //If Bluetooth Mode is Active
    u8g.print("Bluetooth RC");  // Print Bluetooth RC to LCD
  } else if (mode == 2) { //If Line Follower Mode is Active
    u8g.print("Line Follower");  // Print Line Follower to LCD
  } else if (mode == 3) { //If Avoidder Mode is Active
    u8g.print("Avoidder");  // Print Avoider to LCD
  } else if (mode == 4) { //If Light Follower Mode is Active
    u8g.print("Light Follower");  // Print Line Follower to LCD
  } else if (mode == 5) { //If Empty Mode is Active
    u8g.print("Empty");  // Print Empty to LCD
  } else if (mode == 6) { //If Test Mode is Active
    u8g.print("Test Mode");  // Print Test Mode to LCD   
  }
}

  ////////////////////////////////// LCD Battery Print /////////////////////////////////////
void printBattery() {
  u8g.setPrintPos(0,15);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("Battery: ");  // Print Battery to LCD
  u8g.setPrintPos(0,28);  // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("Voltage: ");  // Print Voltage: to LCD
  u8g.setPrintPos(70,28);   // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("8.2");  // Print 8.2 to LCD
  u8g.setPrintPos(105,28);   // Choose Position to start drawing from (0 - 128, 0 - 32)
  u8g.print("V");  // Print V to LCD
}
