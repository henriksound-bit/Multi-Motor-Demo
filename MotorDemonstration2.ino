//Programmer: Alex Jones ajones2@howardcc.edu
//Date: 10-18-19
//Program: Multi-motor demonstration with Adafruit Motorshield V1
//Notes: Requires installation of Adafruit Motorshield V1 Library, and RC_ESC librabry by Eric Nantel
//Current Iteration: ALl motors work independently. Stepper, Servo, and Brushless all worked together. Once Brushed motor was added, only that and Servo were active.
//Need to investigate why all 4 won't run in unison. 

#include <AFMotor.h>
#include <Stepper.h>
#include <Servo.h>
#include <ESC.h>

#define LED_PIN (13)                                      // Pin for the LED 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds


//Connects DC Motor to M2
AF_DCMotor motor(2);
//Connects Stepper Motor with 200 steps to motor port #2 (M3 & M4)
AF_Stepper motor1(200, 2);
//Hobby Servo 
Servo servo1;
int pos = 0; //variable to store servo position
//DC Brushed Motor
ESC myESC (10, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC;                                                 // Variable for the speed sent to the ESC


void setup() {

//DC Brushless Motor Begin
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  // turn on motor
  motor.setSpeed(200);
 
  motor.run(RELEASE);
//DC Brushless Motor End

//Stepper Motor Begin
  motor1.setSpeed(10);  // 10 rpm   
//Stepper Motor End

//Servo Motor Begin
servo1.attach(9);
//Servo Motor End

//DC Brushed Motor Begin
pinMode(LED_PIN, OUTPUT);                               // LED Visual Output
  myESC.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed
  delay(5000);  
//DC Brushed Motor End
}

void loop() {

 //DC Brushless Motor Block Begin---------------------------------------------------------------
  uint8_t i;
  
  Serial.print("tick");
  
  motor.run(FORWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);  
    delay(10);
 }
 
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);  
    delay(10);
 }
  
  Serial.print("tock");

  motor.run(BACKWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);  
    delay(10);
 }
 
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);  
    delay(10);
 }
  

  Serial.print("tech");
  motor.run(RELEASE);
  delay(1000);
//DC Brushless Motor Block End--------------------------------------------------------------

//Stepper Motor Block Begin-------------------------------------------------------------------
Serial.println("Single coil steps");
  motor1.step(100, FORWARD, SINGLE); 
  motor1.step(100, BACKWARD, SINGLE); 

  Serial.println("Double coil steps");
  motor1.step(100, FORWARD, DOUBLE); 
  motor1.step(100, BACKWARD, DOUBLE);

  Serial.println("Interleave coil steps");
  motor1.step(100, FORWARD, INTERLEAVE); 
  motor1.step(100, BACKWARD, INTERLEAVE); 

  Serial.println("Micrsostep steps");
  motor1.step(100, FORWARD, MICROSTEP); 
  motor1.step(100, BACKWARD, MICROSTEP); 
//Stepper Motor Block End-------------------------------------------------------------------------

//Servo Motor Block Begin------------------------------------------------------------------
for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
//Servo Motor Block End--------------------------------------------------------------------

//Brushed DC Motor Block Begin-----------------------------------------------------------------------------------
  for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
    myESC.speed(oESC);                                    // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }
  delay(1000);
  for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
    myESC.speed(oESC);                                    // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed  
   }
  delay(500);                                            // Wait for a while befor restart
  //Brushed DC Motor Block End-------------------------------------------------------------------------------------
}
