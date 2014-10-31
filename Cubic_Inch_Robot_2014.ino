// ================================================================
// ===         2014 Waiakea Robotics Cubic Inch Robot           ===
// ================================================================
/*

5/23/2014

The MPU6050_DMP6 example was used as the starting point for this program 

Final goal is to navigate a fixed grid maze using the gyroscope sensor and front IR sensors to detect gaps in the "maze"
The "maze" is not a real maze but actually a grid of blocks arranged in a 3 x 3 pattern with each block being 150mm square and a 30mm gap between them.
The goal is to drive over the 7 control points. Further details are here: http://imd.eng.kagawa-u.ac.jp/maze/reg_c2.html
The difficulty is doing this with a robot fitting in 1 cubic inch and also doing it faster than all of your opponents as the fastest robot wins. 

There is also a human driver controlled category in the competition, so this program also will have a user operated mode.
Autonomous mode will most likely be activated by pushing a certain button on the remote control. 

The remote control has the same 2.4ghz transceiver connected to the SPI port so they can communicate both ways.  
The remote contains 8 pushbuttons, arranged in two "D" pads one on the left and one on the right.
It also contains a 128x64 pixel OLED display wired to the I2C port. 

Programming both the robot and the remote is facilitated by a USB to serial converter IC located on the remote PCB. 
There is a switch on the remote to select whether the serial port goes to the robot or the remote. 
The robot is physically connected to the remote control for programming and charging via a magnetic pogo pin connector for ease of use. 

Possibly the remote will also have a serial passthrough mode to allow the computers serial terminal to communicate with the robot remotely.
The other option for reading real time data from the robot is to use the OLED display on the remote as a display terminal. 

Both the robot and remote have 2:1 voltage dividers to allow battery voltage monitoring in real time. 

They are both powered by lithium polymer recharable batteries. 
The remote contains chargers for both batteries that are powered through the same USB port used for programming. 

The locally included PID library has been modified to allow its use with continuous rotation inputs that roll over. 
IE with a gyro input from 0 - 360 that will rollover from 360 back to 0 if it is turned further to the right.
The library had to be modified so it treats 0 and 360 as the same value because it may be faster to go upwards from say 300 degrees to get to 10 degrees 
versus going down all the way to 10 if it was not aware of the loopover effect of absolute angular data. 


*/
// ================================================================
// ===                       Gyro Includes                      ===
// ================================================================

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// specific I2C addresses may be passed as a parameter here
// MPU6050 PIN AD0 low = 0x68 (default)
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * =====================================
 ==================================== */

// MPU6050 Gyro control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Interrupt routine for Gyro
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
  mpuInterrupt = true;
}

// ================================================================
// ===               2.4Ghz Transceiver Includes                ===
// ================================================================

#include <SPI.h>  // Library for SPI communications used by the nRF24L01 radio

#include <RH_NRF24.h>
RH_NRF24 nrf24(8, 14); //CE, CSN

// ================================================================
// ===                    PID Library Includes                  ===
// ================================================================
#include "PID_CL_2014.h"

//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,.5,0,0, REVERSE);
// Last input "DIRECT" or "REVERSE" will change which way the correction value goes
//switch them if the correction  makes things worse

// ================================================================
// ===                    Robot Pin Defines                     ===
// ================================================================

// Lets define some nice handy constants eh?

#define LED_R 13 // Red LED - Shared with SCK used for NRF24L01 Transceiver
#define LED_G 0  // Green LED - Shared with serial port radio pin
#define LED_B 4  // Blue LED

#define LED_ON 0 // LEDs are active LOW
#define LED_OFF 1 // LEDs are active LOW

//Pull these pins high to enable a specific LED
//Cathode is connected to 38khz pin

#define IR_LED_R 1 // IR LED Left used for reflective wall sensing
#define IR_LED_L 7 // IR LED Left used for reflective wall sensing
#define IR_38Khz 3    // IR LED 38khz Cathode connection

#define IR_SENSOR_R 15 // Right input from 38khz bandpass filter connected to IR PIN diode
#define IR_SENSOR_L 16 // 

#define WALL_DETECTED 0 // Wall detected
#define WALL_NOT_DETECTED 1 // Wall not detected


#define MOTOR_R_DIR 5 // Right motor direction pin
#define MOTOR_R_SPD 9 // Right motor speed pin - apply PWM signal (analog out) to this pin
#define MOTOR_L_DIR 6
#define MOTOR_L_SPD 10

#define BATT_VOLTAGE 17 //Battery voltage monitor pin - connected to 50% divider to allow the measurment of voltages higher than the vcc of 3.3v

#define FWD 0 // 0 = forward in our robot wiring
#define BWD 1 // 1 = backward in our robot wiring

// The below defines are for the bit location of the corresponding buttons in our 8 bit encoded buttons variable received from the transmitter
#define A 0  // Right D pad up button
#define B 1  // Right D pad right button
#define C 2  // Right D pad down button
#define D 3  // Right D pad left button

#define UP 4    // Left D pad up button
#define RIGHT 5 // Left D pad right button
#define DOWN 6  // Left D pad down button
#define LEFT 7  // Left D pad left button

// ================================================================
// ===                  Variable Definitions                    ===
// ================================================================

uint8_t sendBuffer[7];  // 28 element array of unsigned 8-bit type - 28 is the max message length for the nrf24L01 radio
uint8_t receiveBuffer[2];
uint8_t lengthReceive = sizeof(receiveBuffer);

//String message; // Used by radio code - may not be final
unsigned char buttons; // value of the buttons received from the remote
unsigned char buffer; // receive variable
unsigned char bufferLast; // last received variable

int receiveCheck;

int iteration=0;

bool blinkState = false;
bool blinkState1 = false;
bool blinkState2 = false;

bool startAButton;
bool startBButton;
bool startCButton;
bool startDButton;

int yaw = 0;
int yawLast = 0;
int yawContinuous = 0;
int startYawContinuous = 0;
int yawRotationCount = 0;
int battVoltage;
//int yawStart = 0;
//int yawDiff = 0;

int outputInt;

int forwardRamp;
int forwardRampD;
int forwardRampAuto;
int loopTimer;

int slowTimer;

int stateMachine;

unsigned long lastMillis, timeAway;
unsigned long lastMillisAuto;
unsigned long lastMillisGyro, timeAwayGyro;

unsigned char sendCounter;

// ================================================================
// ===                  SOFTWARE MOD FUNCTION                   ===
// ================================================================

int smod(int z1, int z2) { // Software MOD function
  int ze;
  ze=z1 % z2;
  if (ze>=0)  {
    return(ze);
  }
  else{
    return(z2+ze);
  }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
//  Serial.begin(115200); // Serial turned off because Green LED uses same pin as RX, IR LED Right uses same pin as TX
    
// ================================================================
// ===                        GYRO SETUP                        ===
// ================================================================
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 2; //2 = 800khz I2C - fastest possible data rate
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(220); // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  if (devStatus == 0) { // make sure it worked (returns 0 if so)
    mpu.setDMPEnabled(true);  // turn on the DMP, now that it's ready
  
    attachInterrupt(0, dmpDataReady, RISING); // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    
    dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
    
    packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
  }

// ================================================================
// ===                     Robot Pin Setup                      ===
// ================================================================
  
  pinMode(LED_G, OUTPUT); // configure LED for output
  pinMode(LED_B, OUTPUT);
  
  digitalWrite(LED_G, 1);// 1 = off
  digitalWrite(LED_B, 1);
  
  pinMode(IR_LED_R, OUTPUT);
  pinMode(IR_LED_L, OUTPUT);
  
  pinMode(IR_38Khz, OUTPUT);
  
  digitalWrite(IR_LED_R, 1); // 0 = off
  digitalWrite(IR_LED_L, 1); // 0 = off
  digitalWrite(IR_38Khz, 0);
  
  pinMode(MOTOR_R_DIR, OUTPUT);
  pinMode(MOTOR_R_SPD, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_L_SPD, OUTPUT);
  
  digitalWrite(MOTOR_R_DIR, FWD);
  analogWrite(MOTOR_R_SPD, 0);
  digitalWrite(MOTOR_L_DIR, FWD);
  analogWrite(MOTOR_L_SPD, 0);
  
  pinMode(BATT_VOLTAGE, INPUT);
  
  pinMode(IR_SENSOR_R, INPUT);
  pinMode(IR_SENSOR_L, INPUT);
    
// ================================================================
// ===               2.4Ghz Transceiver Setup                   ===
// ================================================================  

  //nrf24.init();
  
  if (!nrf24.init())
    Serial.println("Radio init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  nrf24.setChannel(2); // Set the desired Transceiver channel valid values are 0-127, in the US only channels 0-83 are within legal bands
  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);   
    
// ================================================================
// ===                 PID Feedback Loop Setup                   ===
// ================================================================ 
  
  setpoint = 1800;	
  myPID.SetOutputLimits(-10,10);
  myPID.SetSampleTime(0);
  myPID.SetMode(AUTOMATIC); //Initialize PID parameters
          
// ================================================================
// ===                       38Khz SETUP                        ===
// ================================================================ 

  pinMode(3, OUTPUT);
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Just enable output on Pin 3 and disable it on Pin 11
  TCCR2B = _BV(WGM22) | _BV(CS22);
  OCR2A = 51; // defines the frequency 51 = 38.4 KHz, 54 = 36.2 KHz, 58 = 34 KHz, 62 = 32 KHz
  OCR2B = 26;  // deines the duty cycle - Half the OCR2A value for 50%
  TCCR2B = TCCR2B & 0b00111000 | 0x2; // select a prescale value of 8:1 of the system clock
  
// ================================================================
// ===           FINAL ACTIONS BEFORE EXITING SETUP             ===
// ================================================================  
  analogWrite(MOTOR_R_SPD, 0); // Make sure both motors are off
  analogWrite(MOTOR_L_SPD, 0);
      
}// end setup loop


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
     
// ================================================================
// ===              READ GYRO AND CALCULATE ANGLE               ===
// ================================================================
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();  
    //Serial.println(F("FIFO overflow!"));
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    timeAwayGyro = millis() - lastMillisGyro;
    lastMillisGyro = millis();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    // Get the Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print(ypr[0] * 180/M_PI);
    
    //yaw = ypr[0] * 57.32;    // Scale -180 to +180 degrees
    //yaw = yaw + 180;         // Scale to 0 - 360 degrees
    
    yaw = ypr[0] * 573.2; // Scale to -1800 - +1800 degrees
    yaw = yaw + 1800; // Scale to 0 - 3600 degrees
    
    if ((yaw > 2700) && (yawLast < 900)) // did tollover occur from low to high = Left 
    {
      yawRotationCount --;
    }
    else if((yaw < 900) && (yawLast > 1800)) // did rollover occur from high to low = right
    {
      yawRotationCount ++;
    }
    
    yawContinuous = yaw + (yawRotationCount * 3600);
    yawLast = yaw;
    
    //yawDiff = yawStart - yaw;
    //yawDiff = yawDiff % 3600;
    //yawDiff = smod(yawDiff, 3600); 
    //yawDiff = yawStart         // Use software mod function to constrain variables
    
    //setpoint = yaw - 90;
    //yawInt = yaw;
    //setpoint = (yaw - 900) % 3600;   // need to see if this works for -180 +180 constraint
    
     // Serial.print("ypr\t");
     // Serial.print(yaw);           // This is just for debugging will not go into final code
    //Serial.print("\t");
    //Serial.print(setpoint);
     
    //setpoint = 1800; // set in setup
    input = yaw; // done in setup of PID
    
    /*
    P_Param is %OutputSpan/%InputSpan (where % is calculated using the Input and Output Limits)
    I_Param and D_Param are both seconds
   
    Parameters and what they do (sort of)
    P_Param: the bigger the number the harder the controller pushes.
    I_Param: the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.
    D_Param: the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered)
    */
    
    myPID.Compute(); // Compute the new PID values based on the setpoint and input values
    
    //analogWrite(MOTOR_R_SPD,output + 100); // Modify the motor speed based on the PID output
    
    outputInt = output;
    
        
    
  } // End gyro update loop
    
     
// ================================================================
// ===                  READ DATA FROM REMOTE                   ===
// ================================================================
  
  if (nrf24.available()){ // Is there received data from the remote control?
   
    if (nrf24.recv(receiveBuffer, &lengthReceive)){ // receive the available data into the "receivebuffer" variable
    
    sendCounter++;
    if (sendCounter > 20){ // Only send data back to the remote periodically so as to improve the receive speed. 
      sendCounter = 0;
      nrf24.send(sendBuffer, sizeof(sendBuffer)); // send the data inside the "sendBuffer" variable
    }
   
    
    timeAway = millis() - lastMillis;
    lastMillis = millis();

    buttons = receiveBuffer[0];
    
    sendBuffer[0] = map(yaw, 0, 3600, 0, 255);
    battVoltage = analogRead(BATT_VOLTAGE);
    sendBuffer[1] = map(battVoltage,0,1023,0,255);
    sendBuffer[2] = timeAway;
    sendBuffer[3] = timeAwayGyro;
    sendBuffer[4] = outputInt; // Send some new data to the remote here for debugging
    sendBuffer[5] = 23; // Send some new data to the remote here for debugging
    sendBuffer[6] = stateMachine; // Send some new data to the remote here for debugging

     
    if (bitRead(buttons, UP) == HIGH){ // Forward
    
      digitalWrite(MOTOR_R_DIR, FWD);
      digitalWrite(MOTOR_L_DIR, FWD);
      analogWrite(MOTOR_R_SPD, forwardRamp);
      analogWrite(MOTOR_L_SPD, forwardRamp);
      
      if (forwardRamp > 240) // keep ramp value from overflowing back to 0
      {
        forwardRamp = 255;
      } 
      else
      { 
        forwardRamp = forwardRamp + 10; // increment ramp value by 1
      }
    }
    else{
      forwardRamp = 30;
      loopTimer = 0;
    }  
    
    //if (bitRead(buttons, A) == HIGH){ // use gyro to drive at 1800 deg
    //}
    
    if (bitRead(buttons, B) == HIGH){ // Use gyro to turn 1800 deg to the right
    
      if(startBButton == true)
      {
        startBButton = false;
        startYawContinuous = yawContinuous;
        
        digitalWrite(MOTOR_R_DIR, FWD);
        digitalWrite(MOTOR_L_DIR, FWD);
      }
     
      if (yawContinuous < (startYawContinuous + 800))
      {
        analogWrite(MOTOR_R_SPD, 20);
        analogWrite(MOTOR_L_SPD, 150);
      }
      else
      {
        analogWrite(MOTOR_R_SPD, 0);
        analogWrite(MOTOR_L_SPD, 0);
      }
    }
    else
    {
      startBButton = true;
    }
    
    if (bitRead(buttons, C) == HIGH){ // USE GYRO TO GO STRAIGHT
      if(startCButton == true)
      {
        startCButton = false;
        setpoint = yaw;
        myPID.SetTunings(.1,.001,.001); // P, I, D tuning parameters set 1.2,0, .01
        myPID.SetOutputLimits(-20,20);
        myPID.SetSampleTime(0);
      }
      
      digitalWrite(MOTOR_R_DIR, FWD);
      digitalWrite(MOTOR_L_DIR, FWD);
       
      analogWrite(MOTOR_R_SPD, 150 + outputInt);
      analogWrite(MOTOR_L_SPD, 150);
    }
    else
    {
      startCButton = true;
    }
    
    if (bitRead(buttons, D) == HIGH) // USE GYRO TO GO STRAIGHT
    {
      if(startDButton == true) // First time running through this function?
      {
        startDButton = false;
        forwardRampD = 30;
        setpoint = yaw;
        myPID.SetTunings(.1,.001,.001); // P, I, D tuning parameters set 1.2,0, .01
        myPID.SetOutputLimits(-5,5);
        myPID.SetSampleTime(0);
      }
      for (int i = 0; i < 1000; i++)
      { 
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
  
        //if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check for overflow (this should never happen unless our code is too inefficient)  
        //  mpu.resetFIFO();  // reset so we can continue cleanly
        //} 
        if (mpuIntStatus & 0x02) 
        {
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();// wait for correct available data length, should be a VERY short wait
          timeAwayGyro = millis() - lastMillisGyro;
          lastMillisGyro = millis();
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          yaw = ypr[0] * 573.2; // Scale to -1800 - +1800 degrees
          yaw = yaw + 1800; // Scale to 0 - 3600 degrees
          input = yaw; // done in setup of PID
    
          if (forwardRampD > 245) // // once full speed reached
          {
            myPID.Compute(); // Compute the new PID values based on the setpoint and input values
            outputInt = output;
            analogWrite(MOTOR_R_SPD, 245 + outputInt); //offset output by gyro PID correction value
            analogWrite(MOTOR_L_SPD, 245 - outputInt);
          } 
          else
          { 
            analogWrite(MOTOR_R_SPD, forwardRampD); // drive straight by ramp value
            analogWrite(MOTOR_L_SPD, forwardRampD);
            forwardRampD = forwardRampD + 5; // increment ramp value by 1
          }
        } // end gyro read
      }// end for loop
    }// end button D if statement
    else
    {
      startDButton = true;
    }
    
    if (bitRead(buttons, DOWN) == HIGH){ // Backwards
      digitalWrite(MOTOR_R_DIR, BWD);
      digitalWrite(MOTOR_L_DIR, BWD);
      analogWrite(MOTOR_R_SPD, 100);
      analogWrite(MOTOR_L_SPD, 100);
    }
    if (bitRead(buttons, LEFT) == HIGH){ // Left
      digitalWrite(MOTOR_R_DIR, FWD);
      digitalWrite(MOTOR_L_DIR, BWD);
      analogWrite(MOTOR_R_SPD, 50);
      analogWrite(MOTOR_L_SPD, 50);
    }
    else if (bitRead(buttons, RIGHT) == HIGH){ // Right
      digitalWrite(MOTOR_R_DIR, BWD);
      digitalWrite(MOTOR_L_DIR, FWD);
      analogWrite(MOTOR_R_SPD, 50);
      analogWrite(MOTOR_L_SPD, 50);
    } 
    
    if (buttons == 0) // No buttons pushed
    {
      analogWrite(MOTOR_R_SPD, 0);
      analogWrite(MOTOR_L_SPD, 0);
    }
    }
    
    if (bitRead(buttons, A) == HIGH) // RUN AUTO
    {
      AUTO();
    }
    else
    {
      stateMachine = 0;
    }
    
  } // end receive avaiable loop
    
  
  digitalWrite(LED_B, digitalRead(IR_SENSOR_L)); // Set the Blue LED to turn ON when the Left IR sensor sees a wall
  
  if (digitalRead(IR_SENSOR_R) == WALL_DETECTED) // this if statement does the same thing as the above lines but is written with an IF statement
  {
    digitalWrite(LED_G, LED_ON); // Turn ON the Green LED
  }
  else
  {
    digitalWrite(LED_G, LED_OFF); // Turn OFF the Green LED
  }
    
    

    
} // end main loop


void AUTO()
{
  if (stateMachine == 0) // setup millis value
  {  
    lastMillisAuto = millis();
    forwardRampAuto = 30;
    stateMachine = 1;
    digitalWrite(MOTOR_R_DIR, FWD);
    digitalWrite(MOTOR_L_DIR, FWD);
  }
  if (stateMachine == 1) // Drive straight to end of first block
  {
    
    if (forwardRampAuto > 245) // // once full speed reached
    {
      analogWrite(MOTOR_R_SPD, 225); // turn slightly to the right
      analogWrite(MOTOR_L_SPD, 245);
    } 
    else
    { 
      analogWrite(MOTOR_R_SPD, forwardRampAuto); 
      analogWrite(MOTOR_L_SPD, forwardRampAuto);
      forwardRampAuto = forwardRampAuto + 5; // increment ramp value
    }
    if ((lastMillisAuto + 50) < millis()); // wait 50ms to clear gap before checking right sensor
    {
      if (CheckRightSensor() == false)
      {
        stateMachine = 2;
        TurnRightNinetyDegrees(1);
      }
    }
  }
  if (stateMachine == 2) // Turn right
  {
    TurnRightNinetyDegrees(0); // run Turn right and check if done 
  }
  if (stateMachine == 3) // Turn right
  {
      analogWrite(MOTOR_R_SPD, 0); 
      analogWrite(MOTOR_L_SPD, 0);
  }
} // end auto function

bool CheckRightSensor()
{
  return (!digitalRead(IR_SENSOR_R));
}

void TurnRightNinetyDegrees(char start)
{
  if(start == 1)
  {
    startYawContinuous = yawContinuous;
    digitalWrite(MOTOR_R_DIR, FWD);
    digitalWrite(MOTOR_L_DIR, FWD);
    analogWrite(MOTOR_R_SPD, 25);
    analogWrite(MOTOR_L_SPD, 150);
  }

  if (yawContinuous > (startYawContinuous + 700))
  {
    stateMachine ++;
  }
}

