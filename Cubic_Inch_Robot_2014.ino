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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


// ================================================================
// ===               2.4Ghz Transceiver Includes                ===
// ================================================================

#include <SPI.h>
// library wait time modified so as not to slow down the program if signal is lost
#include <nRF24L01p.h> 

nRF24L01p receiver(14,8);//CSN,CE

String message;
int iteration=0;

// ================================================================
// ===                    PID Library Includes                  ===
// ================================================================
#include "PID_CL_2014.h"

//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,2,5,1, DIRECT); 
// Last input "DIRECT" or "REVERSE" will change which way the correction value goes
//switch them if the correction  makes things worse

// ================================================================
// ===                      Robot Pin Setup                     ===
// ================================================================

// Lets define some nice handy constants eh?

#define LED_R 13 // Red LED - Shared with SCK used for NRF24L01 Transceiver
#define LED_G 0  // Green LED - Shared with serial port receiver pin
#define LED_B 4  // Blue LED

//Pull these pins high to enable a specific LED
//Cathode is connected to 38khz pin
#define LED_IR_R 1 // IR LED Right used for reflective wall sensing
#define LED_IR_L 7 // IR LED Left used for reflective wall sensing
#define 38Khz 3    // IR LED 38khz Cathode connection

#define SENSOR_R 15 // Right input from 38khz bandpass filter connected to IR PIN diode receiver
#define SENSOR_L 16 // Left

#define MOTOR_R_DIR 5 // Right motor direction pin
#define MOTOR_R_SPD 9 // Right motor speed pin - apply PWM signal (analog out) to this pin
#define MOTOR_L_DIR 6
#define MOTOR_L_SPD 10

#define BAT_VOLTAGE 17 //Battery voltage monitor pin - connected to 50% divider to allow the measurment of voltages higher than the vcc of 3.3v

// these need to be checked!
#define FWD 0 // 0 = forward in our robot wiring
#define BWD 1 // 1 = backward in our robot wiring


bool blinkState = false;
bool blinkState1 = false;
bool blinkState2 = false;

int yaw = 0;
//int yawStart = 0;
//int yawDiff = 0;

// MPU control/status vars
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



// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 2; //2 = 800khz I2C - fastest possible data rate
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
// ================================================================
// ===               2.4Ghz Transceiver Setup                   ===
// ================================================================  

    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setBitOrder(MSBFIRST);
    receiver.channel(90);
    receiver.TXaddress("CIRem");
    receiver.RXaddress("CIBot");
    receiver.init();
    //delay(3000);
    Serial.println("I'm PRX as transceiver");
    //receiver.txPL("Hi PTX. I'm ready.");
    //receiver.send(SLOW);
    //receiver.txPL("tell me the library's name");
    //receiver.send(SLOW);

// ================================================================
// ===                     Robot Pin Setup                      ===
// ================================================================
    // configure LED for output
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    
// ================================================================
// ===                 PID Feedback Loop Setup                   ===
// ================================================================ 
        //Initialize PID parameters
        setpoint = 0;	
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(0,100);
        myPID.SetSampleTime(20);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
       
        
        if(receiver.available()){ // Is there received data from the remote control?
            message="";
            receiver.read();
            receiver.rxPL(message); //save this data to the "message" variable
            Serial.print("PTX says: \""); // print this data to the serial port for debugging
            Serial.print(message); // This will be commented out in final version
            Serial.println("\"");
        }
        
       // blinkState1 = !blinkState1;
       // digitalWrite(LED_B, blinkState1);
    }
    //digitalWrite(LED_B, 0); // turn off the indicator 2 led incase it was left on
    
    //blinkState2 = !blinkState2; // invert blinkstate2
   // digitalWrite(LED2_PIN, blinkState2);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

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
        //yawStart = 1800; // 10 = 1.0 deg = the angle we are trying to follow
        
        //yawDiff = yawStart - yaw;
        //yawDiff = yawDiff % 3600;
        //yawDiff = smod(yawDiff, 3600); 
        //yawDiff = yawStart         // Use software mod function to constrain variables
        
        //setpoint = yaw - 90;
        //yawInt = yaw;
        setpoint = (yaw - 900) % 3600;   // need to see if this works for -180 +180 constraint

        Serial.print("ypr\t");
        Serial.print(yaw);           // This is just for debugging will not go into final code
        Serial.print("\t");
        Serial.println(setpoint);
        
        receiver.txPL(yaw);          // Send the same debugging data over the 2.4ghz transceiver to remote control
        receiver.send(FAST);         // Send it fast without error checking
        
        setpoint = 0;
        input = yaw;
        
        myPID.Compute(); // Compute the new PID values based on the setpoint and input values
        
        analogWrite(5,output + 100); // Modify the motor speed based on the PID output

       // Serial.print("\t");
       //Serial.print(ypr[1] * 180/M_PI);
        //Serial.print("\t");
        //Serial.println(ypr[2] * 180/M_PI);



        // blink LED to indicate activity
       // blinkState = !blinkState;
       // digitalWrite(LED_PIN, blinkState);

    }
}

int smod(int z1, int z2) {
  int ze;
  ze=z1 % z2;
  if (ze>=0)  {
    return(ze);
  }
  else{
    return(z2+ze);
  }
}
