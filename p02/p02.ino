/*********************************************************************

  Provocation 02
  Group: Carolyn Chen, George Moore, Vineet Nair, Cora Zheng

  This arduino script interfaces with a computer.
  The code is largely adapted from online tutorial / example code.

  Resources: 
  https://github.com/jrowberg/i2cdevlib
  Adafruit tutorial on BLE bluetooth low energy 

*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Adafruit_WS2801.h"
#include "SPI.h" 
#include "Adafruit_DRV2605.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#ifdef __AVR_ATtiny85__
 #include <avr/power.h>
#endif

#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"
#define INTERRUPT_PIN 5
#define JUMP_THRESH 200

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
MPU6050 mpu;
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
// FSR constants and variables
const int fsrPin = A0; 
int fsrValue;
// RGB strip variables
uint8_t rgbDataPin = 6;
uint8_t rgbClockPin = 9;
Adafruit_WS2801 strip = Adafruit_WS2801(25, rgbDataPin, rgbClockPin);
// motor variables
Adafruit_DRV2605 drv;

// logic variables
bool inAir = false;


// A small helper
void error(const __FlashStringHelper*err) {
  while(1);
}

// MPU6050 interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(void)
{
  
  // setup RGB strip
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
    clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
  #endif  
  strip.begin();
  colorWipe(Color(0,0,0),1);
  strip.show();

  // setup motor
  drv.begin();
  drv.selectLibrary(1);
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG);

  // setup bluetooth connection
  ble.begin(VERBOSE_MODE);
  if (FACTORYRESET_ENABLE)
  {
    !ble.factoryReset();
  }
  ble.echo(false);
  ble.info();
  ble.verbose(false);

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  ble.setMode(BLUEFRUIT_MODE_DATA);
  delay(3000);


  // setup I2C for MPU6050
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  mpu.testConnection();
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  // gyro and accelerometer offsets here, results from calibration
  mpu.setXAccelOffset(343);
  mpu.setYAccelOffset(4066);
  mpu.setZAccelOffset(1411);
  mpu.setXGyroOffset(107);
  mpu.setYGyroOffset(13);
  mpu.setZGyroOffset(34);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void loop(void)
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // check sensor data
  char data[BUFSIZE+1];

  delay(50);

  if ( getSensorData(data, BUFSIZE) )
  {
    // send data to Bluefruit
    ble.println(data);
    if (fsrValue > 700) {
      // set the effect to play
      drv.setWaveform(0, 52);  // play effect 
      drv.setWaveform(1, 0);       // end waveform
      // play the effect!
      drv.go();
      rainbow(1);
    }
    else {
      drv.setWaveform(0, 0);  // stop 
      drv.go();
      colorWipe(Color(0,0,0),1);
    }
  }

////   check for characters from Bluefruit
//  ble.println("AT+BLEUARTRX");
//  ble.readline();
//  if (strcmp(ble.buffer, "OK") == 0) {
//    // no data
//    return;
//  }

//   Serial.println(ble.buffer);
}

bool getSensorData(char buffer[], uint8_t maxSize)
{
  while (!mpuInterrupt && fifoCount < packetSize) {
    
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fsrValue = analogRead(fsrPin);
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // grab FSR value
      fsrValue = analogRead(fsrPin);
      sprintf(buffer, "%d,%d,%d;", fsrValue, (int) (ypr[1] * 180/M_PI), (int) (ypr[2] * 180/M_PI));

  }

  return true;
}

void rainbow(uint8_t wait) {
  int i, j;
   
  for (j=0; j < 10; j++) {     // 3 cycles of all 256 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel( (i+j+5) % 255));
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint32_t c, uint8_t wait) {
  int i;
  
  for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

