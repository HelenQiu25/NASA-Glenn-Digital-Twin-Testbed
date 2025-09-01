//beginning stuff (defining and including library stuff)

#include <Smoothed.h>
Smoothed <float> mySensor;

// load motor control
#include <VescUart.h>
VescUart VESCUART;
float MIN_BRAKE = 0;
float MAX_BRAKE = 7;
int NUM_SAMPLE = 50;
float BRAKE_INC = (MAX_BRAKE - MIN_BRAKE) / NUM_SAMPLE;
float brakeChange = MIN_BRAKE;
float timeRanges = 10000;
float milliseconds_persamps = timeRanges / NUM_SAMPLE;

// moving the motor stuff & constantly changed the motor speed stuff
#include <AFMotor.h>
AF_DCMotor motor(1);// create motor #3, 64KHz pwm (make more motors by reinputing this code)
float MIN_SPEED = 100;
float MAX_SPEED = 255;
int NUM_SAMPLES = 15;
float SPEED_INC = (MAX_SPEED - MIN_SPEED) / NUM_SAMPLES;
float speedChange = MIN_SPEED;
float timeRange = 10000;
float milliseconds_persamp = timeRange / NUM_SAMPLES;

// temperature sensor stuff
#include <OneWire.h>
int DS18S20_Pin = 5; //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

// encoder stuff
#define ENCODEROUTPUT 700
const int HALLSEN_A = 3; // Hall sensor A connected to pin 3 (external interrupt)
volatile long encoderValue = 0;
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
int rpm = 0;
boolean measureRpm = true;
int motorPwm = 0;

// accelerometer stuff
#include <SpikeFilter.h>
const int xInput = A0;
const int yInput = A1;
const int zInput = A2;
int RawMin = 0; // initialize minimum and maximum Raw Ranges for each axis
int RawMax = 1023;
const int sampleSize = 10;  // Take multiple samples to reduce noise

// voltage/current reading stuff
#include <Adafruit_INA260.h>
Adafruit_INA260 ina260 = Adafruit_INA260();
const int vInput = A5;                                                                                                         

// printing labels stuff
boolean print_labels = false;  // set this to true if we want to print out the labels in between the values

// sound
const int sInput = A3;

const int LED_ON = 5;
const int LED_OFF = 6;

//-------------------------------------------------------------------------------------------------------

// labels function
void write_value( char *label, float value, boolean new_line ) {
    if (print_labels) {
      Serial.print(label);
      Serial.print(" ");
  }
    if  (new_line) {
      Serial.println(value);
  } 
    else {
      Serial.print(value);
      Serial.print(",");
  }

}

//--------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);// set up Serial library at 9600 bps

// load motor setup
  Serial1.begin(115200);
  while (!Serial1) {;}
  VESCUART.setSerialPort(&Serial1);

// encoder setup stuff
  EncoderInit();//Initialize the module
   encoderValue = 0;
   previousMillis = millis();
  
// accelerometer setup stuff
  analogReference(EXTERNAL);

// current sensor setup stuff
  while (!Serial) { delay(10); }
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  //ina260.setAveragingMode(INA260_AVERAGING_MODE_64);
  ina260.setAveragingCount(INA260_COUNT_64);
}

//---------------------------------------------------------------------------------------------------------

//temp sense function
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
}

// -----------------------------------------------------------------------------------------------------

void loop() {

  float time = millis() / 1000.0;
  write_value("Time:", time, false);

// motor move loop stuff
  //sets motor speed to speedChange
  motor.setSpeed(255);
  write_value("Speed:", 255, false);
  // change BACKWARD or FORWARD for direction change
  motor.run(FORWARD);
  
// load motor loop stuff
  VESCUART.setCurrent(brakeChange);
  //VESCUART.setRPM(40);
  write_value("BrakeCurrent:", brakeChange, false);
// speed stuff; increases speed slowly by increments and resets when max is reached
 if(speedChange < MAX_SPEED)
 {
  speedChange += SPEED_INC;
 }
 else
 {
  speedChange = MIN_SPEED;
 }

// brake stuff; increases brake current slowly and resets when max is reached
  if(brakeChange < MAX_BRAKE)
  {
    brakeChange += BRAKE_INC;
  }
  else
  {
    brakeChange = MIN_BRAKE;
  }

// temperature loop stuff
  float temperature = getTemp();
  write_value("Celsius:", temperature, false);

// encoder loop stuff
  float rpm = get_rpm() ;
  write_value("RPM:", rpm, false);
  // Only update display when there is readings
  if ( rpm > 0) {
      }
// accelerometer loop stuff
  //Read raw values
 float x_accel = get_acceleration(xInput) - 501;
 write_value("X:", x_accel, false);

 float y_accel = get_acceleration(yInput) - 508;
 write_value("Y:", y_accel, false);

 float z_accel = get_acceleration(zInput) - 626;
 write_value("Z:", z_accel, false);

// voltage read loop stuff
  float voltage = (get_voltage() * .0075) - .818;
  write_value("Voltage:", voltage, false);

// current loop stuff
  float current = ina260.readCurrent();
  write_value("Current:", current, false);

// sound loop
  float sound = get_sound();
  write_value("Sound:", sound, true);
 
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    switch(inByte) {
      case LED_ON:
        Serial.println("LED_ON received");
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case LED_OFF:
        Serial.println("LED_OFF received");
        digitalWrite(LED_BUILTIN, LOW);
        break;
    }

  }
return;
  
}

// ----------------------------------------------------------------------------------------------------------

//encoder functions stuff
void EncoderInit()
{
// Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder, RISING);
}
void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
}

// ----------------------------------------------------------------------------------------------------------------

// accelerometer functions stuff
int ReadAxis(int axisPin) // Take samples and return the average
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
  reading += analogRead(axisPin);
  }
  return reading/sampleSize;
}

float get_acceleration(int Input)
{
  SpikeFilter myFilter(Input);
  float Raw = myFilter.filter();
  return (float)Raw;
}

// ----------------------------------------------------------------------------------------------------------------

//RPM/encoder function
float get_rpm()
{
    currentMillis = millis(); // Update RPM value on every second
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    // Revolutions per minute (RPM) =
    // (total encoder pulse in 1s / motor encoder output) x 60s <--THIS IS WHAT THE RPM EQUATION IS
    rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);

    encoderValue = 0;
  }
  return rpm;
}
//-------------------------------------------------------------------------------------------------------------------

// voltage function
float get_voltage()
{
  long voltage = analogRead(vInput);
  return (float)voltage;
}
//-------------------------------------------------------------------------------------------------------------------

// sound

float get_sound()
{
  long sound = analogRead(sInput);
  return (float)sound;
}