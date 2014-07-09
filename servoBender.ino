//Four servos used as string benders mounted on a guitar.
//Uses continuous ratiometric hall effect sensors on the pedals
//and trim potentiometers to set the upper limit of the bend. 

#include <Servo.h> 
#include <EEPROM.h>

#define NOFIELD 505L    // Analog output with no applied field, calibrate this

#define TOMILLIGAUSS 1953L 
#define SERVO_MAX 60
#define SERVO_THRESHOLD 10
#define TRIM_THRESHOLD 30

#define PIN_HALL_1 A2
#define PIN_HALL_2 A3
#define PIN_HALL_3 A0
#define PIN_HALL_4 A1

#define PIN_TRIM_1 A5
#define PIN_TRIM_2 A4
#define PIN_TRIM_3 A7
#define PIN_TRIM_4 A6

#define PIN_SERVO_1 5
#define PIN_SERVO_2 9
#define PIN_SERVO_3 10
#define PIN_SERVO_4 11


#define PIN_LED 13
#define PIN_BUTTON_CALIBRATE 12

#define NUM_STRINGS 4
 
Servo servos[NUM_STRINGS];  // create servo object to control a servo 

int servo_pins[NUM_STRINGS] = {PIN_SERVO_1, PIN_SERVO_2, PIN_SERVO_3, PIN_SERVO_4};
int reverse_servos[] = {PIN_SERVO_1, PIN_SERVO_2};
int hall_pins[NUM_STRINGS] = {PIN_HALL_1, PIN_HALL_2, PIN_HALL_3, PIN_HALL_4};
int trim_pins[NUM_STRINGS] = {PIN_TRIM_1, PIN_TRIM_2, PIN_TRIM_3, PIN_TRIM_4};

int servoVals[NUM_STRINGS];    // variable to read the value from the analog pin 
int trimVals[NUM_STRINGS];

int calibrating = false;

int calibrationMax[NUM_STRINGS];
int calibrationMin[NUM_STRINGS];
 
void setup() 
{ 
  pinMode(PIN_BUTTON_CALIBRATE, INPUT);
  digitalWrite(PIN_BUTTON_CALIBRATE, HIGH);
  
  for (int i=0; i<NUM_STRINGS; i++){
    pinMode(hall_pins[i], INPUT);
    pinMode(trim_pins[i], INPUT);
    
    servos[i].attach(servo_pins[i]);  // attaches the servo on pin 9 to the servo object
    pinMode(servo_pins[i], OUTPUT);
    
    calibrationMin[i] = EEPROM.read(i);
    calibrationMax[i] = EEPROM.read(i + NUM_STRINGS);
    
    if(check_reverse_pins(servo_pins[i])){
      servos[i].write(0);
    }
    else {
      servos[i].write(180);
    }
  }
}

int DoMeasurement(int pin)
{
// measure magnetic field
  int raw = analogRead(pin);   // Range : 0..1024

  long compensated = raw - NOFIELD;                 // adjust relative to no applied field 
  long gauss = compensated * TOMILLIGAUSS / 1000;   // adjust scale to Gauss
  
  int val = map(abs(gauss), 0, 1023, 0, 255);
  return val;
}
 
void loop() 
{   
  read_trims();
  
  read_halls();
  
  if (digitalRead(PIN_BUTTON_CALIBRATE) == LOW && !calibrating) {
    calibrating = true;
    calibrate();
  }
  
  delay(15);                           // waits for the servo to get there 
}

void read_halls() {
  for(int i=0; i<NUM_STRINGS; i++){
    int newVal = DoMeasurement(hall_pins[i]);   // reads the value of the potentiometer (value between 0 and 1023) 
    if(abs(newVal - servoVals[i]) > SERVO_THRESHOLD || newVal == 0 || newVal == 255){
      int val = map(newVal, calibrationMin[i], calibrationMax[i], 0, trimVals[i]);
      val = constrain(val, 0, trimVals[i]);
      if(val - servoVals[i] != 0){
        servoVals[i] = val;
        if(check_reverse_pins(servo_pins[i])){
           servos[i].write(val);
        }
        else {
          servos[i].write(180 - val);
        }        // sets the servo position according to the scaled value
      }
    }
  }
}

void read_trims() {
  for(int i=0; i<NUM_STRINGS; i++){
    int newTrim = analogRead(trim_pins[i]);
    if(abs(newTrim - trimVals[i]) > TRIM_THRESHOLD || newTrim == 0 || newTrim == 1023){
      if(newTrim - trimVals[i] != 0){
        int val = map(newTrim, 0, 1023, 0, SERVO_MAX);
        val = constrain(val, 0, SERVO_MAX);
        trimVals[i] = val;
      }
    }
  }
}

void calibrate() {
  
  //Flash the LED to indicate calibration
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
  
  int sensorMax[NUM_STRINGS];
  int sensorMin[NUM_STRINGS];
  
  for (int i=0; i<NUM_STRINGS; i++) {
    sensorMax[i] = 0;
    sensorMin[i] = 255;
  }
  int t = 0;
  while(t < 4000){
    for (int i=0; i<NUM_STRINGS; i++) {
      int val = DoMeasurement(hall_pins[i]);
      if(val < sensorMin[i]){
        sensorMin[i] = val;
      }
      else if(val > sensorMax[i]){
        sensorMax[i] = val;
      }
    }
    delay(2);
    t++;
  }
  
  //write to memory and update variables
  for (int i=0; i<NUM_STRINGS; i++) {
    EEPROM.write(i, sensorMin[i]);
    EEPROM.write(i + NUM_STRINGS, sensorMax[i]);
    
    calibrationMin[i] = sensorMin[i];
    calibrationMax[i] = sensorMax[i];
  }
  
 digitalWrite(PIN_LED, LOW);
 calibrating = false;
} 

int check_reverse_pins(int p){
  for(int i=0; i<sizeof(reverse_servos)/sizeof(int); i++){
    if(p==reverse_servos[i]){
      return true;
    }
  }
  return false;
}
