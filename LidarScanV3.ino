#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect the stepper motor with 200 steps per revolution (1.8 degree) to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200,2);
int stepsToRotate=400; // 400 steps per 360 degree rotation when using interleave
int numRotation=0; // counter to count number of rotations
int currentStep=0; // counter to count the step number
int stepIncrement=1; // X steps to rotate at a time, typically 1
double rotate=.5; // X of a full rotation, 1 for full rotation

// Connect a servo ovject to control a servo
Servo myservo;
int servoStart=35; // start servo at X degrees
int servoEnd=140; // end when servo reaches X degrees
int servoPos=40;  // variable to store the servo posistion
int servoNormal=160; // define angle where servo is normal to ground


// Connect the TFMini Lidar
uint16_t distance = 0; //distance
uint16_t strength = 0; // signal strength
uint8_t rangeType = 0; //range scale, 00 (short dist.), 03 (intermediate dist.), 07 (long dist.)
boolean valid_data = false; //ignore invalid ranging data
const byte sensor1 = 0x10; //TFMini I2C Address

void setup() {
  Wire.begin();
  //Serial.println("Start");
  Serial.begin(115200);  // set-up Serial Library at 115200 bps
  AFMS.begin();  // create with the default frequency 1.6KHz

  myMotor->setSpeed(10); // 10 rpm
  
  myservo.attach(10); // attaches the servo on pin 10 to the servo object
  //myMotor->step(100,BACKWARD,INTERLEAVE); // run the stepper motor
}

void loop() {
    for(int i=servoStart;i<=servoEnd;i++){ // sweep across the full servo range
      myservo.write(i); // increase servo angle 1 degree
      if(i==servoStart){
        delay(400);
      }
      delay(10);
      if(servoPos!=i){ // check if the servo is at a new angle
        if (readDistance(sensor1) == true){
          if (valid_data == true) {
            double rho=distance;
            double theta=toRad(getStepperAngle());
            double phi=toRad(servoNormal-i);
            double x=rho*sin(phi)*cos(theta);
            double y=rho*sin(phi)*sin(theta);
            double z=rho*cos(phi);
            Serial.print(x);
            Serial.print(" ");
            Serial.print(y);
            Serial.print(" ");
            Serial.println(z);
            //Serial.println(";");
          }
          else {
          //don't print invalid data
          }
          
        }
        servoPos=i; // update servoPos to i
      }    
    }
    if(currentStep>=(stepsToRotate*rotate)){ // check if the stepper has fully rotated
      myservo.write(servoStart); // reset the servo to 0 degrees
      delay(100);
      myservo.detach();
      myMotor->release();
      //Serial.println("TFMini Lidar Scan Complete");
      Serial.end();
    }
    
    myMotor->step(stepIncrement,FORWARD,INTERLEAVE); // run the stepper motor
    currentStep+=stepIncrement;      
}

// gets angle of stepper in degrees
double getStepperAngle(){
  int steps=0;
  steps=(currentStep)%stepsToRotate;
  double theta=(360.0/stepsToRotate)*steps;
  return theta;
}

// converts degrees to radians
double toRad(double degree){
  double rad=degree*PI/180.0;
  return rad;
}


// checks if lidar distance is valid
boolean readDistance(uint8_t deviceAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x01); //MSB
  Wire.write(0x02); //LSB
  Wire.write(7); //Data length: 7 bytes for distance data
  if (Wire.endTransmission(false) != 0) {
    return (false); //Sensor did not ACK
  }
  Wire.requestFrom(deviceAddress, (uint8_t)7); //Ask for 7 bytes

  if (Wire.available())
  {
    for (uint8_t x = 0 ; x < 7 ; x++)
    {
      uint8_t incoming = Wire.read();

      if (x == 0)
      {
        //Trigger done
        if (incoming == 0x00)
        {
          //Serial.print("Data not valid: ");//for debugging
          valid_data = false;
          //return(false);
        }
        else if (incoming == 0x01)
        {
          // Serial.print("Data valid:     "); //for debugging
          valid_data = true;
        }
      }
      else if (x == 2)
        distance = incoming; //LSB of the distance value "Dist_L"
      else if (x == 3)
        distance |= incoming << 8; //MSB of the distance value "Dist_H"
      else if (x == 4)
        strength = incoming; //LSB of signal strength value
      else if (x == 5)
        strength |= incoming << 8; //MSB of signal strength value
      else if (x == 6)
        rangeType = incoming; //range scale
    }
  }
  else
  {
    Serial.println("No wire data avail");
    return (false);
  }

  return (true);
}
