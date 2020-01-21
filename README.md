# 3-D Lidar Scanner

The 3-D LIDAR Scanner was a project to learn more about LIDAR and its ability to map an area into point clouds.  In order to map a room, I used a TFMini LIDAR I2C module to measure individual point distances, rho, ranging from 30 cm to 12 m.  I swept the LIDAR module in the theta direction using a stepper motor and in phi direction using a servo motor, providing a field of view of nearly 360 degrees.  This program outputs X, Y, and Z coordinates in the serial monitor.

### Prerequisites

Uses the Adafruit Motorshield V2, NEMA 17 stepper, Arduino Uno Qwiic, 9g servo.

### Installing

Install the AccelStepper and Adafruit_Motorshield libraries.

## Built With

* [Adafruit MotorShield](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino) - The stepper library
* [Sparkfun TFMini](https://learn.sparkfun.com/tutorials/tfmini---micro-lidar-module-qwiic-hookup-guide/all) - TFMini reading
* [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)

## Versioning
V1 -- Sweep complete servo, then step

## Authors

* **Christopher Chan** 

## Acknowledgments

* Inspiration: https://www.youtube.com/watch?v=gCpCGkwwy8I&list=FLpKPRIBELHyMnnyUtZhCl9Q&index=4&t=6s
* Inspriation: http://charleslabs.fr/en/project-3D+Lidar+Scanner
