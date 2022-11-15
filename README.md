# MakeMeWalk
Welcome to EMME Robotics Challenge#1 Make Me Walk. 

In this challenge you will try to write codes to make
a quadruped robot achieve its gait. 

## Goal
Make it walk. 

Don't worry too much about the coding process at the beginning.
This challenge is not about who writes better code, 
but rather to give you guys an over all idea of how the robot work.

## Do your own research and discuss with your team/friends
Don't know how to start? Don't worry, you aren't the first person to try to make a quadruped robot walk.

**Use the internet, Google it, watch YouTube videos, looking at others codes, do some research.**

Also, discuss with your teamates and friends will help a lot.

These are a few question you might want to find out:

1. How do quadrupedal animals walk, how do quadrupedal gaits looks like?
2. What would be the input of the robot?
3. How could we control each motor in a way that make a robot walk?

**KEYWORDS: Quadruped Robot, Quadruped Gait, Dog Robot, Robot Kinematics**

## Choose your board
You can choose these two boards for deployment of your software:
1. Arduino or ESP32 (Using C++) -> Please refer to the directory `Arduino` for details
2. Raspberry Pi 4B (Running ROS) -> Please refer to the directory `ros_pkg` for details

I highly recommded starting with the Arduino or ESP32. Although you will need to learn C++, 
I assure you that it is a powerful language and might not be as hard as you think.

Only choose Raspberry Pi if you really know how to work with ROS (Robot Operating System). 
Raspberry Pi **will support Python**, however, only through ROS.

## Robot's Component and Specification
### Components Diagram
```
______________                    _______________              ____________________              ___________________
|            |                    |             |              |                  |              |                 |
| Controller | ----Bluetooth----> | RPi/Arduino | ----I2C----> | PWM Servo Driver | ----PWM----> | 12x Servo Motor | 
______________                    _______________              ____________________              ___________________

```
### Specifications Table
Now loading ...

## Servo Motor and PWM Servo Driver
Adafruit PCA9685 PWM Servo Driver Board is used. 

Now loading ...

## Bluetooth Controller
Now loading ...

## IMU
Now loading ...