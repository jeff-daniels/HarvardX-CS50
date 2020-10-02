# HarvardX-CS50  
## Autonomous Softball-Seeking Robot  
Repository contains files used for my final project for the HarvardX-CS50 Introduction to Computer Science.
The hardware is a [Parallax BOE-Bot](https://www.parallax.com/product/32335) with an [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3) microcontroller. It has a [passive infrared sensor](https://www.parallax.com/product/555-28027) that detects movement. When movement is detected, the robot scans the area in a 180 degree field of view using an [ultrasonic sensor](https://www.parallax.com/product/910-28015a). This creates an array of distances to objects in the field of view. An algorithm developed by me decides if there is a softball in the vicinity and if there is, the robot turns toward it and moves near it. If a softball is not detected, the robot moves to another part of the area and scans that area. This continues until the softball is found, the robot mistakes something else for a softball, or a mechanical failure occurs. The robot will scan at three positions laterally, then advance forward, and move laterally. This is a crude sweeping pattern and is intended to cover the area where the infrared sensor detects movement.  

## Files in this repository:  
locateAndApproach.ino - The code that is uploaded to the Arduino microcontroller.  Arduino code is very similar in systax to C++.  
locateAndApproach.ods - A spreadsheet for assisting with code development.  It inputs the utrasonic sensor readings and outputs a graph.  

## External Files:  
[CS50 Final Project Autonomous Softball-Seeking Robot](https://youtu.be/zdHkmBB07lQ) - A poorly shot cell phone video of the robot in action.   
