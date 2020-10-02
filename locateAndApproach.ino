

/*
 * Locate and Approach
 * BOE Shield-Bot application uses Ping))) mounted on servo turret.
 * Bot sits until there is movement detected by an IR sensor and 
 * Approaches said object, stops and turns around
 * Much of this code is from Roam_with_Ping_on_Servo_Turret
 * Source: http://learn.parallax.com/BoeShield/RoamingPingShieldBot
 *
 *
 * Created 22Dec2015
 * by Jeff Daniels
 * version 1.0
 */
                                                                                
#include <Servo.h>                                 // Include servo library
#include <NewPing.h>                               // Include NewPing library
                                                   // Supposedly a better ping library

Servo servoLeft;                                   // Servo object instances
Servo servoRight;
Servo servoTurret;  

const int servoLeftPin = 13;                       // I/O Pin constants
const int servoRightPin = 12;
const int servoTurretPin = 11;
const int pingPin = 10;
const int piezoPin = 4;
#define TRIGGER_PIN  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

#define PIR_PIN 2        //passive Infrared detector pin

const int msPerTurnDegree = 6;                     // For maneuvers  
const int msPerCm = 60;
int ccwLim = 1400;                                 // For turret servo control
int rtAngle = 900;

// Number of Turret Positions
const int turretRange = 180;
const int numTurretPositions = 45;
int turretPosition;
const int turretDelay = 0;                      // How long to wait between turret movements

// Declare array to store that many cm distance elements using pre-calculated
// number of elements in array.

int cm[numTurretPositions];

// Pre-calculate degrees per adjustment of turret servo. 
const int degreesTurret = turretRange/numTurretPositions;

// Search parameters
const int maxDetectionDistance = MAX_DISTANCE;
const float targetWidth = 8.25*2.54;
const int distanceSamples = 3;
// Fudge factors for object recognition
const float distTol = targetWidth*0.5;
const float widthTol = targetWidth*0.5;
const float widthCalibration = 0.5;

void setup()                                       // Built-in initialization block
{
  
  Serial.begin(9600);                              // Open serial connection

  servoLeft.attach(servoLeftPin);                  // Attach left signal to pin 13
  servoRight.attach(servoRightPin);                // Attach right signal to pin 12
  servoTurret.attach(servoTurretPin);              // Attach turret signal to pin 11
  turret(90);                            // Set turret to 0 degrees
  delay(5000);                       //Let the PIR sensor rest

  

}  
    
void loop()                                        // Main loop auto-repeats
{
  if (detectMotion(PIR_PIN) == true)
  {
    float targetDistance;
    float targetDirection;
    boolean targetSpotted;
    lookForTarget(targetDirection, targetDistance, targetSpotted);
    if (targetSpotted == true)
    {
      reposition(targetDirection, targetDistance);
    } 
    
    delay(5000);
  }
}

/*
 * Control BOE Shield-Bot servo direction, speed, set and forget version.
 * Parameters: speedLeft - left servo speed
 *             speedRight - right servo speed
 *             Backward  Linear  Stop  Linear   Forward
 *             -200      -100......0......100       200
 */ 
void maneuver(int speedLeft, int speedRight)
{
  // Call maneuver with just 1 ms blocking; servos will keep going indefinitely.
  maneuver(speedLeft, speedRight, 1);              
}

/*
 * Control BOE Shield-Bot servo direction, speed and maneuver duration.   
 * Parameters: speedLeft - left servo speed
 *             speedRight - right servo speed
 *             Backward  Linear  Stop  Linear   Forward
 *             -200      -100......0......100       200
 *             msTime - time to block code execution before another maneuver
 * Source:     http://learn.parallax.com/ManeuverFunction
 */ 
void maneuver(int speedLeft, int speedRight, int msTime)
{
  servoLeft.writeMicroseconds(1500 + speedLeft);   // Set Left servo speed
  servoRight.writeMicroseconds(1500 - speedRight); // Set right servo speed
  if(msTime==-1)                                   // if msTime = -1
  {                                  
    servoLeft.detach();                            // Stop servo signals
    servoRight.detach();   
  }
  delay(msTime);                                   // Delay for msTime
}

void reposition(float targetDirection, float targetDistance)
{
   if (targetDirection < 90)
   {
    //turn right
    maneuver(200, -200, (90-targetDirection)*msPerTurnDegree);
   }
   if (targetDirection >=90)
   {
    //turn left
    maneuver(-200, 200, (targetDirection-90)*msPerTurnDegree);
   }
   maneuver(200,200, msPerCm*targetDistance);
   maneuver(0,0,1000);
}
/*
 * Position the horn of a Parallax Standard Servo
 * Parameter: degreeVal in a range from 90 to -90 degrees. 
 */ 
void turret(int degreeVal)
{
  servoTurret.writeMicroseconds(ccwLim - rtAngle + (degreeVal * 10));
  //Update global turret position
  turretPosition = degreeVal;
}

/*
 * Get cm distance measurment from Ping Ultrasonic Distance Sensor
 * Returns: distance measurement in cm.   
 */ 
int cmDistance()
{
  int distance = 0;                                // Initialize distance to zero
  int us = 0;                                      // Initialize micro seconds to zero
  us = sonar.ping_median(distanceSamples);         // Get Ping))) microsecond measurement
  distance = sonar.convert_cm(us);                // Convert to cm measurement
  
  return distance;                                 // Return distance measurement
}

void sweepTurret()
{
  
  //Always turn turret to zero to avoid servo malfunction
  turret(0);
  
  delay(1000);
  int index = 0;
  int indexIncrement = 1;
  int turretIncrement = degreesTurret;
   
   for (int j = 0; j<numTurretPositions; j++)
   {
     turret(turretPosition);
     delay(turretDelay);
     turretPosition += turretIncrement;
     cm[index] = cmDistance();
     index += indexIncrement;
   }
}

void displayArray(int array[], int arraySize)
{
  int angleIncrement = turretRange/arraySize;
  int angle = 0;
  for (int i = 0; i < arraySize; i++)
   {
    Serial.print(angle);
    Serial.print(" "); 
    Serial.println(array[i]);  //Display result 
    angle += angleIncrement;
   }
}

void displayTarget(float targetDirection, float targetDistance)
{
  Serial.print("There is a target at bearing ");
  Serial.print(targetDirection);
  Serial.print(" that is ");
  Serial.print(targetDistance);
  Serial.println(" cm away");  
}

void searchObjects(float &targetDirection, float &targetDistance, boolean &targetSpotted)
{
   targetDirection = 90; 
   targetDistance = MAX_DISTANCE;
   targetSpotted = false;
   for (int i=0; i<numTurretPositions; i++)
   {
       float viewAngle = 0;
       float distance = maxDetectionDistance;
       if (cm[i] != 0)
       {
          viewAngle += degreesTurret;
          distance = cm[i];
          i++;
       }
       while(cm[i] > (distance - distTol) && cm[i] < (distance + distTol))
       {
          viewAngle += degreesTurret;
          distance = degreesTurret*((distance * (viewAngle/degreesTurret-1))+ cm[i])/viewAngle;
          i++; 
       }
       
       if (isTarget(viewAngle, distance, targetWidth) == true && distance < targetDistance)
       {
           targetDirection = i * degreesTurret - viewAngle/2;
           targetDistance = distance;
           targetSpotted = true; 
       }
   }
   if (targetDistance == MAX_DISTANCE)
   {
      targetDistance = 0; 
   }
}
          
boolean isTarget(float viewAngle, float distance, float width)
{
    viewAngle = viewAngle * PI/180;                        //Convert to radians
    float estWidth = 2*distance*tan(viewAngle*0.5);
    estWidth = estWidth * widthCalibration;
    if (estWidth > (width - widthTol) && estWidth < (width+widthTol))
    {
      return true;
    }
    else
    {
       return false; 
    }
}

boolean detectMotion(int pin)
{
   int digitalInput =  digitalRead(pin);
   if (digitalInput == 1)
   {
      return true; 
   }
   else
   {
      return false; 
   }
}

void lookForTarget(float &targetDirection, float &targetDistance, boolean &targetSpotted)
{
    //Search Parameters for motion
    int straightAhead = 90;
    int turnRight = 0;
    int turnLeft = 180;
    int moveLateral = 50;
    int moveForward = 20;
    int stayPut = 0;
    int maxSearchLoops = 10;
    
    sweepTurret();
    searchObjects(targetDirection, targetDistance, targetSpotted);
    if (targetSpotted == true)
    {
       turret(straightAhead);
       return; 
    }
    //move to x = 50, y = 0
    turret(straightAhead);
    reposition(turnRight, moveLateral);
    reposition(turnLeft, stayPut);
    sweepTurret();
    searchObjects(targetDirection, targetDistance, targetSpotted);
    if (targetSpotted == true)
    {
       turret(straightAhead);
       return; 
    }
    //move to x = -50, y = 0
    turret(straightAhead);
    reposition(turnLeft, moveLateral*2);
    reposition(turnRight,stayPut);
    sweepTurret();
    searchObjects(targetDirection, targetDistance, targetSpotted);
    if (targetSpotted == true)
    {
       turret(straightAhead);
       return; 
    }
    for (int searchLoops = 0; searchLoops < maxSearchLoops;)
    {
      //move to x = -50, y = 10
      turret(straightAhead);
      reposition(straightAhead, moveForward);
      sweepTurret();
      searchObjects(targetDirection, targetDistance, targetSpotted);
      if (targetSpotted == true)
      {
         turret(straightAhead);
         return; 
      }
      //move to x = 0, y = 10
      turret(straightAhead);
      reposition(turnRight, moveLateral);
      reposition(turnLeft, stayPut);
      sweepTurret();
      searchObjects(targetDirection, targetDistance, targetSpotted);
      if (targetSpotted == true)
      {
         turret(straightAhead);
         return; 
      }
      //move to x = 50, y = 10
      turret(straightAhead);
      reposition(turnRight, moveLateral);
      reposition(turnLeft, stayPut);
      sweepTurret();
      searchObjects(targetDirection, targetDistance, targetSpotted);
      if (targetSpotted == true)
      {
         turret(straightAhead);
         return; 
      }
      //move to x = 50, y = 20
      turret(straightAhead);
      reposition(straightAhead, moveForward);
      sweepTurret();
      searchObjects(targetDirection, targetDistance, targetSpotted);
      if (targetSpotted == true)
      {
         turret(straightAhead);
         return; 
      }
      //move to x = 0, y = 20
      turret(straightAhead);
      reposition(turnLeft, moveLateral);
      reposition(turnRight, stayPut);
      sweepTurret();
      searchObjects(targetDirection, targetDistance, targetSpotted);
      if (targetSpotted == true)
      {
         turret(straightAhead);
         return; 
      }
      //move to x = -50, y = 20
      turret(straightAhead);
      reposition(turnLeft, moveLateral);
      reposition(turnRight, stayPut);
      sweepTurret();
      searchObjects(targetDirection, targetDistance, targetSpotted);
      if (targetSpotted == true)
      {
         turret(straightAhead);
         return; 
      }
      searchLoops++;
    }      
    
    
}
