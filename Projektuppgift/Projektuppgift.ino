// User-defined variables:
// OBS: when mounting servo during robot assembly, use SensorCenterAngle = 90;
int SensorCenterAngle  = 110;    //Angle when "distance sensor" points straight ahead. OBS: between 80-100 !
int SensorTurningAngle = 70;    //Angle the "distance sensor" turns relative to center
int ForwardWheelSpeed  = 92;   // min-max = 0-255 Choose an even number = possible to divide by 2
int BackwardWheelSpeed = ForwardWheelSpeed-0; // min-max = 0-255 Choose an even number = possible to divide by 2
int turnSpeedCompensator = 20; //If vehicle turns to the left - increase this variable. OBS between ~-30 and 30 to start with...

// Here starts the main code:
#include <Servo.h>
int pinLB=6;          // define pin6 as left back connect with IN1
int pinLF=7;          // define pin9 as left forward connect with IN2
int pinRB=8;          // define pin10 as right back connect with IN3
int pinRF=9;          // define pin11 as right back connect with IN4
int pinRSpeed = 3;    //ENA - Right motor - Black cord
int pinLSpeed = 11;   //ENB - Left  motor - Green cord
int inputPin = A0;    // define ultrasonic receive pin (Echo)
int outputPin =A1;    // define ultrasonic send pin(Trig)
int RightSensorAngle   = SensorCenterAngle-SensorTurningAngle; //Angle the distance sensor turns to the right (90 is straight ahead)
int LeftSensorAngle    = SensorCenterAngle+SensorTurningAngle; //Angle the distance sensor turns to the left  (90 is straight ahead)
int scanspeedd = 0;      //  variable for testing

int lightInputPin = A2; // define light sensor recieving pin
int lightLevel = 500; // give lightlevel a starting value to allow first loop
bool lightCutOff = false; // Cut of point where the car will stop (true = stop, false = go)
int lightCounter = 0;
int scanCounter = 0; //How many times the ultrasound has moved.
const byte scanTimes = 5; // 5 OR CODE BRAKES. must be uneven number >= 3
int scanArray[scanTimes] = {50,50,50,50,50}; // Saves values of ultrasound 
int scanDelay = 700/scanTimes; // wait per scan
int SensorAngles[scanTimes]; // Angles to check
int deltaAngle = LeftSensorAngle - RightSensorAngle; // Total movement of ultrasound
int jumpPerScan = deltaAngle/(scanTimes-1); // Jump per scan
bool descend = false; // When rotation should change

#define SERVOS 2      // Define the number of servos
Servo myservo[SERVOS];// new myservo
int servo_pins[SERVOS] = {5,10}; // means that two servos are controlled on port 5 and 10
                                 // myservo[0] will be on port 5, and myservo[1] will be on port 10, 
                                 // (and so on if more servos will be attached...)
void setup()
{
  Serial.begin(9600);
  pinMode(pinLB,OUTPUT);
  pinMode(pinLF,OUTPUT);
  pinMode(pinRB,OUTPUT);
  pinMode(pinRF,OUTPUT);
  pinMode(pinLSpeed,OUTPUT);
  pinMode(pinRSpeed,OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  pinMode(lightInputPin, INPUT);
  for(int i = 0; i < scanTimes; i++)
  {
    SensorAngles[i] = LeftSensorAngle-(jumpPerScan*i);
  }
  for(int i = 0; i < SERVOS; i++) {
    myservo[i].attach(servo_pins[i]); // Attach the servo to the servo object 
    delay(500);
  }  
  myservo[1].write(125);
  delay(100);
  delay(150);
}
void checkLightLevel() // Check brightness of surroundings 
{
  lightLevel = analogRead(lightInputPin); //read light sensor value
  Serial.println("");
  if(lightLevel <= 350) // If surroundings are too dark for too long, let boolean lightCutOff be true
  {                     // Higher value means more light.
    if(lightCounter >= 1)
    {
      lightCutOff = true;
      Serial.println("Too dark to move");
      lightCounter = 3;
    }
    if(lightCounter > 0 && lightCounter < 1)
    {
      Serial.print("Low light Counter: ");
      Serial.print(lightCounter);
      Serial.println("");
    }
    lightCounter++;
  }else
  {
    lightCutOff = false;
    lightCounter = 0;
  }
}

void turnWheelRight() // turn wheels to go right
{
  myservo[1].write(150);       
}
void turnWheelLeft() // turn wheels to go left
{
  myservo[1].write(90);       
}
void turnWheelCenter() // recenter wheels to go straight
{
  myservo[1].write(130);       
}
void checkFullScan(int c) // Call scan() 10 times in a row
{
  for(int i = 0; i < 10; i++){
    scan(c);
  }
}
void newAdvance() // spin wheels forward
{
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);
  digitalWrite(pinLF,HIGH);
  analogWrite(pinLSpeed,ForwardWheelSpeed);
  analogWrite(pinRSpeed,ForwardWheelSpeed-turnSpeedCompensator);
}
void newAdvanceFast() // spin wheels forward faster
{
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);
  digitalWrite(pinLF,HIGH);
  analogWrite(pinLSpeed,ForwardWheelSpeed+20);
  analogWrite(pinRSpeed,ForwardWheelSpeed+20-turnSpeedCompensator);
}
void newStop() // Do not spin wheels
{    
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
  digitalWrite(pinLSpeed,HIGH);
  digitalWrite(pinRSpeed,HIGH);
  delay(300);
}
void newReverse() // spin wheels backward, wait for a bit, stop, do full scan.
{       
  analogWrite(pinLSpeed,BackwardWheelSpeed);
  analogWrite(pinRSpeed,BackwardWheelSpeed);
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);
  delay(1400);
  newStop();
  checkFullScan(1.5);
}
void scan(int d) // check surroundings one step per loop
{
  if(SensorAngles[scanCounter] <= 180 && SensorAngles[scanCounter] >= 40) // Check if angles fall within safe limits
  {
    myservo[0].write(SensorAngles[scanCounter]); // send sensor to said angle
    delay(scanDelay*d);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float angledistance = pulseIn(inputPin, HIGH);
    angledistance= angledistance/5.8/10;
    Serial.println(angledistance);
    scanspeedd = angledistance;
    scanArray[scanCounter] = angledistance;

    Serial.print("Sensor angle: ");
    Serial.print(SensorAngles[scanCounter]);
    Serial.println("");

    Serial.print("Scan Value: ");
    Serial.print(scanArray[scanCounter]);
    Serial.println("");
  }else
  {
    Serial.println("SENSOR ANGLES NOT SAFE TO TRY"); // if angle does not fall within the interval [40, 180] degrees, they are not safe to try.
    Serial.println("UNSAFE ANGLE: ");
    Serial.println(SensorAngles[scanCounter]);
  }
  if((scanCounter+1) >= scanTimes){
  descend = true;
  }
  if(scanCounter == 0){
    descend = false;
  }
  if (descend){
  scanCounter = scanCounter - 1;
  }else{
  scanCounter++;
  }
}
void calculateAction()
{
  // block for collecting information
  int distance0 = scanArray[0];
  int distance1 = scanArray[1];
  int distance2 = scanArray[2];
  int distance3 = scanArray[3];
  int distance4 = scanArray[4];
  Serial.print("Distances L->R: ");
  Serial.print(distance0);
  Serial.print(", ");
  Serial.print(distance1);
  Serial.print(", ");
  Serial.print(distance2);
  Serial.print(", ");
  Serial.print(distance3);
  Serial.print(", ");
  Serial.print(distance4);
  Serial.println("");

  // block for acting on information
  if(distance0 < 15 || distance1 < 30 || distance2 < 30 || distance3 < 30 || distance4 < 15) // if stuff is to close to car, stop and then reverse away to point to side that has open space
  {
    Serial.println("Stop and reverse");
    newStop();
      if((distance0+distance1)/2 < (distance3+distance4)/2)
      {
        turnWheelLeft();
      }else if((distance0+distance1)/2 > (distance3+distance4)/2)
      {
        turnWheelRight();
      }else
      {
        turnWheelCenter();
      }
      newReverse();
      newStop();
      turnWheelCenter();
      delay(300);
      return;
  }else
  {
    if(distance1 < 40 || distance2 < 60 || distance3 < 40) // If stuff isn't too close to the car, turn wheels as required
    {
      if((distance0+distance1)/2 < (distance3+distance4)/2) // If more space is avalable to the right, turn wheels right
      {
        turnWheelRight();
      }else if((distance0+distance1)/2 > (distance3+distance4)/2) //If more space is avalable to the left, turn wheels left
      {
        turnWheelLeft();
      }else
      {
        turnWheelCenter();
      }
    }else
    {
      turnWheelCenter();
    }
  }
  if(distance0 > 20 && distance1 > 60 && distance2 > 80 && distance3 > 60 && distance4 > 20) // If stuff far away from the car, drive fast.
    {
     Serial.println("Gotta go fast");
      newAdvanceFast();
      return;
    }else // drive 
    {
    Serial.println("Go");
    newAdvance();
    return;
    }
}
void loop()  // Main loop from where the program is run. 
{   
  checkLightLevel(); // Call Brightness function
  if(lightCutOff) // If it's too dark, the car will not call other loops except newStop()
    {
    newStop();
    }else
    {
    scan(1); // scan once
    calculateAction(); // calculate the action the car should take based on scan data, as well as execute that action
    }
}
