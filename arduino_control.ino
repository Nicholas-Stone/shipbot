/** WIRING GUIDE **
 *  Initial Pins
 *  
STOP SWITCHES:
front: A0, GND
side: A1, GND

start switch: 

VERT1. MOTOR CONTROLLERS:
Enable Pin (White): D6
Rev. Pin (Grey): D5
Fwd. Pin (Orange): D7

VERT1. MOTOR CONTROLLERS:
Enable Pin (White): D6
Rev. Pin (Grey): D5
Fwd. Pin (Orange): D7

LAT1. MOTOR CONTROLLERS:
Enable Pin (White): D3
Rev. Pin (Grey): D2
Fwd. Pin (Orange): D4

LAT2. MOTOR CONTROLLERS:
Enable Pin (White): D3
Rev. Pin (Grey): D2
Fwd. Pin (Orange): D4

 * SD CARD READER:
 * CS Pin: D53
 * DI Pin: D51
 * DO Pin: D50
 * CLK Pin: D52
GND Pin: GND
5V Pin: 5V

VISION:
CLK PIN (P4 - YELLOW): 20
Data Pin (P5 - WHITE): 21 
Analog: A5

STEPPER MOTOR CONTROLLER:
Step Pin: 13
Direction Pin: 7

DATA SENT FROM CAMERA:

  V1 orientation (V or H or N)
+ V3 orientation (V or H or N)
+ Centered (L or R or C)
+ angle
+ V1H arm position (O or C)
+ V1V arm position (O or C)

ex.
VHC


***********************/
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Stepper.h>

/** PINS **/
//ENCODERS PINS (not constant!)
int vert1EncChnA = 22; //NO
int vert1EncChnB = 23; //NO
int vert2EncChnA = 24; //WORKS
int vert2EncChnB = 25; //WORKS
int lat2EncChnA  = 26; //WORKS
int lat2EncChnB  = 27; //WORKS
const int rotateEncChaA = lat2EncChnA;
const int rotateEncChaB = lat2EncChnB;
int lat1EncChnA  = 28; //WORKS
int lat1EncChnB  = 29; //WORKS

//LIMIT SWITCH PINS
int rightSwitch = A0;
int leftSwitch  = A1;
int frontSwitch  = A2;
int backSwitch = A3;
int startSwitch = A4;

//VERTICAL bodyS H-BRIDGE PINS
int vertHBridgeEnablePin = 8;
int vert1FwdPin = 9;
int vert1RevPin = 12;
int vert2FwdPin = 10;
int vert2RevPin = 11;

//LATERAL bodyS H-BRIDGE PINS
int latHBridgeEnablePin = 2;
int lat1FwdPin = 3;
int lat1RevPin = 5;
int lat2FwdPin = 6;
int lat2RevPin = 4;

//STEPPER DRIVER PINS
int stepPin = 13;
int dirPin = 7;
Stepper motor(200, 13, 7);

//MICROSD CARD READER
int microSDPin = 53;

/** GLOBALS **/
//DRIVE body STATES
int vert1FwdState = LOW;
int vert1RevState = LOW;
int vert2FwdState = LOW;
int vert2RevState = LOW;
int lat1FwdState  = LOW;
int lat1RevState  = LOW;
int lat2FwdState  = LOW;
int lat2RevState  = LOW;

//SWITCH STATE
int frontSwitchState = 0; 
int rightSwitchState = 0; 
int leftSwitchState = 0;
bool switchState; 

//ENCODER 
int initEncoderValueA;
int initEncoderValueB;
int prevEncoderValueA;
int prevEncoderValueB;
int encoderValueA = 0;
int encoderValueB = 0;

//SHIPBOT
const float subcount2Distance = 0.1021; //inches
const int subcount2Count = 120; //2045; //pulses
float xCurrPosition; //inches
float yCurrPosition;
float bodyMaxPosition = 50;
int locoDirectionX = 1;
int locoDirectionY = 1;
int bodySpeed = 175; //0 - 255 (PWM)
int rotationSpeed = 200;
float rotateInches = 13;
int rotationDirection = 1;
bool isRotated = false;
int counterSpinSpeed = 72;  //speed for correcting locomotion

//RESET 
bool zeroPos = false;

//STEPPER
int numSteps = 0;
int maxLeftStepper = -160;
int maxRightStepper = 320;
int rotations = 0;

//FILE PARSING
File missionFile;
String fileName = "test4.txt";
String missionStr = "";
int numTasks = 0;
String missionTasks[] = {"","","","","","","",""};
String angle = "";
int initAngle = 0;
int totAngle = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  //VERT1. body ENCODER
  pinMode(vert1EncChnA,INPUT_PULLUP);
  pinMode(vert1EncChnB,INPUT_PULLUP);
  
  //VERT2. body ENCODER
  pinMode(vert2EncChnA,INPUT_PULLUP);
  pinMode(vert2EncChnB,INPUT_PULLUP);
  
  //LAT2. body ENCODER
  pinMode(lat2EncChnA,INPUT_PULLUP);
  pinMode(lat2EncChnB,INPUT_PULLUP);
  
  //LIMIT SWITCHES
  pinMode(frontSwitch, INPUT_PULLUP); 
  pinMode(backSwitch, INPUT_PULLUP); 
  pinMode(leftSwitch, INPUT_PULLUP); 
  pinMode(rightSwitch, INPUT_PULLUP);
  pinMode(startSwitch, INPUT_PULLUP);  
  
  //H-BRIDGE PWM
  analogWrite(vertHBridgeEnablePin,0); //PWM full duty cycle
  analogWrite(latHBridgeEnablePin,0); //PWM full duty cycle

  //pin test
  digitalWrite(vert1FwdPin,LOW);
  digitalWrite(vert1RevPin,LOW);
  digitalWrite(lat1FwdPin,LOW);
  digitalWrite(lat1RevPin,LOW);
  digitalWrite(vert2FwdPin,LOW);
  digitalWrite(vert2RevPin,LOW);
  digitalWrite(lat2FwdPin,LOW);
  digitalWrite(lat2RevPin,LOW);

  //MICROSD CARD READER
  pinMode(microSDPin, OUTPUT);

  //CAMERA GPIO CONNECTION
  pinMode(A5, OUTPUT);
  
  //initialize SD card reader
  initSD();

  //read mission file into a string
  missionStr = readFile(fileName);
  delay(1000);
  //Put each task into an array
  getTasks(missionStr, ',');

  //STEPPER SETUP
  motor.setSpeed(12000);

  /*********************************/
  /********** MAIN CODE ************/
  /*********************************/

  while(digitalRead(startSwitch) != 0){
    digitalWrite(A5, HIGH);
    delay(100);
    digitalWrite(A5, LOW);
    getCameraData();
  }

   if(zeroPos == false) {
     zeroPosition();
     delay(500);
   }

  for(int i = numTasks-1; i >= 0; i--){
     String station = String(getStation(i));

     //ROTATE ROBOT IF NEEEDED
     if(((station == "F") ||(station == "G") || (station == "H")) && isRotated == false){
        delay(250);
        moveRoboX(-6);
        moveRoboY(-6);
        rotateRobo(1);
        delay(250);
        zeroPosition();
     }
     //COUNTER-ROTATE ROBOT IF NEEDED
     if((station != "F") && (station != "G") && (station != "H") && (isRotated == true)){
        delay(250);
        moveRoboX(-6);
        moveRoboY(-6);
        //Serial.println("ROTATE");
        rotateRobo(-1);
        zeroPosition();
     }

     //MOVE ROBOT IN FRONT OF STATION
      moveRoboY(-2);
      delay(500);
      moveRoboX(getXPos(getStation(i))-xCurrPosition);
      moveRoboY(4);
      moveRoboY(-2);
      moveRoboY(4);
      //delay(1000); //1000**
        
     //STORE STATION BEFORE REFORMATTING GETS RID OF IT
     char stationValue = getStation(i) ; 

     //MOVE LATERAL STEPPER IF AT CORNER OR VALVE THAT NEEDS IT
     moveStepper(i, stationValue);

     //CENTER ARM ON VALVE
     centerObject();
     //zeroStepper(); //DELETE

     //REFORMAT THE MISSION FILE B4 SENDING TO MATLAB
     updateTask(i);
     //Serial.print("TASK: ");
     //Serial.println(missionTasks[i]);

     //SEND MISSION TO MATLAB - HAVE TO SEND TWICE FOR SOME UNKNOWN REASON (FOR MATLAB TO RECIEVE)
     //IF BREAKER DO OWN CODE
     String dev = getDeviceVal(i);
     //Serial.print("DEVICE: ");
     //Serial.println(dev);
     if(dev.charAt(0) == 'B'){
        //STATION A
        if(missionTasks[i].charAt(0) == 'A'){
           if(dev.charAt(1) == '1'){
             //Serial.println("B1");
             rotateStepper(18);
             sendToMatlab(i);
           }
           else if(dev.charAt(1) == '2'){
             //Serial.println("B2");
             rotateStepper(-20);
             sendToMatlab(i);
           }
           else{
             //Serial.println("B3");
             rotateStepper(-74);
             sendToMatlab(i);
           }
        } 
        //STATION B
        else{
          //Serial.println("STATION B");
           if(dev.charAt(1) == '1'){
             //Serial.println("B1");
             rotateStepper(22);
             sendToMatlab(i);
           }
           else if(dev.charAt(1) == '2'){
             //Serial.println("B2");
             rotateStepper(-21);
             sendToMatlab(i);
           }
           else{
             //Serial.println("B3");
             rotateStepper(-70);
             sendToMatlab(i);
           }
        }
     }
     else{
       //check that the valve (V3) isn't already in correct position
      digitalWrite(A5, HIGH);
      delay(100);
      digitalWrite(A5, LOW);
      String cameData = getCameraData();
      if((dev == "V3") && (getDeviceVal(i) == cameData.substring(6, 7))){ 
        //DO NOTHING SKIP
      }
      else{
         sendToMatlab(i);
      }
     }

     //RETURN ARM TO CENTER OF BASE
     zeroStepper();
  }
  delay(300000);
}


void loop() {    
}  

void sendToMatlab(int val){
  //Serial.println("SEND");
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  Serial.println(missionTasks[val]);
  delay(20000);//delay(20000);
}

void moveStepper(int station, char stationVal){
  if(stationVal == 'E'){
    //numSteps -= 32;
    rotateStepper(-32);
    updateTask(station);
    //numSteps += 5;
    rotateStepper(5);  
  }
  else if(stationVal == 'F'){
    //numSteps += 27;
    rotateStepper(27);
  }
  if(stationVal == 'V'){
    //used to further center arm on V1V (for somereason doesnt always center correctly)
    rotateStepper(16);
  }
}

void rotateStepper(int rotations){
   int n = 0;
   while(n <= abs(rotations)){
    if(((numSteps) < maxRightStepper) && ((numSteps) > maxLeftStepper)){
      if(rotations > 0){
        motor.step(1600*1); //6400 = 1 rotation //1 rotation = 5.5mm
        numSteps++;
      }  
      else if(rotations < 0){
        motor.step(1600*-1);
        numSteps--;
      }
      n++;
    }
  }
}

void zeroStepper(){
   //Serial.print("STEPPER ZERO: ");
   //Serial.println(numSteps);
   rotateStepper(-1*numSteps); //6400 = 1 rotation
                                //1 rotation = 5.5mm
   numSteps = 0;                             
}

void centerObject(){
  //Serial.println("CENTER OBJECT");
  digitalWrite(A5, HIGH);
  delay(100);
  digitalWrite(A5, LOW);
  String camData = getCameraData();
  //Serial.print("DATA: ");
  //Serial.println(camData);
  
  while((camData.charAt(0) != 'C')){
    //Serial.println("centering");
    digitalWrite(A5, HIGH);
    delay(100);
    digitalWrite(A5, LOW);
    camData = getCameraData();
    if(camData.charAt(0) == 'R'){
      //Serial.println("MOVE RIGHT FAST");
      rotateStepper(-8); //rotate 1/2 rev //maxLeftStepper = 40;
    }
    else if(camData.charAt(0) == 'E'){
      //Serial.println("MOVE RIGHT MEDIUM");
      rotateStepper(-4); //rotate 1/4 rev //maxLeftStepper = 40;
    }
    else if(camData.charAt(0) == 'H'){
      //Serial.println("MOVE RIGHT SLOW");
      rotateStepper(-1); //rotate 1/4 rev //maxLeftStepper = 40;
    }
    else if(camData.charAt(0) == 'L'){
      //Serial.println("MOVE LEFT FAST");
      rotateStepper(+8);
    }
    else if(camData.charAt(0) == 'T'){
      //Serial.println("MOVE LEFT MEDIUM");
      rotateStepper(4); //rotate 1/4 rev //maxLeftStepper = 40;
    }
    else if(camData.charAt(0) == 'K'){
      //Serial.println("MOVE LEFT SLOW");
      rotateStepper(+1); //rotate 1/2 degree //maxLeftStepper = 40;
    }
  }
  //Serial.println("CENTERED!");
}

 
/** Zero Position:                     
    move robot to 0,0 coordinate in field - i.e. corner of field **/
void zeroPosition(){
    //Serial.println("Zeroing Robo");
    //move robot foward until it hits front rail
    analogWrite(vertHBridgeEnablePin,bodySpeed);
    vert1FwdState = HIGH;
    vert1RevState = LOW;
    vert2FwdState = HIGH;
    vert2RevState = LOW;
    digitalWrite(vert1FwdPin,vert1FwdState);
    digitalWrite(vert1RevPin,vert1RevState);
    digitalWrite(vert2FwdPin,vert2FwdState);
    digitalWrite(vert2RevPin,vert2RevState);
    analogWrite(vertHBridgeEnablePin,bodySpeed);
    
    switchState = false;
    while(switchState == false) {
      switchState = readSwitches(frontSwitch,backSwitch);
    }
    //stop moving foward
    analogWrite(vertHBridgeEnablePin,0);
    vert1FwdState = LOW;
    vert2FwdState = LOW;
    digitalWrite(vert1FwdPin,vert1FwdState);
    digitalWrite(vert2FwdPin,vert2FwdState);
    
    //zero the position
    yCurrPosition = 0;
    
    //move robot right until it hits the side rail
    lat1FwdState = HIGH;
    lat1RevState = LOW;
    lat2FwdState = HIGH;
    lat2RevState = LOW;
    digitalWrite(lat1FwdPin,lat1FwdState);
    digitalWrite(lat1RevPin,lat1RevState);
    digitalWrite(lat2FwdPin,lat2FwdState);
    digitalWrite(lat2RevPin,lat2RevState);
    analogWrite(latHBridgeEnablePin,bodySpeed);

    switchState = false;
    while(switchState == false) {
      switchState = readSwitches(leftSwitch,rightSwitch);
    }
    
    //stop moving right
    lat1FwdState = LOW;
    lat2FwdState = LOW;
    vert1FwdState = LOW;
    vert2FwdState = LOW;
    digitalWrite(lat1FwdPin,lat1FwdState);
    digitalWrite(lat2FwdPin,lat2FwdState);
    analogWrite(latHBridgeEnablePin,0);
    digitalWrite(vert1FwdPin,vert1FwdState);
    digitalWrite(vert2FwdPin,vert2FwdState);
    xCurrPosition = 0;

    //move robot foward until it hits front rail
    analogWrite(vertHBridgeEnablePin,bodySpeed);
    vert1FwdState = HIGH;
    vert1RevState = LOW;
    vert2FwdState = HIGH;
    vert2RevState = LOW;
    digitalWrite(vert1FwdPin,vert1FwdState);
    digitalWrite(vert1RevPin,vert1RevState);
    digitalWrite(vert2FwdPin,vert2FwdState);
    digitalWrite(vert2RevPin,vert2RevState);
    analogWrite(vertHBridgeEnablePin,bodySpeed);
    
    switchState = false;
    while(switchState == false) {
      switchState = readSwitches(frontSwitch,backSwitch);
    }
    //stop moving foward
    analogWrite(vertHBridgeEnablePin,0);
    vert1FwdState = LOW;
    vert2FwdState = LOW;
    digitalWrite(vert1FwdPin,vert1FwdState);
    digitalWrite(vert2FwdPin,vert2FwdState);
        
    //stop moving right
    lat1FwdState = LOW;
    lat2FwdState = LOW;
    vert1FwdState = LOW;
    vert2FwdState = LOW;
    digitalWrite(lat1FwdPin,lat1FwdState);
    digitalWrite(lat2FwdPin,lat2FwdState);
    analogWrite(latHBridgeEnablePin,0);
    digitalWrite(vert1FwdPin,vert1FwdState);
    digitalWrite(vert2FwdPin,vert2FwdState);
    xCurrPosition = 0;
    
    //flag zero position procedure as complete
    zeroPos = true;
}



/** Move Robo:                     
    move robot to x,y coordinate in field 
    1 subcount = 5.625 degrees --> arc length 
    12.25" = wheel circ.
    **/
void moveRoboX(float displacement){
  //Serial.println("MOVE X");
   
   if(displacement == 0) {
    return;
    } else {
    //Serial.println("Move Robo X");
    }
   //MOVE TO X POSITION
   
   //initial position from encoder
   int initEncoderValueA = digitalRead(lat2EncChnA);
   int initEncoderValueB = digitalRead(lat2EncChnB);
   int prevEncoderValueA = initEncoderValueA;
   int prevEncoderValueB = initEncoderValueB;
   
   //check x direction
   if(displacement < 0) {
    //ccw
    lat1FwdState = LOW;
    lat1RevState = HIGH;
    lat2FwdState = LOW;
    lat2RevState = HIGH;
    vert1FwdState = LOW;
    vert1RevState = LOW;
    vert2FwdState = HIGH;
    vert2RevState = LOW;
    locoDirectionX = -1;
    //Serial.println("REVERSE");
   } else {
    lat1FwdState = HIGH;
    lat1RevState = LOW;
    lat2FwdState = HIGH;
    lat2RevState = LOW;
    vert1FwdState = LOW;
    vert1RevState = LOW;
    vert2FwdState = LOW;
    vert2RevState = LOW;
    locoDirectionX = 1;
    
    //Serial.println("FORWARD");
   }
   
   //set direction
   digitalWrite(lat1FwdPin,lat1FwdState);
   digitalWrite(lat1RevPin,lat1RevState);
   digitalWrite(lat2FwdPin,lat2FwdState);
   digitalWrite(lat2RevPin,lat2RevState);
   
   digitalWrite(vert1FwdPin,vert1FwdState);
   digitalWrite(vert1RevPin,vert1RevState);
   digitalWrite(vert2FwdPin,vert2FwdState);
   digitalWrite(vert2RevPin,vert2RevState);
   
   //set speed
   analogWrite(latHBridgeEnablePin, bodySpeed);
   if(isRotated != true){
      analogWrite(vertHBridgeEnablePin, counterSpinSpeed);
   }
   xCurrPosition = readBodySensors(displacement,locoDirectionX,xCurrPosition,lat2EncChnA,lat2EncChnB,leftSwitch,rightSwitch);
   //reset counter and stop moving sideways
   lat1FwdState = LOW;
   lat1RevState = LOW;
   lat2FwdState = LOW;
   lat2RevState = LOW;
   vert1FwdState = LOW;
   vert1RevState = LOW;
   vert2FwdState = LOW;
   vert2RevState = LOW;
   
   digitalWrite(lat1FwdPin,lat1FwdState);
   digitalWrite(lat1RevPin,lat1RevState);
   digitalWrite(lat2FwdPin,lat2FwdState);
   digitalWrite(lat2RevPin,lat2RevState);
   digitalWrite(vert1FwdPin,vert1FwdState);
   digitalWrite(vert1RevPin,vert1RevState);
   digitalWrite(vert2FwdPin,vert2FwdState);
   digitalWrite(vert2RevPin,vert2RevState);
}

void moveRoboY(float displacement){
   
   if(displacement == 0) {
    return;
    } else {
    //Serial.println("Move Robo Y");
    }
   //MOVE TO Y POSITION
   
   //initial position from encoder
   int initEncoderValueA = digitalRead(vert2EncChnA);
   int initEncoderValueB = digitalRead(vert2EncChnB);
   int prevEncoderValueA = initEncoderValueA;
   int prevEncoderValueB = initEncoderValueB;
   
   //check x direction
   if(displacement < 0) {
    //ccw
    vert1FwdState = LOW;
    vert1RevState = HIGH;
    vert2FwdState = LOW;
    vert2RevState = HIGH;
    locoDirectionY = -1;
    //Serial.println("REVERSE");
   } else {
    vert1FwdState = HIGH;
    vert1RevState = LOW;
    vert2FwdState = HIGH;
    vert2RevState = LOW; 
    locoDirectionY = 1;
    //Serial.println("FORWARD");
   }
   
   //set direction
   digitalWrite(vert1FwdPin,vert1FwdState);
   digitalWrite(vert1RevPin,vert1RevState);
   digitalWrite(vert2FwdPin,vert2FwdState);
   digitalWrite(vert2RevPin,vert2RevState);
   //set speed
   analogWrite(vertHBridgeEnablePin, 150);
   yCurrPosition = readBodySensors(displacement,locoDirectionY,yCurrPosition,vert2EncChnA,vert2EncChnB,frontSwitch,backSwitch);
   //reset counter and stop moving sideways
   vert1FwdState = LOW;
   vert2FwdState = LOW;
   digitalWrite(vert1FwdPin,vert1FwdState);
   digitalWrite(vert2FwdPin,vert2FwdState);
   vert1RevState = LOW;
   vert2RevState = LOW;
   digitalWrite(vert1RevPin,vert1RevState);
   digitalWrite(vert2RevPin,vert2RevState);
   analogWrite(vertHBridgeEnablePin,0);
}

//Read the base's encoder values
//until the number of pulses has been reached 
//OR a switch has been activated
//OR a maximum distance has been reached
//OR vision has indicated to stop TODO
float readBodySensors(float displacementValue,int directionValue, float currPosition, int pinChannelA,int pinChannelB,int switch1,int switch2)
{
  int counter = 0;
  int subcounter = 0;
  float distanceCounter = 0;
  bool switchState = false;
  while((distanceCounter < abs(displacementValue)) && //reached a point
    (switchState == false) && //triggered a switch
    (currPosition < bodyMaxPosition))//max distance threshold TODO fix the issue when this is reached
    {  
    if(distanceCounter > 2) {
      switchState = readSwitches(switch1,switch2);
    }
    //read encoder value
     encoderValueA = digitalRead(pinChannelA);
     encoderValueB = digitalRead(pinChannelB);
     //Serial.print("ENNCODER A: ");
     //Serial.println(encoderValueA);
     //Serial.print("ENNCODER B: ");
     //Serial.println(encoderValueB);
  
     if((encoderValueA == initEncoderValueA) //detect cycle
          && (encoderValueB == initEncoderValueB)
          && (encoderValueA != prevEncoderValueA || encoderValueB != prevEncoderValueB)
          ){
        subcounter++;
        //Serial.print("SUBCOUNTER: ");
        //Serial.ln(subcounter);
        delay(10);
        distanceCounter = distanceCounter + subcount2Distance;
        currPosition = currPosition + (directionValue*subcount2Distance); 
        if(subcounter == subcount2Count) {
//          Serial.print("COUNTER: ");
//          Serial.println(counter);          
          counter++;
          subcounter = 0;
        }
     }
     prevEncoderValueA = encoderValueA;
     prevEncoderValueB = encoderValueB;
   }
  return currPosition;
}

bool readSwitches(int switch1,int switch2) {
      
      int switch1Value = digitalRead(switch1);
      int switch2Value = digitalRead(switch2);
        //TODO investigate why this println statement is necessary
        //Serial.println(switch1Value);
        //debounce switch
        if((switch1Value == 0) || (switch2Value == 0)) {
          delay(50);
          switch1Value = digitalRead(switch1);
          switch2Value = digitalRead(switch2);
        }
        if((switch1Value == 0) || (switch2Value == 0)) {
            return true;
          } else {
            return false;
         }   
  
}


void rotateRobo(int direction){ 
   
   if(direction == 0){
    return;
   }
   
   //initialize encoder
   int initEncoderValueA = digitalRead(rotateEncChaA); // const to ensure working
   int initEncoderValueB = digitalRead(rotateEncChaB);
   int prevEncoderValueA = initEncoderValueA;
   int prevEncoderValueB = initEncoderValueB;
   
   //set direction
   if(direction < 0) { //ccw
     digitalWrite(lat1FwdPin,LOW);
     digitalWrite(lat2RevPin,LOW);
     digitalWrite(vert1RevPin,LOW);
     digitalWrite(vert2FwdPin,LOW);
     digitalWrite(lat1RevPin,HIGH);
     digitalWrite(lat2FwdPin,HIGH);
     digitalWrite(vert1FwdPin,HIGH);
     digitalWrite(vert2RevPin,HIGH);
   } else { //unsure about this //cw
     digitalWrite(lat1FwdPin,LOW);
     digitalWrite(lat2RevPin,LOW);
     digitalWrite(vert1RevPin,HIGH);
     digitalWrite(vert2FwdPin,HIGH);
     digitalWrite(lat1RevPin,HIGH);
     digitalWrite(lat2FwdPin,HIGH);
     digitalWrite(vert1FwdPin,LOW);
     digitalWrite(vert2RevPin,LOW);
   }
 
   analogWrite(latHBridgeEnablePin, rotationSpeed);
   analogWrite(vertHBridgeEnablePin, rotationSpeed);
   readBodySensors(rotateInches,0,0,rotateEncChaA,rotateEncChaB,leftSwitch,rightSwitch);
   
   //reset counter and stop rotation
   vert1FwdState = LOW;
   vert2FwdState = LOW;
   digitalWrite(vert1FwdPin,vert1FwdState);
   digitalWrite(vert2FwdPin,vert2FwdState);
   vert1RevState = LOW;
   vert2RevState = LOW;
   digitalWrite(vert1RevPin,vert1RevState);
   digitalWrite(vert2RevPin,vert2RevState);
   lat1FwdState = LOW;
   lat2FwdState = LOW;
   digitalWrite(lat1FwdPin,lat1FwdState);
   digitalWrite(lat2FwdPin,lat2FwdState);
   lat1RevState = LOW;
   lat2RevState = LOW;
   digitalWrite(lat1RevPin,lat1RevState);
   digitalWrite(lat2RevPin,lat2RevState);

   analogWrite(latHBridgeEnablePin,0);
   analogWrite(vertHBridgeEnablePin,0);

   updateOrientation(direction,vert1EncChnA,vert1EncChnB,vert2EncChnA,vert2EncChnB,lat1EncChnA,lat1EncChnB,lat2EncChnA,lat2EncChnB,
        vertHBridgeEnablePin,vert1FwdPin,vert1RevPin,vert2FwdPin,vert2RevPin,
        latHBridgeEnablePin,lat1FwdPin,lat1RevPin,lat2FwdPin,lat2RevPin,
        rightSwitch,leftSwitch,frontSwitch,backSwitch);

}
//this is right, direction maybe swapped
void updateOrientation(int direction,int vert1EncChnA_IN,int vert1EncChnB_IN,int vert2EncChnA_IN,int vert2EncChnB_IN,int lat1EncChnA_IN,int lat1EncChnB_IN,int lat2EncChnA_IN,int lat2EncChnB_IN,
        int vertHBridgeEnablePin_IN,int vert1FwdPin_IN,int vert1RevPin_IN,int vert2FwdPin_IN,int vert2RevPin_IN,
        int latHBridgeEnablePin_IN,int lat1FwdPin_IN,int lat1RevPin_IN,int lat2FwdPin_IN,int lat2RevPin_IN,
        int rightSwitch_IN,int leftSwitch_IN,int frontSwitch_IN,int backSwitch_IN) 
        {
        if(direction == 0) {
          return;
        }

        isRotated = !isRotated;

        if(direction < 0) { //ccw
            lat1FwdPin = lat1RevPin_IN;
            lat1RevPin = lat1FwdPin_IN;
            lat2FwdPin = lat2RevPin_IN;
            lat2RevPin = lat2FwdPin_IN;
     
        } else { //cw
            lat1FwdPin = lat1RevPin_IN;
            lat1RevPin = lat1FwdPin_IN;
            lat2FwdPin = lat2RevPin_IN;
            lat2RevPin = lat2FwdPin_IN; 
        }
}

/* GET TASK:
 *  
 * RECIEVES: Entire mission string 
 * RETURNS: Each task within string in an array
 *          Also sets number of tasks to complete
 * 
 * EX: RX: "AV1 175, CA B2, DV3 0, 132"
 *     TX:  tasks = {"AV1 175", "CA B2", "DV3 0", "132"}
 *
 * Assumes: at least one task sent
 */
void getTasks(String mission, char separator){
  int strLen = mission.length();
  int prevSubStrEnd = 0;
  numTasks = 0;
  
  //add all the tasks to array
  for(int i = 0; i <= strLen; i++){
    if(mission.charAt(i) == separator){
      missionTasks[numTasks] = mission.substring(prevSubStrEnd, i);
      prevSubStrEnd = i+2;
      numTasks++;
    }
  }
  
  //Serial.println("Number of tasks to complete:" + numTasks);
  
  //Add time to array
  missionTasks[numTasks] = mission.substring(prevSubStrEnd, strLen);
}

/* READ FILE:
 *  
 * RECIEVES: name of file to read
 * SETS: missionStr to the contents of the file 
 * 
 */
String readFile(String file){
  String fileStr = "";
  
  //open txt file for reading 
  // *** --> change to file name want to open!!!!
  missionFile = SD.open(file); 

  //check if file opened
  if(missionFile){
    //read from file until there is nothing else to read
    while(missionFile.available()){
      fileStr = (fileStr + char(missionFile.read()));
    }

    //close the file
    missionFile.close();
    Serial.println("Mission Command: " + fileStr);
    return fileStr;
  }
  //if file didnt open
  else{
    Serial.println("ERROR: File did not open!");
  }
}

/* INIT. SD CARD:
 *  
 * Initializes SD card reader 
 * 
 */
void initSD(){
  if(!SD.begin(10)){
    Serial.println("ERROR: SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialization complete");
}

/* GET STATION:
 *  
 * RECIEVES: Task number (which task we are trying to complete)
 * RETURNS: The station we must go to, to complete the task 
 * 
 * EX: RX: 1, given tasks = {"AV1 175", "CA B2", "DV3 0"}
 *     TX: returns A
 *
 * Assumes: at least one task sent
 */
char getStation(int task){
  return missionTasks[task].charAt(0);
}

/* GET DEVICE:
 *  
 * RECIEVES: Task number (which task we are trying to complete)
 * RETURNS: The device we are trying to actuate 
 * USE: To calculate y pos of robot for valve
 * 
 * EX: RX: 1, given tasks = {"AV1 175", "CA B2", "DV3 0"}
 *     TX: returns V1
 *
 */
String getDevice(int task){
  int strLen = missionTasks[task].length();
  int spacePos = missionTasks[task].indexOf(' ');
  return missionTasks[task].substring(1, spacePos);
}

/* GET X POS:
 *  
 * RECIEVES: Station letter 
 * RETURNS: X position of station in inches
 * 
 * EX: RX: A
 *     TX: x-position to station A
 *
 */
float getXPos(char station){
  switch (station) {
         case 'A':
             return -66;
             break;
         case 'B':
             return -49;
             break;
         case 'C':
             return -31;
             break; 
         case 'D':
             return -13.5;
             break;
         case 'E':
             return 0;
             break;
         case 'F':
             return 3;
             break;
         case 'G':
             return -12.5;
             break;
         case 'H':
             return -29.5;
             break;
         default:
             Serial.print("INVALID POSITION!");
      }
}

/* UPDATE TASK:
 *  
 * RECIEVES: Task number 
 * RETURNS: Finds the valve, if it is V1 or V3, it will find the 
 *          orientation of the valve, than reformat the task such 
 *          that it is in the fomatt needed for matlab
 * 
 * Format: V1V~ 350
 *         V1H~ 350
 *         V2~~ 350
 *         V3H0 
 *         V3H1 
 *         V3V0 
 *         V3V1 
 *         A~~~ 350
 *         B~~~ 350
 */
int updateTask(int task){
    String tDevice = getDevice(task);
    //Serial.println("DEVICE: " + tDevice);
    // mdelay(1000);
    
    if(tDevice.equals("V1")){
      digitalWrite(A5, HIGH);
      delay(100);
      digitalWrite(A5, LOW);
      String camData = getCameraData();
      while((camData.charAt(1) != 'V') && (camData.charAt(1) != 'H')){
        //Serial.println("V1 waiting for good camera data");
        digitalWrite(A5, HIGH);
        delay(100);
        digitalWrite(A5, LOW);
        camData = getCameraData();
      } 
      if (camData.charAt(1) == 'V'){
        //Serial.println("V1 VERTICAL");
        missionTasks[task] = String("V1V~ "+getDeviceVal(task));
        moveStepper(0, 'V');
      }
      else if(camData.charAt(1) == 'H'){
        //Serial.println("V1 HORIZONTAL");
        angle = camData.substring(3, 6);
        initAngle = angle.toInt()-100;
        totAngle = getDeviceVal(task).toInt()+initAngle;
        missionTasks[task] = String("V1H~ "+String(totAngle));
      }
      else{
        Serial.println("V1 NA: " + camData.charAt(1));
      }

      /*//CODE FOR SYSTEM DEMO
      char station = getStation(task);
      if(station == 'A'){
        missionTasks[task] = String("V1H~ "+getDeviceVal(task));
      }
      else{
        missionTasks[task] = String("V1V~ "+getDeviceVal(task));
      }*/
    }
  
    else if(tDevice == "V2"){
      digitalWrite(A5, HIGH);
      delay(100);
      digitalWrite(A5, LOW);
      String camData = getCameraData();
      angle = camData.substring(3, 6);
      initAngle = angle.toInt()-100;
      totAngle = getDeviceVal(task).toInt()+initAngle;
      missionTasks[task] = String("V2~~ "+String(totAngle));
    }
  
    else if(tDevice == "V3"){
      digitalWrite(A5, HIGH);
      delay(100);
      digitalWrite(A5, LOW);
      String camData = getCameraData();
      while((camData.charAt(2) != 'V') && (camData.charAt(2) != 'H')){
        //Serial.println("V3 waiting for good camera data");
        digitalWrite(A5, HIGH);
        delay(100);
        digitalWrite(A5, LOW);
        camData = getCameraData();
      }
      if (camData.charAt(2) == 'V'){
        //Serial.println("V3 VERTICAL");
        missionTasks[task] = String("V3V"+getDeviceVal(task));
      }
      else if(camData.charAt(2) == 'H'){
        //Serial.println("V3 HORIZONTAL");
        missionTasks[task] = String("V3H"+getDeviceVal(task));
      }
      else{
        Serial.println("V3 N/A");
      }
      
      /*//CODE FOR SYSTEM DEMO
      char station = getStation(task);
      if(station == 'B'){
        missionTasks[task] = String("V3V"+getDeviceVal(task));
      }
      else{
        missionTasks[task] = String("V3H"+getDeviceVal(task));
      }*/
    }
  
    else if(tDevice == "A"){
      missionTasks[task] = String("A~~~ "+getDeviceVal(task));
    }
  
    else if(tDevice == "B"){
      missionTasks[task] = String("B~~~ "+getDeviceVal(task));
    }
  
    else{
      Serial.println("ERROR: invalid device!");
    }
}


String getCameraData(){
  int32_t temp = 0;
  String data = "";
  //Serial.println("CAMERA DATA");
  digitalWrite(A5, HIGH);
  digitalWrite(A5, LOW);
  Wire.requestFrom(0x12, 2); //FREEZES HERE
  //Serial.println("NOT FROZEN");
  if(Wire.available() == 2) { // got length?
    temp = Wire.read() | (Wire.read() << 8);
    delay(1); // Give some setup time...
    Wire.requestFrom(0x12, temp);
    if(Wire.available() == temp) { // got full message?  
      temp = 0;
      while(Wire.available()){
        data = data + char(Wire.read());
      }
    } 
    else{
      while(Wire.available()) Wire.read(); // Toss garbage bytes.
    }
  } 
  else {
    while(Wire.available()) Wire.read(); // Toss garbage bytes.
  }
   digitalWrite(A5, LOW);
   return data;
   delay(1); // Don't loop to quickly.
}


/* GET DEVICE VALUE:
 *  
 * RECIEVES: mission task 
 * RETURNS: second string in mission task (amount to rotate)
 * 
 *
 */

String getDeviceVal(int task){
    int strLen = missionTasks[task].length();
    int spacePos = missionTasks[task].indexOf(' ')+1;
    return missionTasks[task].substring(spacePos, strLen);
}