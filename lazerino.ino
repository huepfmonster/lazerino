/***************************************************
Lazerino Project by play:vienna
Calibration and Beam detection code taken from https://www.instructables.com/LASER-Maze-2012-Halloween-Haunted-House/
 ****************************************************/

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "JC_Button.h"

SoftwareSerial mySoftwareSerial(10,11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
Button startButton(4);
Button stopButton (5);
Button testButton (6);
void printDetail(uint8_t type, int value);
bool gameRunning = false;
bool testMode = false;
bool singlePlayer=true;

const int laserPins[] =  {7,8};// the number of the pins to turn the lasers on and off

// Variables will change:
int laserPinState = LOW;             // ledState used to set the LED

// constants to set up pins  
int mySensors[] = {A0,A1,A2,A3,A4,A5};    // analog pins with CDS cells to detect laser beam
const int numSensors = 6;   // total number of CDS cells used ###MODIFY IF NEEDED###

// variables to handle analog values from CDS cells
int curValue;               // stores the value of the CDS cell that is currently being read
int prevValue[numSensors];  // stores the previous value of each CDS cell
int baseValue[numSensors];  // stores a baseline value for each CDS cell when the laser beam is not broken
int sensitivity = 40;       // an offset from the baseValue to determine if laser beam is broken ###MODIFY IF NEEDED###

//variables to handle Error Timers
unsigned long startMillisPlayer1;    
unsigned long startMillisPlayer2;    
unsigned long currentMillis;
unsigned long startMillis;
const unsigned long period = 5000;           // penalty time for mistake
bool player1Mistake=false;
bool player2Mistake=false;
bool playMistake=false;



//Variables for debug logging in test mode
unsigned long previousMillisDebug = 0;        // will store last time LED was updated
// constants won't change:
const long debugInterval = 1000;           // interval at which to blink (milliseconds)


void setup()
{
  startButton.begin();
  stopButton.begin();
  testButton.begin();
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  gameRunning=false;

  // set all of the CDS cell analog pins as inputs
  for (int thisSensor = 0; thisSensor < numSensors; thisSensor++)  {
    pinMode(mySensors[thisSensor], INPUT);     
  }
  
  for (int thisLaser = 0; thisLaser < 2; thisLaser++)  {
    pinMode(laserPins[thisLaser], OUTPUT);
    digitalWrite(laserPins[thisLaser], HIGH);
  }

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));
   myDFPlayer.volume(20);  //Set volume value. From 0 to 30
}

void loop()
{
  stopButton.read();
  startButton.read();
  testButton.read();
  currentMillis=millis();
  uint8_t dfPlayerState=myDFPlayer.readType();
  if (gameRunning==false and startButton.wasReleased()) {
    singlePlayer=true;
    startGame();
    Serial.println(F("Singleplayer Game started")); 
  }
  else if (gameRunning==false and startButton.pressedFor(2000)) {
    singlePlayer=false;
    startMillisPlayer1=0;    
    startMillisPlayer2=0; 
    startGame();
    Serial.println(F("2-Player Game started")); 
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  if (playMistake==true && (currentMillis - startMillis) >= period) {
        playMistake=false;
        startMillis=0;
        Serial.println(F("timer ended, game resumed")); 
        startGame();
  }
  
  if (player1Mistake==true && (currentMillis - startMillisPlayer1) >= period) {
        startMillisPlayer1=0;       
        digitalWrite(laserPins[0], LOW);
        myDFPlayer.advertise(3);
        delay(1000);
        player1Mistake=false;
        Serial.println(F("Timer  Player 1 ended, game resumed for Player 1")); 

  }

  if (player2Mistake==true && (currentMillis - startMillisPlayer2) >= period) {
        startMillisPlayer2=0;
        digitalWrite(laserPins[1], LOW);
        myDFPlayer.advertise(4);
        delay(1000);
        player2Mistake=false;
        Serial.println(F("Timer  Player 2 ended, game resumed for Player 2")); 
  }

  else if ((gameRunning==true) and (stopButton.wasPressed())) {
     winGame();
  }

  if (gameRunning==true){
      checkGameStatus();
    }

  if (testButton.wasPressed()) {
    toggleTest();
  }

    if ((currentMillis - previousMillisDebug >= debugInterval) and testMode==true) {
        previousMillisDebug = currentMillis;
        Serial.println("Sensor 1:");
        Serial.println(analogRead(A0));
        Serial.println("Sensor 2:");
        Serial.println(analogRead(A1));
        Serial.println("Sensor 3:");
        Serial.println(analogRead(A2));
        Serial.println("Sensor 4:");
        Serial.println(analogRead(A3));
        Serial.println("Sensor 5:");
        Serial.println(analogRead(A4));
        Serial.println("Sensor 6:");
        Serial.println(analogRead(A5));
  }
}

void startGame() {
    digitalWrite(laserPins[0], LOW);
    digitalWrite(laserPins[1], LOW);
    delay(1000);
    calibrate();
    myDFPlayer.playMp3Folder(1);
    myDFPlayer.enableLoop();
    gameRunning=true;
  }

void winGame(){
      myDFPlayer.playMp3Folder(3);
      gameRunning=false;
      digitalWrite(laserPins[0], HIGH);
      digitalWrite(laserPins[1], HIGH);
      Serial.println(F("Game won!")); 
  }

void checkGameStatus(){
  // Check each CDS cell to see if the laser beam is broken
  for (int thisSensor = 0; thisSensor < numSensors; thisSensor++)  {
    curValue = analogRead(mySensors[thisSensor]);
    //Serial.println(curValue);
    
   // A broken beam is detected if the current sensor value and
   // the previous value are both less than the base value
   // minus the sensitivity. 
    if (curValue < (baseValue[thisSensor] - sensitivity) && prevValue[thisSensor] < (baseValue[thisSensor] - sensitivity)) {
      if (singlePlayer==true){
      madeMistake();
        }
        else{
        if (thisSensor < 3 && player1Mistake==false) {
          player1Mistake=true;
          madeMistakeMP(0);
        }
        else if (thisSensor > 2 && player2Mistake==false) {
          player2Mistake=true;
          madeMistakeMP(1);
        }
      }
    }
    prevValue[thisSensor] = curValue;  // update the previous value
  }
}

void madeMistake(){
    myDFPlayer.playMp3Folder(2);
    digitalWrite(laserPins[0], HIGH);
    digitalWrite(laserPins[1], HIGH);
    gameRunning=false;
    startMillis=millis();
    playMistake=true;
    Serial.println(F("Mistake made, countdown started")); 
  }

int madeMistakeMP(int playerNum){
    digitalWrite(laserPins[playerNum], HIGH);
    if (playerNum==0){
        startMillisPlayer1=millis();
        myDFPlayer.advertise(1);
        Serial.println(F("Player 1 mistake made, countdown started")); 
    }
    if (playerNum==1){  
      startMillisPlayer2=millis();
      myDFPlayer.advertise(2);
      Serial.println(F("Player 2 mistake made, countdown started")); 
    }
  }

  void calibrate() {
  // for each CDS cell, average three readings together to 
  // calculate base value when laser beam is not broken
  for (int thisSensor = 0; thisSensor < numSensors; thisSensor++)  {
    baseValue[thisSensor] = analogRead(mySensors[thisSensor]);    // first reading
    delay(20);
    baseValue[thisSensor] += analogRead(mySensors[thisSensor]);   // plus second reading
    delay(20);
    baseValue[thisSensor] += analogRead(mySensors[thisSensor]);   // plus third reading
    baseValue[thisSensor] /= 3;                                   // divided by 3 to find average
    //Serial.println(thisSensor);
    //Serial.println(baseValue[thisSensor]);
  }
}

void toggleTest(){
  if (testMode==false){
        digitalWrite(laserPins[0], LOW);
        digitalWrite(laserPins[1], LOW);
        testMode=true;
        gameRunning = false;
        Serial.println(F("Test Mode turned on"));
        startMillisPlayer1=0;    
        startMillisPlayer2=0; 
    }
  else {
        digitalWrite(laserPins[0], HIGH);
        digitalWrite(laserPins[1], HIGH);
        myDFPlayer.pause();
        Serial.println(F("Test Mode turned off"));
        testMode=false;
  }
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
