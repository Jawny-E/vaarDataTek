//Importering av nødvendige bibliotek
#include <Wire.h>
#include <Zumo32U4.h>

// Definerer maksimalhastigheten til zumo-bilen
const uint16_t maxSpeed = 400;

uint16_t cont = 0;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4LCD display;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors; // til prox sensor

int16_t lastError = 0; //Variabel for korrigering av feilbevegelse 
int x;

uint16_t leftSpeed;
uint16_t rightSpeed; 

unsigned int lineSensorValues[2];

int countspermeter = 8195;
unsigned long now;
unsigned long wait;
int tid = 200;
int  strom = 1200;//mAh Strøm

float use_battery(){
  int x = hurtighetms();
  strom = strom - (2*x + 10);
  return strom;
  }
float hurtighetms(){
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
  now = millis();
  wait = millis();
  while(now-wait < tid){
    now = millis();
    } 
  int x = (encoders.getCountsLeft() + encoders.getCountsRight()) / 2;
  int distance = (x/countspermeter)*100;
  int hurtigheit = distance / (tid/1000);
  return hurtigheit; 
}

const int period = 100;
unsigned long lastMillis = 0;

//Funksjon som står for kalibereringen 
void Calibrate(){
  for (cont = 0; cont < 100; cont++){
    delay(100);
  

    if (cont >= 25 || cont >= 85) {
      leftSpeed = -200;
      rightSpeed = 200;
      motors.setSpeeds(leftSpeed, rightSpeed);
      }
    
    if (20 >= cont || cont <= 80) {
      leftSpeed = -200;
      rightSpeed = 200;
      motors.setSpeeds(leftSpeed, rightSpeed);
      }

      delay(100);
      motors.setSpeeds(0,0);
    }
    lineSensors.calibrate();
    }



void setup()
{
  Serial.begin(9600);
  display.clear();
  motors.setSpeeds(0,0);
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
  Calibrate(); 
  motors.setSpeeds(0,0);  
}


void loop() 
{
  float x = hurtighetms();
  display.clear();
  display.print("hei");
  if(x>0){
    float battery = use_battery();
    display.gotoXY(0, 0);
    display.print(battery);
    if(battery<= 0){
      while(true){
        delay(500);
        display.print("fail");
        delay(500);
        display.clear();
        motors.setLeftSpeed(0); 
        motors.setRightSpeed(0); 
      }
    }
  }

  unsigned long currentMillis = millis();
  int16_t position = lineSensors.readLine(lineSensorValues);//Mål for posisjsonen til bilen
  int16_t error = position - 1000; //Feilestimat 
  int16_t speedDifference = error / 4 + 6 * (error - lastError);

  lastError = error;

  leftSpeed = maxSpeed + speedDifference;
  rightSpeed = maxSpeed - speedDifference;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);


if (currentMillis-lastMillis > period) 
{
    
    proxSensors.read();
   int center_left_sensor = proxSensors.countsFrontWithLeftLeds(); 
   //int center_right_sensor = proxSensors.countsFrontWithRightLeds();

   while (( center_left_sensor == 6 ) && ( leftSpeed > 0 &&  rightSpeed > 0 )) { 
           //forsøker å stoppe dersom den ser en objekt

       leftSpeed = 1 ;
       Serial.println(leftSpeed);
       rightSpeed = 1;
       motors.setSpeeds(leftSpeed, rightSpeed);
       

    proxSensors.read();
    center_left_sensor = proxSensors.countsFrontWithLeftLeds();
    //center_right_sensor = proxSensors.countsFrontWithRightLeds();
    display.clear();
    display.gotoXY(0,1);
    display.print(center_left_sensor);
lastMillis = millis(); 
}}}
