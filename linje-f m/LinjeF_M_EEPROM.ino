//Importering av nødvendige bibliotek
#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>                                        //ny

#define EEPORM_SIZE 1

// Definerer maksimalhastigheten til zumo-bilen
const int16_t maxSpeed = 190; //pleide å være 400

uint16_t cont = 0;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4LCD display;
Zumo32U4Encoders encoders;                                 // ny
Zumo32U4ButtonA buttonC;                                   // ny
Zumo32U4ProximitySensors proxSensors; // til prox sensor

int16_t lastError = 0; //Variabel for korrigering av feilbevegelse 
int x;

int hurtigheit;

unsigned int lineSensorValues[2];

int countspermeter = 8195;
unsigned long now;
unsigned long wait;

int StopMillis = 0;

int  strom = EEPROM.read(0);//mAh Strøm                    //lagt til ny
//kan sette knapper til å assigne størm en eller anna verdi. 
 
float use_battery(){
  strom = strom - (2 * hurtigheit);
  return strom;
  }

 /*
float hurtighetms(){
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
  
  now = millis();
  wait = millis();
  while(now-wait < tid){ . //tatt bort now wait, while og now
    now = millis();
    } 
  int x = (encoders.getCountsLeft() + encoders.getCountsRight()) / 2;
  int distance = (x/countspermeter)*100;
  int hurtigheit = distance / (tid/1000);
  return hurtigheit; 

  */

void ResetStrom() {
  if (buttonC.isPressed())
  {
  //  EEPROM.write(1, 1000);                          alternativ hvis ikke stom lagres automatisk i eeprom
    strom = 1000; //mah                              //ny strom som derretter skal lagres i EEPROM
  }
}

const int period = 100;
unsigned long lastMillis = 0;

//Funksjon som står for kalibereringen 
void Calibrate(){
  for (cont = 0; cont < 100; cont++){
  
    
    if (cont <= 20 || cont >= 70) {
      motors.setSpeeds(150, -150);
      }
    if (cont >= 25 && cont <= 65) {
      motors.setSpeeds(-150, 150);
     
      }
       lineSensors.calibrate();
    }
    motors.setSpeeds(0, 0);
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
  unsigned long currentMillis = millis();
  int16_t position = lineSensors.readLine(lineSensorValues);
  //leser av linje verdien 
  // 0 = linje under 1. sensor
  //1000 = linje under center sensor
  //2000 = linje under 3. sensor 

  int16_t error = position - 1000; //Feilestimat 
  /*
  display.clear();
  display.print(position);
  */
  
  int16_t speedDifference = error / 4 + 6 * (error - lastError);
  
  lastError = error;
  
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  
  motors.setSpeeds(leftSpeed, rightSpeed);

  int Left_encoder = encoders.getCountsRight(); //Kan få overflow  (over 16 bit)
  int Right_encoder = encoders.getCountsLeft();

  float  x = ((Left_encoder + Right_encoder) / 2);
  float  distance = (x/countspermeter)*100; //måler distanse den har kjørt i cm. 

  float StartMillis = millis();
  float hurtigheit = distance/(StartMillis - currentMillis);

  ResetStrom();
  

  /*
  display.clear();
  display.print(distance); */


  if (hurtigheit > 0) {
    float battery = use_battery();
    /*
    display.clear();
    display.print(battery);*/
    display.clear();
   // display.gotoXY(0, 0);
    display.print(battery);
    if(battery<= 0){
      while(true){
        delay(500);
        display.clear();
        display.print("Bat low");
        delay(500);
        display.clear();
        motors.setLeftSpeed(0); 
        motors.setRightSpeed(0); 
      }
    }
  }



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
    lastMillis = millis(); 
}}}
