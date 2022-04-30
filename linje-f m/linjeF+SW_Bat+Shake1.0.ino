//Importering av nødvendige bibliotek
#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>                                        //ny
#include <zumo32U4IMU.h>

// #define EEPORM_SIZE 1

// Definerer maksimalhastigheten til zumo-bilen
const int16_t maxSpeed = 190; //pleide å være 400

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4LCD display;                                       // kan ta bort
Zumo32U4Encoders encoders;                                 // ny   
Zumo32U4ButtonA buttonA;                                   // ny   
Zumo32U4ProximitySensors proxSensors; // til prox sensor
Zumo32U4IMU imu;

uint16_t cont = 0;
float strom;
int forrige_tid = 0;
int16_t lastError = 0; //Variabel for korrigering av feilbevegelse 
int x;
int hurtigheit;
unsigned int lineSensorValues[2];
int countspermeter = 8195;
unsigned long now;
unsigned long wait;
int StopMillis = 0;
// battery;
//leftSpeed;
//rightSpeed;
const int period = 100;
unsigned long lastMillis = 0;

 
// Alt som har med angle å gjøre
float gyroOffsetY; //henter Y aksen på gyro
uint16_t angle = -90; // angle kofigureres til å bare vare basert på gyro
float aAngle =0;     // aAngle konfigureres til å bare vare basert på accelrometer
// vi bruker akselrometer senere for å klibrere gyroskopet korrekt slik at vi minsker 
// feil estimat. Av erfaring vil verdien skli litt ut etterhvert med denne koden.
float pastAngle = -90; // pastAngle brukes for å regne verdien i grader hvor mye zumoen har 
// flyttet seg i Y aksen. 


// int  strom = EEPROM.read(0);//mAh Strøm                    //lagt til ny
//kan sette knapper til å assigne størm en eller anna verdi. 


void use_battery(float hastighet){
  strom = strom - (0.1 * hastighet);
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
 // if (buttonC.getSingleDebouncedPress())
  if (buttonA.isPressed())
  {
    // Serial.println("I Button C");
  //   EEPROM.update(1,1000);                    alternativ hvis ikke stom lagres automatisk i eeprom
    strom = 100; //mah      //ny strom som derretter skal lagres i EEPROM
    Serial.println("Resett batteri");
   
  }
}
/*
void SHORT_RECHARGE() {
  motors.setSpeeds(0, 0);
  SHAKER(); 
}

void SHAKER () {

   imu.init();
   uint16_t tid = micros();
   uint16_t delta_T = tid - forrige_tid;
   uint16_t forige_tid = tid;
   imu.readGyro();
   new_angle += ((float)imu.g.y - gyroOffsetY) * 70*delta_T/ 1000.0; 
   Serial.println(new_angle);     */
   /*uint16_t delta_A = new_angle - angle;
   uint16_t angle = new_angle;
   while (delta_A != 0) {*/
 /*    strom += 20; 
     Serial.println("lader");
     while (strom >= 90) {
     break; 
     }}*/
     
 

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
void CalibrateGyro() {
  for (uint16_t i = 0; i < 1024; i++)
  {
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Y axis reading to the total.
    gyroOffsetY += imu.g.y;
  }
  gyroOffsetY /= 1024; 
}


void setup()
{
  
  Serial.begin(9600);
  Wire.begin();
  imu.init(); //Set up for inertial sensors
  imu.configureForBalancing(); // drar nytte av funksjoner som er ment for noe annet. men fungerer bra her. 
  display.clear();
  motors.setSpeeds(0,0);
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
  Calibrate();
  CalibrateGyro();
  motors.setSpeeds(0,0);  
  EEPROM.begin();
  strom = EEPROM.read(0);//mAh Strøm                    //lagt til ny
//kan sette knapper til å assigne størm en eller anna verdi. 
 
}

void diffAngle() {
  if (pastAngle >= angle + 30){
    Serial.println("angle elevated by 30ª");
    pastAngle = angle;
    }
  
  if (pastAngle <= angle -30) {
    Serial.println("Angle decreased by 30ª");
    pastAngle = angle;
    if (strom < 100) {
      strom = strom + 1;
      Serial.println(strom);
    }
    }
    }


template <typename Ta, typename Tb> float vector_dot(const Zumo32U4IMU::vector<Ta> *a, const Zumo32U4IMU::vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

// Reads the accelerometer and uses it to adjust the angle
// estimation.
void correctAngleAccel()
{
  imu.readAcc();

  // Calculate the angle according to the accelerometer.
  aAngle = -atan2(imu.a.z, -imu.a.x) * 180 / M_PI;

  // Calculate the magnitude of the measured acceleration vector,
  // in units of g.
  Zumo32U4IMU::vector<float> const aInG = {
    (float)imu.a.x / 4096,
    (float)imu.a.y / 4096,
    (float)imu.a.z / 4096}
  ;
  float mag = sqrt(vector_dot(&aInG, &aInG));

  // Calculate how much weight we should give to the
  // accelerometer reading.  When the magnitude is not close to
  // 1 g, we trust it less because it is being influenced by
  // non-gravity accelerations, so we give it a lower weight.
  float weight = 1 - 5 * abs(1 - mag);
  weight = constrain(weight, 0, 1);
  weight /= 10;

  // Adjust the angle estimation.  The higher the weight, the
  // more the angle gets adjusted.
  angle = weight * aAngle + (1 - weight) * angle;
}


// Reads the gyro and uses it to update the angle estimation.
void updateAngleGyro() {
  // Figure out how much time has passed since the last update.
  static uint16_t lastUpdate = 0;
  uint16_t m = micros();
  uint16_t dt = m - lastUpdate;
  lastUpdate = m;

  imu.readGyro();

  // Calculate how much the angle has changed, in degrees, and
  // add it to our estimation of the current angle.  The gyro's
  // sensitivity is 0.07 dps per digit.
  angle += ((float)imu.g.y - gyroOffsetY) * 70 * dt / 1000000000;
 /* if ((angle <= -120) || (angle >= -60)){
    Serial.println("Angle is more then 30 degrees");      //Får ut verdi fra angle hvor -90 er horisontal
    }  */
}


/*void printAngles()
{
  display.gotoXY(0, 0);
  display.print(angle);
  display.print(F("  "));
}*/

void TimeStamp() {
    correctAngleAccel();
    //printAngles(); 
     }
  

void loop() 
{
  
  updateAngleGyro(); 
  diffAngle();    
  // Every 20 ms (50 Hz), correct the angle using the  /trur koden bruker så lang tid at det ikke er nødvendig å vente
  // accelerometer, print it, and set the motor speeds.
  TimeStamp();
  
  unsigned long currentMillis = millis();
  int16_t position = lineSensors.readLine(lineSensorValues);
                    //leser av linje verdien 
                    // 0 = linje under 1. sensor
                    //1000 = linje under center sensor
                    //2000 = linje under 3. sensor 
  int16_t error = position - 1000; //Feilestimat 
  
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
  float  distance = (x/countspermeter); //måler distanse den har kjørt i cm.  //ly. måler i meter
  float StartMillis = millis();
  float hurtigheit = distance/(StartMillis - currentMillis);
  EEPROM.write(0,strom);
  EEPROM.update(0, strom);
  ResetStrom();
  /*
  display.clear();
  display.print(distance); */

  

  if (hurtigheit > 0) {
    use_battery(hurtigheit);
    float battery = strom;
    /*
    display.clear();
    display.print(battery);
    display.clear();
    */
   // display.gotoXY(0, 0);
   // display.print(strom);
  // Serial.println(strom);     
    }
    while (strom <= 1){
        delay(500);
        display.clear();
        display.print("DEAD BAT");
        delay(500);
        display.clear();
        motors.setLeftSpeed(0); 
        motors.setRightSpeed(0); 
        ResetStrom();                      //ny strom som derretter skal lagres i EEPROM
   
  }
 
  



if (currentMillis-lastMillis > period) 
{
    
    proxSensors.read();
   int center_left_sensor = proxSensors.countsFrontWithLeftLeds(); 
   //int center_right_sensor = proxSensors.countsFrontWithRightLeds();

   while (( center_left_sensor == 6 ) && ( leftSpeed > 0 &&  rightSpeed > 0 )) { 
           //forsøker å stoppe dersom den ser en objekt

       leftSpeed = 1 ;
       rightSpeed = 1;
       motors.setSpeeds(leftSpeed, rightSpeed);
       

    proxSensors.read();
    center_left_sensor = proxSensors.countsFrontWithLeftLeds();
    //center_right_sensor = proxSensors.countsFrontWithRightLeds();
    lastMillis = millis(); 
}}}
