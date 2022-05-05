// This file contains constants for the address and commands in
// the messages transmitted by the Mini IR Remote Control:
// https://www.pololu.com/product/2777

#pragma once

//Importering av nødvendige bibliotek
#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>                                        //ny
#include <zumo32U4IMU.h>

#include <Wire.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors; // til prox sensor
Zumo32U4IMU imu;
Zumo32U4Encoders encoders; 
Zumo32U4ButtonA buttonA;   


//Fikk begynt på å sette inn i .h fil biblotek for å vise at vi har skjønt hvordan det fungerer
//men har desverre ikke tid til å gjøre det med alt.

// Definerer maksimalhastigheten til zumo-bilen
// Siden koden er blirr såpass lang som den er, tar det lengre tid mellom hver målinger 
// fra linesensoren. kombinert med at koden bare bruker 3/5 linjesensorer resulterte dette 
// i at zumoen kjører mer upresist en det vi orginalt hadde planer om. Derfor har vi blirr 
// nødt til å sette ned hastigheten bilen kjører


// Alt som har med angle å gjøre
float gyroOffsetY; //henter Y aksen på gyro
uint16_t angle = -90; // angle kofigureres til å bare vare basert på gyro
float aAngle =0;     // aAngle konfigureres til å bare vare basert på accelrometer
// vi bruker akselrometer senere for å klibrere gyroskopet korrekt slik at vi minsker 
// feil estimat. Av erfaring vil verdien skli litt ut etterhvert med denne koden.
float pastAngle = -90; // pastAngle brukes for å regne verdien i grader hvor mye zumoen har 
// flyttet seg i Y aksen. 

//Til overføring av kode fra ESP til zumo. velger max lengde på meldingen vi forventer å motta. 
const unsigned int MAX_MESSAGE_LENGTH = 4; //velger antall ut ifra meldings lengde

// ReciveFromESP er det det høres ut som. Den tar inn informasjonen som kommer inn fra Serial1
// og detter sammen informasjonen bit for bit til meldingen er ferdig.
const int16_t maxSpeed = 220;
const int grense_hastighet = maxSpeed*0.7;  
uint16_t cont = 0;
float strom;  //Strom er i dette tilfellet bilens SW batteriprosent og regnes kontinuerlig
int forrige_tid = 0;
int16_t lastError = 0; //Variabel for korrigering av feilbevegelse 
int data_speed[] =  {};
int x;
int h;
int hastighet;
unsigned int lineSensorValues[2];
int countspermeter = 8195;
unsigned long now;
unsigned long wait;
int StopMillis = 0;
const int period = 100;
unsigned long lastMillis = 0;

int new_battery = 0 ;
const int period_charge = 1000;
int cycles = 0;
int low_battery = 0 ;
int forrige_hurtighet = 0;
int i = 1; 
int last_speed = 0; 
int level_1_cyles = 0; 
unsigned long currentMillis;
int Left_encoder;
int Right_encoder;
unsigned long Start;
unsigned long Stop; 
int under_five_battery = 0;
int duration; 
float max_speed ;
unsigned long prevMillis = 0;

int battery_health;
int new_strom = 0; 
float average_speed;

uint16_t lastMessageTimeMs = 0;
bool messageActive = false;
const uint16_t messageTimeoutMs = 115;

const uint16_t remoteAddressByte0 = 0x00;
const uint16_t remoteAddressByte1 = 0xBF;

void RecivefromESP()
{
    while (Serial1.available() > 0)
  {
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0; 

    //leser ut bytes fra serial recive buffer. Dette gjøres en og en byte omgangen. 
    char inByte = Serial1.read();
    
    // meldinger som kommer inn. 
    if ((inByte != "\n") && (message_pos < MAX_MESSAGE_LENGTH -1)) {
     //legg til inkommende bytes til meldingen siden den ikke er ferdig. 
     message[message_pos] = inByte; //vi setter inn riktig char på riktig plass på arrayet
     message_pos++; // denne gjør at neste byte kan lagres i neste plass på arrayet. 
    }
    else {
             // do something. Hele meldingen er kommet igjennom.
    
       // legger til null karakter til string
       message[message_pos] = "\n";

       //printer meldingen, eller så kan vi legge til at den skal gjøre noe annet.
       Serial.println(message);
       //eller konverterer meldingen til integer and print
       //meldingene som sender sendes som numeriske meldinger vil automatisk som
       //ASCII meldinger. Derfor konverterer vi disse til integer med atoi.
       
       int number = atoi(message); //
       //Serial.println(number);
       Serial.println(message);
       // hvis meldingen som kommer inn. altså batteriprosent e større en strømmen vi har
       // så skal vi sette inn ny støm. Detta funke bare sålenge bilen ikke e fulstendig 
       // fri for strøm. 
       // Denne nedlastnings koden har ikke vært testet siden zumoen sluttet å fungere. 
       if (strom < number){
        strom = number;
       }

       //resette for neste melding
       message_pos = 0;
    }
  }
}

//  use_battery regner ut hvor mye strøm som skal tas bort ut ifra bilens hastighet. 
void use_battery(float hastighet){
  strom = strom - (0.5 * hastighet);
  }

//Funksjon som står for kalibereringen av linjesensor. 
void Calibrate(){
  for (cont = 0; cont < 200; cont++){
  
    
    if (cont <= 50 || cont >= 105) {
      motors.setSpeeds(150, -150);
      }
    if (cont >= 55 && cont <= 100) {
      motors.setSpeeds(-150, 150);
     
      }
       lineSensors.calibrate();
    }
    motors.setSpeeds(0, 0);
    }
//Funksjon som står for kalibereringen av gyroskop og akselrometer. 
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

// diffAnfle er en funksjonsom måler endringer i grader i zumoens Y-akse
void diffAngle() {
  if (pastAngle >= angle + 30){
    Serial.println("angle elevated by 30ª");
    pastAngle = angle;
    }
     //Y aksen har en drift i seg slik at den kontinuerlig vil øke litt. Dette skjer
    //ettersom koden ikke for rebalibrert seg ofte nokk pga kodens lengde. 
    //Vi er derimot bare interesert i koden når den tilter fremover, ikke bakover. 
  if (pastAngle <= angle -30) {
    Serial.println("Angle decreased by 30ª");
    pastAngle = angle;
    if (strom < 100) {
      strom = strom + 5;
      Serial.println(strom);
  }}}

// Reads the accelerometer and uses it to adjust the angle
// estimation.
//CorrectAngleAccel bruker akselrometeret til å gjøre justeringer dersom zumoen ikke opplever
//akselrerende eller de-aks. 

   // Oppdatere gyroskopet og bruker den til å regne ut vinkel estimat. 
void updateAngleGyro() {
  // Finn ut hvor mye tid som har passert siden sist lesing. 
  static uint16_t lastUpdate = 0;
  uint16_t m = micros();
  uint16_t dt = m - lastUpdate;
  lastUpdate = m;

  imu.readGyro(); //Hente sensorverdier fra gyroskopet.
   // Regner ut hvor mye vinkelen her endret seg ut i grader og legger det til current cangle. 
   //Gyroens sensitivitet er satt på 0.07 dps. Gyro sensitivitet har vi hentet fra nettside.
  angle += ((float)imu.g.y - gyroOffsetY) * 70 * dt / 1000000000;
}


/*Strom to inter en kjapp omregning som gjær det mulig å sende strømverdien til batteriet 
 * opp til ESP32 via Serial1.  */
void stromtoint() {
 int toint = strom;
 Serial1.write(toint);
 
}
