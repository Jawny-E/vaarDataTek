//Importering av nødvendige bibliotek
#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>                                        
#include <zumo32U4IMU.h>
#include "RemoteConstants.h"
#include "RemoteDecoder.h"


Zumo32U4LCD display;   // LCD display brukes for å vise verdier slik at man vet hvor man er i koden
                               
RemoteDecoder decoder;

void correctAngleAccel()
{
  //imu er sensorverdier til aks og gyro. Imu.readAcc() gjør at den leser verdiene fra 
  //akselrometeret. 
  imu.readAcc();

  // Formel for å regne ut ved bruk av akselrometeret.
  aAngle = -atan2(imu.a.z, -imu.a.x) * 180 / M_PI;


  //regner ut omfanget målt av akselrometor i unit g. Formelen er hentet fra nett.
  Zumo32U4IMU::vector<float> const aInG = {
    (float)imu.a.x / 4096,
    (float)imu.a.y / 4096,
    (float)imu.a.z / 4096}
  ;
  float mag = sqrt(vector_dot(&aInG, &aInG));

  //Regner på hvor mye vi skal telle med akselrometerets svar på endring i akselrasjon. 
  //Jo mere g zumoen opplever, desto mindre stoler vi på verdien den gir ut. 
  float weight = 1 - 5 * abs(1 - mag);
  weight = constrain(weight, 0, 1);
  weight /= 10;

  //Juster vinkel estimat, desto høyere weight, jo mer blir binkelen justert. 
  angle = weight * aAngle + (1 - weight) * angle;
}


void setup()
{
  decoder.init();
  Serial.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
  imu.init(); //Set up for inertial sensors
  imu.configureForBalancing(); // drar nytte av funksjoner som er ment for noe annet. men fungerer bra her. 
  display.clear();  // I tilfelle det var noe på LCD skjermen sist gang den var ibruk. 
  motors.setSpeeds(0,0);
  proxSensors.initThreeSensors(); //Aktiverer IR-Sensorer
  lineSensors.initThreeSensors();  //Aktiverer 3 av linje sensorene
  Calibrate();  //Kalibrerer linjesensor i setup
  CalibrateGyro();  // Kalibrerer gyrp og aks
  motors.setSpeeds(0,0);    //Setter hastighet = 0 
  EEPROM.begin();   // Gjør det mulig å hente og skrive inn Write
  strom = EEPROM.read(0);//mAh Strøm                    //lagt til ny
//kan sette knapper til å assigne størm en eller anna verdi. 
 
}

//Stiller inn aksene med en ferdigstilt formel hentet fra maler på nett. 
template <typename Ta, typename Tb> float vector_dot(const Zumo32U4IMU::vector<Ta> *a, const Zumo32U4IMU::vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}



//ProceccRemoteEvent Brukes til IR meldinger fra fjernkontroll. 
//Denne ser at hvis vi får inn signal, så skal vi prosessere meldingen vidre. 
void processRemoteEvents()
{
  if (decoder.getAndResetMessageFlag())
  {
    // The remote decoder received a new message, so record what
    // time it was received and process it.
    lastMessageTimeMs = millis();
    messageActive = true;
    processRemoteMessage(decoder.getMessage());
  }

  if (decoder.getAndResetRepeatFlag())
  {
    // The remote decoder receiver a "repeat" command, which is
    // sent about every 109 ms while the button is being held
    // down.  It contains no data.  We record what time the
    // repeat command was received so we can know that the
    // current message is still active.
    lastMessageTimeMs = millis();
  }
}


void processRemoteMessage(const uint8_t * message)
{
  // Print the raw message on the first line of the display, in hex.
  // The first two bytes are usually an address, and the third
  // byte is usually a command.  The last byte is supposed to be
  // the bitwise inverse of the third byte, and if that is the
  // case, then we don't print it.
  display.clear();
  char buffer[9];
  if (message[2] + message[3] == 0xFF)
  {
    sprintf(buffer, "%02X%02X %02X ",
      message[0], message[1], message[2]);
  }
  else
  {
    sprintf(buffer, "%02X%02X%02X%02X",
      message[0], message[1], message[2], message[3]);
  }
  display.print(buffer);

  // Go to the next line of the display.
  display.gotoXY(0, 1);

  // Make sure the address matches what we expect.
  if (message[0] != remoteAddressByte0 ||
    message[1] != remoteAddressByte1)
  {
    motors.setSpeeds(0, 0);
    display.print(F("bad addr"));
    return;
  }

  // Make sure that the last byte is the logical inverse of the
  // command byte.


}


void TimeStamp() {
    correctAngleAccel();
    //printAngles(); 
     }
  

void loop() 
{
/*   
 *    Desverre har ikke vi løst hvorfor ikke remote funksjonen vil fungere i loopen. 
 *    Vi går ut ifra at den venter på en lesning å setter seg i en "state" som holder 
 *    zumoen fra å gjennomføre loopen kontinuerlig. Desverre fikk ikke vi til å feilsøke 
 *    ettersom zumoen vår sluttet å virke. 
 *    Remotecontrol koden er hentet fra eksempelkode og modifisert til å passe hva
 *    vi har lyst til å oppnå med koden. 
 */
/*
decoder.service();
ledYellow(messageActive); If messageActive, lys på basicly. Gjør det lett å se når
// vi mottar signaler fra fjernkontroll. 
ledRed(decoder.criticalTime());

  if (decoder.criticalTime())
  {
    // We are in the middle of receiving a message from the
    // remote, so we should avoid doing anything that might take
    // more than a few tens of microseconds, and call
    // decoder.service() as often as possible.
  }
  else
  {
    // We are not in a critical time, so we can do other things
    // as long as they do not take longer than about 7.3 ms.
    // Delays longer than that can cause some remote control
    // messages to be missed.

    processRemoteEvents();
  
  


if (messageActive && (uint16_t)(millis() - lastMessageTimeMs) > messageTimeoutMs)
  {
  messageActive = false;
  */
  // Oppdaterer gyroskop slik at det er mest mulig presis. 
  updateAngleGyro(); 
  //Ser om zumoen har bevegd seg 30 grader ned da dette er en trigger for 
  //nødmodus SHAKE
  diffAngle();    
  //Hvert 20ms retter vi på vinkelen 
  TimeStamp();
  //Starter tid taking (currentMilles)
  unsigned long currentMillis = millis();
  //Linjesensor kode. Denne justerer seg etter linjen. 
  int16_t position = lineSensors.readLine(lineSensorValues);
                    //leser av linje verdien 
                    // 0 = linje under 1. sensor
                    //1000 = linje under center sensor
                    //2000 = linje under 3. sensor 
  int16_t error = position - 1000; //Feilestimat Pga at vi bruker 3 sensorer vil vi rette 
  // oss mer inn mot midten. 


  int16_t speedDifference = error / 4 + 6 * (error - lastError);//tatt fra eksempelkoden 
  lastError = error;
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed); //tvinger speed i en range 
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed); //--||--
  motors.setSpeeds(leftSpeed, rightSpeed); //Setter angitt hastigheten til motorene. 

  int Left_encoder = encoders.getCountsRight(); //Kan få overflow  (over 16 bit)
  int Right_encoder = encoders.getCountsLeft(); // Men vi har valgt å resette den etter hver loop
  // slik at den aldri går så høyt. 
  float sum = 0;
  
 // for (int h = 0; h < 4; h++) {    Denne skulle denne som gjør at det er mulig å lese
 //gjennomsnitts fart og noen andre variabler som måtte til for å båle batterihelse. 
 
  float  x = ((Left_encoder + Right_encoder) / 2); //  gjennomsnittet av begge encoderene 
  float  distance = (x/countspermeter)*100; //måler distanse den har kjørt i cm.  //ly. måler i meter
  //Serial.println(distance, "distance");
  float StartMillis = millis();//Starter tid Millis
  float hurtigheit = distance/(StartMillis - currentMillis);
  EEPROM.write(0,strom);        //skriver ny stromverdi til minnet
  EEPROM.update(0, strom);      //oppdaterer til den nye stormverdi
  ResetStrom();                 // resete strøm //bare hvis betingelsen er oppfylt

  Left_encoder = encoders.getCountsAndResetLeft(); //Resette encoder slik at den ikke får overflow
  Right_encoder = encoders.getCountsAndResetRight(); //--||--
  stromtoint();  //regner formel tidligere beskrevet. 
  //Funksjon for å sende motta meldinger fra ESP.
  RecivefromESP();  //Regner en formel som tar imor kode fra esp og omgjør dette til int. 

    float max_speed_now = 0;
    data_speed[h] = hastighet; 
    sum = sum + data_speed[h];
    if (data_speed [h] > max_speed_now)
    { max_speed_now = data_speed[h];}
    sum = sum + data_speed[h];
    Serial.print("h :");
    Serial.println(h);

  if (hurtigheit > 0) {
    use_battery(hurtigheit);  // Regner ut stromtaper dersom bilen er i bevegelse. 
    float battery = strom;
    }
     while (strom <= 1){
      motors.setSpeeds(0,0); 
      // Dersom strømmen er helt tom og du ikke kna kjøre eller nødlade. Kan 
      //Brukere ta tilbake sitt dålige valg å ikke følge med på strømprosenten 
      // og trykke på jukseknappen A for å resette seg til 100%
      ResetStrom(); }  
      //ny strom som derretter skal lagres i EEPROM}
      // siden strom er en float, vil den endre seg stort sett hele tiden. Nå nå 
      // strommen endre seg med minst 1% for å inisiere en count.
    if (strom > new_strom +1) { 
     //Vi oppdaterer new_battery slik ser en endring
     //Den skal legge til dersom strømmen går opp. 
     //hver gang vi gå i løkken vil cyles øke med 1 
     cycles = cycles + 1;
     new_strom = strom;
   }

    

  /*
   if ((strom <= 100) && (strom >= 60)) {
    ledGreen(1); //grønt lys indikerer at batteriprosent er mellom 60-100%
    ledYellow(0);
    ledRed(0);
    //Serial.println(strom);
   }
   if ((strom <= 59) && (strom >= 30)) {
    ledGreen(0);  //gult lys indikerer på at batteriprosent er mellom 30-60%
    ledYellow(1);
    ledRed(0);
    //Serial.println(strom);
   }
   if ((strom <=29) && (strom >= 2)) {
    ledGreen(0);  //rødt lys indikerer på at batteriprosent under 30%
    ledYellow(0);
    ledRed(1);
    }
    */  //Lysene skal indikere strømnivået, Men ESP kortet ligger over lydene, Da blir
    //det unødvendig å ha disse lysene på. 


  //bruker hastighet fraen tidligere 
 while (hastighet >= grense_hastighet  ) { //hvis hastigheten er over 70 av maxSpeed;
     unsigned long  Start = millis(); // tar tiden 
      //vi oppdaterer hastigheten  
     }

   unsigned long Stop = millis();//tar stop tiden 
   int duration = (Stop - Start)/1000; 

     if (strom > 5) {
     int low_battery = low_battery + 1; }
   
if (currentMillis-lastMillis > period) 
{
    
   proxSensors.read();
   int center_left_sensor = proxSensors.countsFrontWithLeftLeds(); 
  // int center_right_sensor = proxSensors.countsFrontWithRightLeds();

   //forsøker å stoppe dersom den ser en objekt foran seg hvis ikke farten er null 

   while (( center_left_sensor == 6 ) && ( leftSpeed > 0 &&  rightSpeed > 0 )) { 
       //forsøker å stoppe dersom den ser en objekt
       leftSpeed = 1 ;
       rightSpeed = 1;
       motors.setSpeeds(leftSpeed, rightSpeed);
       Serial.println("Bat Dead");
       

    proxSensors.read();
    center_left_sensor = proxSensors.countsFrontWithLeftLeds();
    //center_right_sensor = proxSensors.countsFrontWithRightLeds();
    lastMillis = millis(); 
          }
        }
     
Serial.println("HER SKAL ALLE VERDIENE PRINTES");
  average_speed = sum/4;

  battery_health = 100 - 0.05*(average_speed + max_speed + duration + low_battery + cycles);
   //har lagt egen funksjon, det sto større verdi vi får desto dårligere  // vi har lust å lage en range hvis den overskrider 100; 
   battery_health = constrain(battery_health, 0, 100); 
   // vil ha 0 = dårlig battery health og 100 = bra battery
   Serial.print("strom : ");
   Serial.println(strom);
   Serial.print("hastighet :");
   Serial.println(hastighet);
   Serial.print("average :");
   Serial.println(average_speed);
   Serial.print("max_speed :");
   Serial.println(max_speed);
   Serial.print("low_battery :");
   Serial.println(low_battery);
   Serial.print("cycles :");
   Serial.println(cycles);
   Serial.print("Duration :");
   Serial.println(duration);
   Serial.println(battery_health);
 }
