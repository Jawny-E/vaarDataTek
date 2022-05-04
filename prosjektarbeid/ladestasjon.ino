/*
Oppsett RFID scanner:
SDA pinne 5
SCK pinne 18
MOSI pinne 23
MISO pinne 19
RQ pinne Ingen tilkobling
RST pinne 22

Oppsett andre komponentar:
Gul led-pinne 35
Raud led-pinne 4
Grønn led-pinne 2
Potmeter-pinne 34
Pull-down knapp pinne 33
Servo-pinne 12
Anbefalar å koble ein LED til målepinnen til potmeteret,
for å gje kjøparen visuell input

Switch-case struktur
case 1: ventecase{Her blir konto og batteri oppdatert, venter på eit av to gyldige rfid kort, eit kort tar deg til case 2 det andre til case 3}
case 2: ladecase{Venter til knappen blir trykt inn, reknar ut ny kontosaldo og batteriprosent frå vridninga på potmeteret, 
        går til case 1 om betalinga er godkjent, tilbake i case 2 om ikkje}
case 3: getReady{Venter på trykk for å signalisere at spilleren er klar, går til case 4}
case 4: spillVent{Dersom spilleren trykker inn knappen, case 1, viss ikkje case 5}
case 5: reaksjonscase{Spilleren må trykke på knappen innen 3 sek for å vinne peng, går alltid til case 1}
*/

//Nødvendige bibliotek for ladestasjonen
#include <SPI.h>
#include <MFRC522.h>
#define RST_PIN         22         
#define SS_PIN          5   
MFRC522 mfrc522(SS_PIN, RST_PIN); 
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Servo.h>

// Lagrer SSID og passord på nettverket vi benytter oss av i to konstanter 
const char* ssid = "ThisIsANetwork";
const char* password = "ThisIsPassword";
// Lagrer IP-adressen til MQTT-brokeren vi benytter oss av 
const char* mqtt_server = "192.168.231.216";

// Setter opp Wifi, PubSubClient og servo
WiFiClient espLADEClient;
PubSubClient client(espLADEClient);
long lastMsg = 0;
char msg[50];
int value = 0;
Servo servo;

/*Funksjon for å sette opp Wifi*/
void setup_wifi() { 
  delay(500);
  // Indikerer at vi starter oppkoblingen til Wifi 
  Serial.println();
  Serial.print("Kobler til");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  //Dersom vi venter på å bli koblet opp printes det "..."
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //Indikerer at vi er koblet opp, samt returnerer IP adressen
  Serial.println("");
  Serial.println("WiFi koblet til");
  Serial.println("IP addressen: ");
  Serial.println(WiFi.localIP());
}

//Bestemmer navn for ledpinner, potentiometerpinne, servo og knapp pinne
const int redLed = 4;
const int greenLed = 2;
const int yellowLed = 13;
const int potPin = 34;
const int SW1 = 33;
const int servoPin = 12;
/*Ventevariablar, kan kanskje omgjerast til lokale?*/
unsigned long wait;
unsigned long now = 0;
unsigned long wait1 = 0;

int cases = 1; //Variabelen som handtera case-struktur
int strompris = 20; //Gir strømprisen ein basisverdi, fordi den kunn blir oppdatert når miljøstasjonen sender ut MQTT melding
int konto; //Denne variabelen skal halde saldoen på kontoen, oppdaterast i case 1 og 2
const int batteriKap = 64; //Batterikapasitet i kWh, illustrerande verdi
int batteri; //Held batteriet i prosent i Zumoen, oppdaterast i case 1 og 2

void setup() {
  //Led-pinnene er output
  pinMode(redLed,OUTPUT);
  pinMode(yellowLed,OUTPUT);
  pinMode(greenLed,OUTPUT);  
  //Knapp og pot-meter er input noder
  pinMode(SW1,INPUT);
  pinMode(potPin, INPUT);
  //Starter serie kommunikasjon
  Serial.begin(115200);
  //Starter Serial Peripheral Interface og setter opp rfid
  SPI.begin();
  mfrc522.PCD_Init();
  //Setter opp servo
  servo.attach(servoPin);
  //Caller setup wifi
  setup_wifi();
  //Setter opp mqtt server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

/*Denne funksjonen handterar inkommande MQTT meldingar*/
void callback(char* topic, byte* message, unsigned int length) {
  if(String(topic)== "esp32/konto"){ //Om meldingen kjem på topicet konto
    Serial.println("Henta kontoutskrift");
    String messageTemp;
    for (int i = 0; i < length; i++) {
      messageTemp += (char)message[i];} //Gjer meldinga om til ein string som kan lesast
    konto = messageTemp.toInt(); //oppdaterar konto med int versjonen av meldinga
    Serial.println(konto);
    } 
  if(String(topic) == "esp32/solcelle"){ //Om meldinga kjem på topicet solcelle
    Serial.println("Henta straumpris");
    String messageTemp;
    for (int i = 0; i < length; i++) {
      messageTemp += (char)message[i];}
    strompris = messageTemp.toInt();
    Serial.println(strompris);
    }
  if(String(topic) == "esp32/batteri"){ //Om meldinga kjem på batteri topic
    Serial.println("Henta resterande batteri: ");
    String messageTemp;
    for (int i = 0; i < length; i++) {
      messageTemp += (char)message[i];}
    batteri = messageTemp.toInt();
    Serial.println(batteri);
    if(batteri < 40){
      servo.write(0);
      }
    else{
      servo.write(90);
      }
    }
}

/*Handterar kobling til Wifi om det fell ut*/
void reconnect() {
  // Loop fram til vi er kopla til igjen
  while (!client.connected()) {
    Serial.print("Forsøker MQTT tilkopling...");
    // Forsøker å koble til, printer resultat til serial monitor
    if (client.connect("ESP8266Client")) {
      Serial.println("tilkopla");
      // Subscribe til topicsa vi bruker i modulen
      client.subscribe("esp32/konto"); 
      client.subscribe("esp32/solcelle");
      client.subscribe("esp32/batteri");
    } else {
      Serial.print("feilet, rc=");
      Serial.print(client.state());
      Serial.println(" prøver igjen om 5 sekund");
      // Venter 5 sek før omprøving
      delay(5000);
    }
  }
}

/*Funksjon for grønn ledbllink*/
void celebrate(){ 
    for(int i = 0; i < 3;i++){
      digitalWrite(greenLed, HIGH);
      delay(100);
      digitalWrite(greenLed, LOW);
      delay(100);}
    }
/*Funksjon for raud ledblink*/
void feil(){ //Her er funksjonen for feilblink
  for(int i=0; i<3;i++){
      digitalWrite(redLed, HIGH);
      delay(100);
      digitalWrite(redLed, LOW);}
      delay(100);
      }

/*Her køyrer hovudprogrammet*/
void loop(){
  //Basert på switch case struktur
  switch(cases){
    case 1: {
      //Skrur av alle ledsa her, sidan det kan være ulike resultat i spelet
      digitalWrite(yellowLed,LOW);
      digitalWrite(greenLed,LOW);
      digitalWrite(redLed,LOW);
      //køyrer reconnect om vi har falt ut av wifi tilkoplinga
      if (!client.connected()) {
         reconnect();
      }
      client.loop();
      int period = 2000;
      unsigned long now1 = millis();
      //Oppdaterar batteri og konto kvart andre sekund
      if(now1 - wait1 > period){
          client.publish("esp32/requestKonto", "40");//Verdien vi publisera er irrelevant
          client.publish("esp32/requestBattery", "40");
          int x = servo.read();//Leser av vinkelen til servoen
          char vinkelString[8]; 
          dtostrf(x, 1, 2, vinkelString);
          client.publish("esp32/bomVinkel",vinkelString);//Publiserer vinkelinfoen
          
          wait1 = millis();}
      
      if ( ! mfrc522.PICC_IsNewCardPresent()){
        return;} 
      if ( ! mfrc522.PICC_ReadCardSerial()) {
        return;}
        
      //Oppretter en plass vi kan lagre avlesingene fra readeren
      String innhold= "";
      byte letter;
      //So lenge ingenting er registrert på RFID scanneren
      while(innhold ==""){
      for (byte i = 0; i < mfrc522.uid.size; i++) 
      {
         innhold.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
         innhold.concat(String(mfrc522.uid.uidByte[i], HEX));
      }
      Serial.println();
      innhold.toUpperCase();
      if (innhold.substring(1) == "B2 B9 2C 3D"){ //UID-taggen til kortet på zumobilen {
        Serial.println("Velkommen gruppe 1");
        cases = 2; //Gå til case 2, lading
        delay(200);
        break;}
      if (innhold.substring(1) == "D2 F6 97 18"){ //UID-taggen til kortet på zumobilen {
        Serial.println("Velkommen til Casino");
        Serial.println();
        delay(200);
        cases = 3; //Gå til case 3, spel del 1
        break;}
     else {
        Serial.println("Ikke registrert kunde"); //Kortet er ikkje registret
        delay(1000);//Går tulbake til case 1
        break;}
     }//Etter while-løkka
     break;
    }
     
   case 2:{ //Ladestasjon 
      digitalWrite(yellowLed,HIGH); //Skrur på det gule lyset
      int x; //Lokal varibel for mellombels lagring av data frå potmeteret
      //So lenge knappen ikkje er trykt inn
      while(digitalRead(SW1)==HIGH){
        x = analogRead(potPin);// Leser av potentiometerverdien
        
        //Kjører samme sekvensen for å få batteri og konto oppdatert som i case 1
        if (!client.connected()) {
         reconnect();}
        client.loop();
        int period = 2000;
        unsigned long now1 = millis();
        if(now1 - wait1 > period){
          client.publish("esp32/requestKonto", "40");//Verdien vi publisera er irrelevant
          client.publish("esp32/requestBattery", "40");
          wait1 = millis();}
        }
      digitalWrite(yellowLed,LOW);//Skrur av det gule lyset 
      
      int percentage = map(x, 0, 4095, 0, 100); //Finner prosent opplading frå potmeterverdien
      if((percentage + batteri) >100){ //Sørger for at stømmen ikkje kan være over 100%
        percentage = 100 - batteri;
        }
      int nyKonto = konto - (percentage * batteriKap * strompris * 0.01); //Reknar ut straumpris
      int nyBatteri = batteri + percentage; //Reknar ut ny batteriprosent
      
      if(nyKonto<0 && batteri>10){ //Dersom kontoen hadde blitt negativ, og batteriet er over 10% frå før
        cases = 2; //Neste case er case 2 igjen
        feil();//Blinker raudt for å indikere at transaksjonen feila
        feil();}
      else{ //Ved ny saldo godkjent
        cases = 1; //Går tilbake til case 1 når sekvensen er over
        celebrate(); //Blinker grønt for å indikere at transaksjonen fungerte
        celebrate();
        char kontoString[8]; //Konvertera til rett datatype
        dtostrf(nyKonto, 1, 2, kontoString);
        client.publish("esp32/updateKonto", kontoString); //Publisera ny kontoverdi

        char batteriString[8]; //Konvertera til rett datatype
        dtostrf(nyBatteri, 1, 2, batteriString);
        client.publish("esp32/updateBattery", batteriString); //Publisera ny batteriverdi
        servo.write(90); //Setter opp bommen
        delay(1000); //Bruker delay for å vente, da dette ikkje skal påverke andre delar av programmet
        }
      break;
      }
   case 3: {  //Spel - del 1, gjer klar
     while(digitalRead(SW1)==HIGH){ //Vent på knppetrykk
        digitalWrite(yellowLed,HIGH); //Gul led på
        cases = 4;} //Neste case er 4
   }
   case 4: { //Spel - del 2, ventefase
    digitalWrite(redLed, HIGH); //Gul og raud led lyser no
    int time = random(3000, 6000); //Ventetida er tilfeldig
    delay(500);
    wait = millis();
    now = millis();
    cases = 5; //Neste case er 5, om vi ikkje går inn i if-setninga
    while(now - wait <= time){ //I ventetida
      if(digitalRead(SW1)==LOW){ //Om du trykker på knappen
        cases = 1; //Gå tilbake til case 1
        feil(); //Blink raudt for tap
        break; 
        }
     now = millis();
     } 
    }
   case 5:{ //Spel - del 3, reaksjonsfase
     digitalWrite(redLed,LOW); //Skru av raud led, på med grønn
     digitalWrite(greenLed, HIGH);
     int period = 3000; //Kan reagere opp til 3 sek
     now = millis();
     wait = millis();
     while(now - wait < period){ //I perioden 3 sek
     if(digitalRead(SW1)==LOW){ //Dersom du klikker på knappen
          konto = konto + 750; //Oppdater konto med 750kr ekstra
          char kontoString[8]; //Konvertera til rett datatype
          dtostrf(konto, 1, 2, kontoString);
          client.publish("esp32/updateKonto", kontoString); //Publisera ny kontoverdi
          celebrate(); //Blinkar grønt lys for iindikere vinning
          break; }
     }
     cases = 1; //GÅ TIL CASE 1
     break;
    }
  }
}

/*
Forbetring av koden vidare
Gjere nokre av desse om til funksjonar:
-Oppdatering av tal via MQTT(gjerast 4 gonga)
-case 3 og 4 kan gjerne være pakka inn i funksjonar då det ikkje skjer nokon kommunikasjon her
-Utrekning av tal o.l.

Rydde i ka som treng å være globale variablar og kva som kan være lokalt

Navngivinga er ganske innafor, men bør sjåast gjennom

Legge til kontroll av bommen frå nettside, burde være greit å gjennomføre gjennom http
*/

//Koden til autorisasjonen ved hjelp av RFID er inspierert av denne videoen:
//https://www.youtube.com/watch?v=pJLjFm4Ipro
