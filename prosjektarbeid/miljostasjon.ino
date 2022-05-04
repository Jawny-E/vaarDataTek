// Hvordan koble opp AHT10 til ESP32:
// SCL = D22
// SDA = D21
// VIN = VIN
// GND = GND

//Hvordan koble opp OLED-skjermen til ESP32:
// SDA = D21
// SCK = D22
// VDD = 3.3V
// GND = GND

//Solcellepanel- to spenningsdelere
//photoresistor i serie med en resistans 2x
//Målepinne 1 = pinne 35
//Målepinne 2 = pinne 34

//Anna
//LED lys: pinne 26 
//Servo: pinne 13
//DC-motor: pinne 4

// Deklarerer konstanter(pinner)og variabler brukt
const int vifte = 4;
//const int ledpin = 6;
const int sensorPin = 32;
const int gatelys = 26;
const int photo1 = 35;
const int photo2 = 34;
const int period = 10000;
unsigned long nowtime;
unsigned long starttime;
int A;
int B;
int solcounter = 0;
int tidligere;
int vinkel = 90;
// Lagrer SSID og passord på nettverket vi benytter oss av i to konstanter 
const char* ssid = "ThisIsANetwork";
const char* password = "ThisIsPassword";
// Lagrer IP-adressen til MQTT-brokeren vi benytter oss av 
const char* mqtt_server = "192.168.231.216";

//Bibliotek inkludert
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHT10.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Oppsett av sensor, OLED skjerm og servomotor
Adafruit_AHT10 aht;
Servo servo;
#define SCREEN_WIDTH 128 // OLED display dimensjoner på skjermen vår (128x32)
#define SCREEN_HEIGHT 32 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Setter opp tilkobling mellom ESP32 og WiFi
WiFiClient espMiljoClient;
PubSubClient client(espMiljoClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void justerPanel(){ //Justera solcellepanelet slik at det er minst mogleg skilnad mellom nodane
  nowtime = millis(); //Bruker millis for å avgjøre hvor lenge solcellepanelet skal instille seg
  starttime = millis();
  int diff = nowtime - starttime;
  
  while(diff < 1000){ //While-løkken kjører i 100 millisekund
    servo.write(vinkel); //Setter servoen til ny vinkel
    A = analogRead(photo1); //Leser av pinnene med photoresistorer
    B = analogRead(photo2);
    Serial.println(vinkel);
    if(A<B){ //hvis A > B
      if(vinkel < 135){ // servovinkelen kan ikke være mer enn 135deg 
        vinkel += 1; //oppdaterer vinkelen med + 1 grad så lenge A != B
        }
      }
    if(A>B) { 
      if(vinkel > 45){ //servoen kan ikke være mindre enn 45 grader 
        vinkel = vinkel-1; //oppdaterer vinkelen med - 1 grad så lenge A != B
        }     
    }
    nowtime = millis(); //Oppdaterer millis og differansen
    diff = nowtime - starttime;
  }
  
  if(A <500){ //Bestemmer om gatelyset skal være på dersom photoresistoren er lav
    digitalWrite(gatelys, HIGH);}
  else{
    digitalWrite(gatelys, LOW);}
}


/*Funskjon for å sette opp WiFi*/
void setup_wifi() { 
  delay(500);
  // Indikerer at vi starter oppkoblingen til Wifi 
  Serial.println();
  Serial.print("Tilkobler til ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  //Dersom vi venter på å bli koblet opp printes det "..."
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  //Indikerer at vi er koblet opp, samt returnerer IP adressen
  Serial.println("");
  Serial.println("WiFi koblet til");
  Serial.println("IP addressen: ");
  Serial.println(WiFi.localIP());
}

void AHT10_finder(){ //Funskjon som sjekker om sensoren er riktig tilkoblet 
    if (! aht.begin()) {
    Serial.println("Kan ikke finne AHT10, sjekk om den er koblet til riktig!");
    Serial.println("SCL = D22");
    Serial.println("SDA = D21");
    Serial.println("VIN = VIN");
    Serial.println("GND = GND");//Gir brukeren en melding om korrekt oppsett viss den ikke finner sensoren
    while (1) delay(10);
  }
  Serial.println("AHT10 funnet"); //Printer at sensoren er på plass
}

void OLED_finder(){ //Funksjon som sjekker om OLED-skjermen er koblet til riktig
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);}
  display.clearDisplay(); // Clearer skjerm for å evt. fjerne det tidligere innholdet
  display.setTextSize(1); // Velger størrelse på tekst
  display.setTextColor(WHITE); // Velger farge på tekst 
  display.setCursor(0, 10); //
}

void callback(char* topic, byte* payload, unsigned int length) { //Denne funksjonen blir callet dersom ein
  //melding kjem inn på eit topic vi har subscriba til (kunn juster sollcellepanel her)
  Serial.print("Message arrived in topic: "); //Viser at vi har fått melding
  Serial.println(topic); //Printer topicet meldingen kom på
  justerPanel(); //Caller juster panel, siden det er den einaste tingen vi subscriba for
}
void setup() {
  Serial.begin(115200); //Oppretter seriell-kommunikasjon til datamaskin, med baud rate 115200
  tidligere = millis();
  pinMode(vifte, OUTPUT); //deklarerer viften som output 

  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(gatelys, OUTPUT);
  pinMode(photo1, INPUT);
  pinMode(photo2, INPUT);
  servo.attach(15);
  
  AHT10_finder(); //Sjekker om sensor og skjerm er tilkobla rett
  OLED_finder();
  justerPanel(); //Panelet justerast ved setup
}


void reconnect() { //Kobler til nettet om det har fallt ut
  // Loopen fortsetter til wifi er koblet til
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Prøver å koble til
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe til solcellepaneljustering når vi koblest til
      client.subscribe("esp32/solcellejuster");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Prøver igjen om 5sek
      delay(5000);
    }
  }
}

void Temphum(){ //Finner fuktigheit og temperatur og publisera til MQTT
    //Henter verdier fra sensoren
    sensors_event_t humidity, temp;
    int humidity_value;
    aht.getEvent(&humidity, &temp);
    // Hentar ut temperaturen i celsius
    float temperature = temp.temperature;  
    // Konvertera temperaturen til sting so dtostrf
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    display.setCursor(0, 0);
    display.print(tempString); //Viser temperaturen på oled skjermen
    display.print(" Celcius");
    display.display();
    client.publish("esp32/temperature", tempString); //publiserer til mqtt
    //Tar viften på hvis temperaturen overstiger 25 grader
    if(temperature > 25){
      digitalWrite(vifte, HIGH);}
    else{
      digitalWrite(vifte, LOW);}

    //Henter ut luftfuktigheitsverdien
    humidity_value = humidity.relative_humidity;
    // Konvetera til korrekt datatype
    char humString[8];
    dtostrf(humidity_value, 1, 2, humString);
    //Displayer verdiene på skjermen
    display.setCursor(0, 20);
    display.print(humString);
    display.print(" %");
    display.display();
    //Publiserer fuktigheitsverdien i MQTT
    client.publish("esp32/humidity", humString);
}


void solcellepanel(){ //Denne funksjonen regner ut strømprisen
  A = analogRead(photo1); //Leser av pinnene med photoresistorer
  B = analogRead(photo2);
  float regnetal = abs(A-B); //Bruker absoluttverdien av differansen som utgangspunkt for regning
  regnetal = map(regnetal, 0, 4095, 0.1, 100);
  regnetal = 12*log(regnetal)+5; //Logaritmisk funksjon virker naturlig
  char stromPris[8]; //Omgjør datatypen før publisering
  dtostrf(regnetal, 1, 2, stromPris);
  client.publish("esp32/solcelle", stromPris); //Publiserer strømprisen
 }

void loop(){  
  if (!client.connected()) {
    reconnect(); //Kobler til Wifi dersom det har fallt ut
  }
  client.loop();
  
  long now = millis();
  if (now - lastMsg > period) { //Kjører hver periode
    display.clearDisplay(); //Rydder skjerm 
    display.setCursor(0, 10);
    lastMsg = millis(); //Oppdaterer lastMsg
    Temphum(); //Kjører temperatur og humidity funksjonen
    solcellepanel(); //Kjører solcellepanelfunksjonen
  }
  delay(500); //Venter litt for å unngå kluss
}

//Koden inneholder elementer innspirert av denne guiden:
//https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
