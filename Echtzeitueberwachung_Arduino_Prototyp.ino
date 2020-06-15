#include <EEPROM.h>
#include "Adafruit_FONA.h"
#include "BlueDot_BME680.h"
#include "RTClib.h"
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <MPU6050.h>
//#include <SD.h>
//#include <SPI.h>

#define FONA_RX   2
#define FONA_TX   3
#define FONA_RST  4

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);     //SoftwareSerial für Kommunikation zwischen dem Arduino und Fona808
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

BlueDot_BME680 bme680 = BlueDot_BME680();                     //Abkürzung für den Umgebungsensor BME680

MPU6050 mpu;
RTC_DS3231 rtc;
#define PIN       9
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


//Variablen Definition


char phonenumber[16] = {'', '', '', '', '', '', '', '', '', '', '', '', '', '', ''};     
//char phonenumber[16] = {'', '', '', '', '', '', '', '', '', '', '', '', '', '', ''};   
bool smsbat = false;              //Variable für einmaliges versenden der SMS
bool smstemp = false;             //Variable für einmaliges versenden der SMS
bool smsHasFallen = false;        //Ändert sich nach jedem Fall wieder auf false und setzt die Anzahl der Fälle hoch
bool lockstatus = true;           //Gibt wieder ob das Objekt offen oder abgeschlossen ist.
uint8_t smsnumber = 1;            //Start der SMS Nummer zum Auslesen
uint16_t batP;                    //Akkustand
uint8_t humidity;                    //Luftfeuchtigkeit
float latitude, longitude, temperature;
String baseurl = "http://api.thingspeak.com/update.json?api_key=*****************";     //Deklaration Datahoster & API Key
uint16_t FreeFallTime = 0;        // Counter für die Zählung der kontinuierlichen Fälle
uint16_t NumTempCounter = 0;      //Anzahl der Temperaturüberschreitungen
float xa;                         //Normalisierter Vektor Accelerometer X-Achse
float ya;                         //Normalisierter Vektor Accelerometer Y-Achse
float za;                         //Normalisierter Vektor Accelerometer Z-Achse
uint8_t NumberOfDrops = 0;        //Anzahl der Fälle
char replybuffer[50];            //Speicher für eingehende SMS
const int chipSelect = 10;        //Pin Auswahl für SD-Karte
uint8_t ErrorCount = 0;           // Counter for check writing to EEPROM
uint8_t ErrorCheck = 0;           // Check Counter for wrting to EEPROM


//Variablen für Timer
unsigned long timeUpload = 0;
unsigned long timeBattery = 0;
unsigned long timeTemp = 0;
unsigned long timeGPS = 0;
unsigned long timeFallcheck = 0;
unsigned long timeGY521 = 0;
unsigned long timeUnlockLock = 0;
unsigned long timeOfflineSave = 0;
unsigned long timeEEPROMW = 0;
unsigned long timeEEPROMR = 0;
unsigned long timerBattery = 1000.0 * 60.0 * 5.1;       //5 min --Es muss sich zwingend um Fließkomma zahlen handeln, ansonsten werden falsche Werte gesichert
unsigned long timerUpload = 1000.0 * 60.0 * 4.1;        //Upload              : 4.1 min
unsigned long timerTemp = 1000.0 * 60.0 * 3.3;          //Temperaturcheck     : 3.3 min
unsigned long timerGPS = 1000.0 * 60.0 * 2.1;           //GPScheck            : 2.1 min
unsigned long timerFallcheck = 1000.0 * 60.0 * 1.1 ;    //SMS bei Fall        : 1.1 min
unsigned long timerGY521 = 50.0;                        //Fallcheck           : 50 Millisekunden
unsigned long timerUnlockLock = 1000.0 * 60.0 * 10.0;   //Unlock/Lock         : 10 min
//unsigned long timerOfflineSave = 1000.0 * 60.0 * 15.0; //SD-Card            : 15 min
unsigned long timerEEPROMW = 1000.0 * 60.0 * 1.0;       //EEPPROM Speicherung : 1 min
unsigned long timerEEPROMR = 1000.0 * 60.0 * 30.0;      //EEPROM              : 30 min

//Variablen für EEPROM

int a = 0;              //Variable für read Funktion
int eeAddress = 0;      //Der Beginn der zu 


struct EEPROMsave {
  uint16_t day2;
  uint16_t month2;
  uint16_t year2;
  uint16_t hour2;
  uint16_t minute2;
  uint16_t second2;
  bool lock;
  float temp;
  uint8_t humi;
  float lat;
  float lon;
  uint16_t bat;
  uint16_t drops;
  uint8_t error2;
  





};

void setup() {
  Serial.println(F("Initialisierung der Module gestartet"));
  //initiiere Software-Serial für die Kommuniukation mit dem Rechner
  initSERIAL();
  delay(1000);              //Delay, für eine bessere Verfolgung der Schritte und der LED Azeige im SerialMonitor

  //initiiere Anzeigeelement NeoPixel Stick
  initPIXEL();
  delay(1000);

  //initiiere Real-Time-Clock
  initRTC();
  delay(1000);

  //initiiere Kommunikation mit FONA808
  initFONA();
  delay(1000);

  //initiiere die GPR-Internet Verbindung des FONA808
  initGPS();
  delay(1000);

  //initiiere die GPRS-Internet Verbindung des FONA808
  initGPRS();
  delay(1000);

  //initiiere Temperatursensor BME680
  initBME680();
  delay(1000);

  //initiiere Gyrosensor GY-521
  initGY521();
  delay(1000);

  //initiiere SD CarD Reader
  //initSDCardReader();
  //delay(1000)


  Serial.println(F("Initialisierung abgeschlossen"));

  delay(1000);

  Serial.println(F("Testlauf für Funktion wird gestartet"));
  //Prüft Freien Fall und schaltet LED für den Start auf Grün: Indiziert, dass noch kein Freier Fall stattgefunden hat
  checkGefallen();
  delay(1000);

  //Prüft, ob GPS Daten empfangen worden sind
  checkGPS();
  delay(1000);
  
 //Lädt die ausgelesenen Sensordaten auf Thingspeak.com hoch
  uploadData();
  delay(1000);
  
    //Prüft den Akkustand der Batterie
  checkBAT();
  delay(1000);

  //Prüft die Temperatur
  checkTEMP();
  
  //eraseEE();
 
  while (a <= 1000) {    //Auslesen von bis zu 20 Fehlerlogs des letzten Durchlaufs aus dem EEPROM
      readEE();
    }
          //Löscht den EEPROM bei Neustart des Programmes
                      //Sollte auskommentiert werden, wenn Fehler ausgelesen werden sollen

  Serial.println(F("Testlauf abgeschlossen"));
  Serial.println(F("Setup abgeschlossen"));
}


void loop()
{
  
  unsigned long aktuelleZeit = millis();

  //Timer Upload
  if (aktuelleZeit - timeUpload >= timerUpload) {
    timeUpload = millis();
    uploadData();
  }
  //Timer Akkustand
  if (aktuelleZeit - timeBattery >= timerBattery) {
    timeBattery = millis();
    checkBAT();
  }
  //Timer Temperaturprüfung
  if (aktuelleZeit - timeTemp >= timerTemp) {
    timeTemp = millis();
    checkTEMP();
  }
  //Timer GPS-Abfrage
  if (aktuelleZeit - timeGPS >= timerGPS) {
    timeGPS = millis();
    checkGPS();
  }
  //Timer Freier Fall
  if (aktuelleZeit - timeGY521 >= timerGY521) {     //timer auf 50ms eingestellt. Freier Fall nach 150ms Falldauer
    timeGY521 = millis();
    ErkennungFreierFall();
  }
  //Timer für SMS, falls gefallen
  if (aktuelleZeit - timeFallcheck >= timerFallcheck) {
    timeFallcheck = millis();
    checkGefallen();
  }
  //Timer für Öffnung und Schließung
  if (aktuelleZeit - timeUnlockLock >= timerUnlockLock) {
    timeUnlockLock = millis();
    LockUnlock();
  }

  //Timer Speicherung im EEPROM
  if (aktuelleZeit - timeEEPROMW >= timerEEPROMW) {
    timeEEPROMW = millis();
    if (eeAddress <= 1000) {                 // Länge des EEPROM auf einem Arduino Pro Mini ist 1012 bytes
      writeEE();      
    }
    else {
      eraseEE();
      eeAddress = 0;                        // Wird wieder auf 0 gesetzt, es sind 20 Speichervorgänge möglich
    }
  }
  //*/

  //Auslesen des EEPROM - kann bei Bedarf wieder eingefügt werden.
  /*
    if (aktuelleZeit - timeEEPROMR >= timerEEPROMR) {
    timeEEPROMR = millis();
    while (a <= 550) {
      readEE();
    }
    a = 0;
    }
  */
  /*
    if(aktuelleZeit - timeOfflineSave >= timerOfflineSave){
    timeOfflineSave = millis();
    SDOfflineSave();
    }
  */
}    //Ende Loop

void checkTEMP() {
  bme680.writeCTRLMeas();
  temperature = bme680.readTempC();
  Serial.println(temperature);
  
  //Die Temperaturgrenzwerte können je nach Bedarf angepasst werden

  if (temperature <= 10 && NumTempCounter <= 3)                         //Die LED für die Temperatur springt nach überschreiten des Grenzwertes nicht mehr zurück
    pixels.setPixelColor(7, pixels.Color(0, 0, 255));                 //DeepSkyBlue

  if (temperature <= 15 && temperature > 10 && NumTempCounter <= 3)
    pixels.setPixelColor(7, pixels.Color(0, 191, 255));                 //SpringGreen

  if (temperature <= 20 && temperature > 15 && NumTempCounter <= 3)
    pixels.setPixelColor(7, pixels.Color(150, 255, 50));                //Green

  if (temperature <= 25 && temperature > 20 && NumTempCounter <= 3)     //Gold
    pixels.setPixelColor(7, pixels.Color(255, 215, 0));                  

  if (temperature <= 30 && temperature > 25 && NumTempCounter <= 3)     //OrangeRed1
    pixels.setPixelColor(7, pixels.Color(255, 69, 0));
    
  if (temperature >= 30)
    pixels.setPixelColor(7, pixels.Color(205, 0, 0));                   //Rot
  NumTempCounter ++;                                                  //Legt fest, wie oft die Temperatur für eine Warnung überschritten werden muss (korreliert mit dem Timer der Temperaturabfrage)

  if (NumTempCounter > 3 && temperature > 30 && smstemp == false) {
    String SMStemp = String(F("Die zulaessige Hoechsttemperatur von 
    30 Grad wurde ueberschritten."));
    char cSMStemp[SMStemp.length() + 1];
    SMStemp.toCharArray(cSMStemp, SMStemp.length() + 1);
    delay(200);
    fona.sendSMS(phonenumber, cSMStemp);                              //sendet einmalig eine SMS, wenn der Temperaturgrenzwert eine gewisse Anzahl überschritten wurde
    pixels.setPixelColor(7, pixels.Color(205, 0, 0));
    smstemp = true;
    NumTempCounter = 0;
    ErrorCount ++;
  }
  pixels.show();
}

void checkBAT() {
  fona.getBattPercent(&batP);

  if (batP >= 40)
    pixels.setPixelColor(6, pixels.Color(0, 150, 0));      //grün

  if (batP < 40 && batP > 2)
    pixels.setPixelColor(6, pixels.Color(255, 255, 0));    //gelb

  if (batP <= 2 && smsbat == false) {
    String SMS = String(F("Akku liegt unter 2%"));        //sendet einmalig eine SMS, wenn die Batterie unter 20% fällt
    char cSMS[SMS.length() + 1];
    SMS.toCharArray(cSMS, SMS.length() + 1);
    delay(200);
    fona.sendSMS(phonenumber, cSMS);
    pixels.setPixelColor(6, pixels.Color(205, 0, 0));      //rot
    smsbat = true;
    ErrorCount++;
  }
  pixels.show();
}

void checkGefallen() {             //Prüft ob das Objekt runtergefallen ist und sendet eine SMS
  if (smsHasFallen == true) {
    NumberOfDrops ++; //Die Anzahl der Fälle werden in dieser Variable gesichert
    String SMSFall = String("Gefallen und " + String(NumberOfDrops) + " Mal heruntergefallen");
    char cSMSFall[SMSFall.length() + 1];
    SMSFall.toCharArray(cSMSFall, SMSFall.length() + 1);
    delay(200);
    fona.sendSMS(phonenumber, cSMSFall);
    pixels.setPixelColor(4, pixels.Color(205, 0, 0));     //rot
    smsHasFallen = false;                //bool wird zurückgesetzt, damit der nächste Freie Fall erkannt werden kann
    ErrorCount ++;
  }
  if (smsHasFallen == false) {
    pixels.setPixelColor(4, pixels.Color(0, 150, 0));     //grün
  }
  pixels.show();
}

void uploadData() {
bme680.writeCTRLMeas();
  fona.getBattPercent(&batP);
  temperature = bme680.readTempC();
  humidity = bme680.readHumidity();
  fona.getGPS(&latitude, &longitude);     //zieht einzelne Werte aus der GPS-Abfrage

  // Zusammenbau des Strings aus URL, API und Sensordaten. Strings benötigen mehr Speicher, sind aber einsteigerfreundlicher
  String url = String(baseurl + "field1="     + batP + "&"                    //Batterie
                      +"field2="        + temperature + "&"                   //Temperatur
                      + "field3="       + humidity + "&"                      //Luftfeuchtigkeit
                      + "field4="       + String(longitude, 5) + "&"          //Längengrad mit 5 Nachkommastellen
                      + "field5="       + String(latitude, 5)  + "&"          //Breitengrad mit 5 Nachkommastellen
                      //+ "field6="       + lockstatus                        //Wenn diese oder eine andere an der sechsten Stelle Variable mitgesendet wird verhält ist die HTTP Verbindung instabil und wird nicht jedes mal korrekt ausgeführt
                     );
  Serial.println(url);

  char charUrl[url.length() + 1];                   //Konvertierung des Strings in ein Char Array, welches für die HTTP Post Funktion benötigt wird
  url.toCharArray(charUrl, url.length() + 1);
  delay(500);
  // Start HTTP POST request (Daten werden als URL hochgeladen, deswegen ein leeres Char Array für den body)
  char postData[0];
  uint16_t statusCode;
  int16_t length;
  bool successful = fona.HTTP_POST_start(
                      charUrl,
                      F("text/plain"),
                      (uint8_t *) postData,
                      strlen(postData),
                      &statusCode,
                      (uint16_t * ) & length);

  delay(5000);          //Warten auf Antwort der Website und ausgabe des HTML Statuscodes
  if (!successful) {
    pixels.setPixelColor(5, pixels.Color(205, 0, 0));
    pixels.show();
    ErrorCount++;
  }
  if (successful)
  pixels.setPixelColor(5, pixels.Color(0, 150, 0));
  pixels.show();
  fona.HTTP_POST_end();
}

void checkGPS() {
  uint8_t GPSstat ;
  GPSstat = fona.GPSstatus();           
  //fona.getGPS(&latitude, &longitude);
  if (GPSstat == 3) {          //Laut Softwarebibliothek des Fona808 der Wert, wenn das FonaV2 eine Verbindung aufgebaut hat
    pixels.setPixelColor(1, pixels.Color(0, 150, 0)); // LED 2 auf grün
    pixels.show();
  }
  else {
    pixels.setPixelColor(1, pixels.Color(205, 0, 0)); //  LED 2 auf rot
    pixels.show();
  }
  /*    //Ändert sich nicht mehr zurück, wenn die GPS-Verbindung verloren geht
  if (latitude == 0 && longitude == 0) {          //wenn die Variablen leer sind, besteht keine GPS Verbindung. Besserer Indikator als ob das GPS aktiviert wurde
    pixels.setPixelColor(1, pixels.Color(205, 0, 0)); // setzt LED 2 auf rot, wenn keine Verbindung besteht
    pixels.show();
  }
  else {
    pixels.setPixelColor(1, pixels.Color(0, 150, 0)); // setzt LED 2 auf grün, wenn die Werte gefüllt sind
    pixels.show();
  }
  */
}

void ErkennungFreierFall() {
  Vector normAccel = mpu.readNormalizeAccel();        //Auslesen der normalisierten Vektoren und Sicherung in einzelne Variablen
  xa = normAccel.XAxis;
  ya = normAccel.YAxis;
  za = normAccel.ZAxis;
  float TotalAcc = sqrt(xa * xa + ya * ya + za * za); //Errechnung der absoluten Beschleunigung in einem Wert

  if (TotalAcc < 1) {                                 //wenn TotaLAcc unter 1 ist, befindet sich das Objekt in einem freien Fall
    FreeFallTime ++;                                  //Erhöht den Zähler um 1. Bei Abfrage alle 50ms muss der Zähler bis 3 hochzählen
  }
  else {
    FreeFallTime = 0;
  }
  if (FreeFallTime >= 3) {    // Objekt wird als gefallen gewertet, ab 150ms freier Fall, da die Funktion alle 50ms abgefragt wird.
    Serial.println(F("ICH FALLE"));
    smsHasFallen = true;
  }
}

void LockUnlock() {
  if (lockstatus == true) {
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));       //setzt LED 1 auf Grün, wenn das Gerät offen ist
    pixels.show();
  }
  else {
    pixels.setPixelColor(0, pixels.Color(205, 0, 0));       //setzt LED 1 auf rot, wenn das Gerät geschlossen ist
    pixels.show();
  }

  smsnumber = fona.getNumSMS();
  uint16_t smslen;
  fona.readSMS(smsnumber, replybuffer, 50, &smslen);
  Serial.println(replybuffer);

  if (replybuffer[0] == 'o' && lockstatus == false) {     //nur der erste Buchstabe des Char arrays wird ausgelesen; FEHLER
    lockstatus = true;
    String SMSStatus = String(F("Der Prototyp wurde aufgeschlossen"));
    char cSMSStatus[SMSStatus.length() + 1];
    SMSStatus.toCharArray(cSMSStatus, SMSStatus.length() + 1);
    delay(200);
    fona.sendSMS(phonenumber, cSMSStatus);
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  }
  if (replybuffer[0] == 'c' && lockstatus == true) {
    lockstatus = false;
    String SMSStatus2 = String(F("Der Prototyp wurde abgeschlossen"));
    char cSMSStatus2[SMSStatus2.length() + 1];
    SMSStatus2.toCharArray(cSMSStatus2, SMSStatus2.length()  + 1);
    delay(200);
    fona.sendSMS(phonenumber, cSMSStatus2);
    pixels.setPixelColor(0, pixels.Color(205, 0, 0));
  }
}

void initSERIAL() {
  while (!Serial);
  Serial.begin(9600);
  Wire.begin();
}

void initRTC() {
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));     //synchronisiert die RTC mit der Kompilierzeit des Sketches
}

void initFONA() {
  fonaSerial->begin(4800);
  if (!fona.begin(*fonaSerial)) {
    pixels.setPixelColor(0, pixels.Color(205, 0, 0));       //setzt LED 4 auf Grün
    pixels.show();
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  else {
    fona.setGPRSNetworkSettings(F("internet"));        //"web.vodafone.de" für vodafone ; "internet" für o2
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));       //setzt LED 1 auf Grün, wenn die Verbindung mit dem Fona808 hergestellt werden konnte
    pixels.show();
  }

}

void initBME680() {
  bme680.parameter.I2CAddress = 0x77;                     //deklariert die I2C-Adresse
  bme680.parameter.sensorMode = 0b01;
  bme680.parameter.humidOversampling = 0b101;
  bme680.parameter.tempOversampling = 0b101;
  bme680.parameter.tempOutsideCelsius = 15;
  bme680.init() != 0x61;
  if (bme680.init() != 0x61) {                              //Änderung - auf Funktionalität prüfen
    pixels.setPixelColor(3, pixels.Color(205, 0, 0));       //setzt LED 4 auf Grün
    pixels.show();
  }
  else {
    pixels.setPixelColor(3, pixels.Color(0, 150, 0));       //setzt LED 4 auf Grün
    pixels.show();
  }
  bme680.writeCTRLMeas();
}

void initGPRS() {
  fona.enableGPRS(true);
  pixels.setPixelColor(2, pixels.Color(0, 150, 0));       //setzt LED 3 auf Grün
  pixels.show();
}

//Aktivierung GPS
void initGPS() {
  fona.enableGPS(true);
  pixels.setPixelColor(1, pixels.Color(0, 150, 0));       //setzt LED 2 auf Grün
  pixels.show();
}

//Aktivierung des NeoPixel Stick
void initPIXEL() {
  pixels.begin();
  pixels.clear();
  pixels.show();
  pixels.setBrightness(50);         //Setzt die Helligkeit des NeoPixel Stick auf einen Wert zwischen 0 und 255
}

//Aktivierung des Gyrosensors
void initGY521() {
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x69))
  {
    Serial.println(F("Anschlussfehler"));
    delay(1000);
  }
}

//SD Card ist auskommentiert, da der Speicher des Arduino nicht ausreicht
/*
  void initSDCardReader(){
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");

  }
  Serial.println("card initialized.");
  }

  void SDOfflineSave(){

  // Aktualisiere alle Datenwerte

  bme680.writeCTRLMeas();
  DateTime time = rtc.now();
  String datum = time.timestamp();
  fona.getBattPercent(&batP);
  temperature = bme680.readTempC();
  humidity = bme680.readHumidity();
  fona.getGPS(&latitude, &longitude);

  //Erstellung des Datenstrings für die Speicherung
  String DataOffline = String(datum) + ";"
                      +String(batP) + ";"
                      +String(temperature) + ";"
                      +String(humidity) +";"
                      +String(latitude) +";"
                      +String(longitude) +";"
                      +String(NumberOfDrops);

  //öffnet die Datei auf der SD Karte - es kann immer nur eine Datei gleichzeitig geöffnet sein
  File dataFile = SD.open("PrototypOfflinespeicherung.txt", FILE_WRITE);

  //Sichert die Daten und schließt die Datei daraufhin
  if (dataFile) {
    dataFile.println(DataOffline);
    dataFile.close();
    //Ausgabe der gesicherten Daten an den Serial Monitor
    Serial.println(DataOffline);
  }
  else {
    Serial.println(F("Fehler bei dem Oeffnen der Textdatei");
  }
  }
*/
void writeEE() {
  Serial.println(ErrorCount);
  if (ErrorCount > ErrorCheck) {
    ErrorCheck = ErrorCount;
    EEPROMsave s{};
    bme680.writeCTRLMeas();
    fona.getBattPercent(&batP);
    temperature = bme680.readTempC();
    fona.getGPS(&latitude, &longitude);
    DateTime time = rtc.now();
    s.day2 = time.day();
    s.month2 = time.month();
    s.year2 = time.year();
    s.hour2 = time.hour();
    s.minute2 = time.minute();
    s.second2 = time.second();
    s.lock = lockstatus;
    s.temp = temperature;
    s.humi = humidity;
    s.lat = latitude;
    s.lon = longitude;
    s.bat = batP;
    s.drops = NumberOfDrops;
    s.error2 = ErrorCount;
    EEPROM.put(eeAddress, s);
    eeAddress = eeAddress + 50;
  }
}


void readEE() {           //Auslesen des EEPROM
  EEPROMsave s{};
  EEPROM.get( a, s );
  Serial.print(F( "Read EEPROM: AdressStart:" ));
  Serial.println( a);
  Serial.println(s.day2);
  Serial.println(s.month2);
  Serial.println(s.year2);
  Serial.println(s.hour2);
  Serial.println(s.minute2);
  Serial.println(s.second2);
  Serial.println(s.lock);
  Serial.println(s.temp);
  Serial.println(s.humi);
  Serial.println(s.lat, 5);   //Ausgabe der GPS-Daten mit 5 Dezimalstellen
  Serial.println(s.lon, 5);   //Ausgabe der GPS-Daten mit 5 Dezimalstellen
  Serial.println(s.bat);
  Serial.println(s.drops);
  Serial.println(s.error2);

  a = a + 50;
}

void eraseEE() {
  for (a = 0 ; a < EEPROM.length() ; a++) {
    EEPROM.write(a, 0);
  }
  a = 0;
}
