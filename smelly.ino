/*
Verwendete Portpins am Arduino:
2: INPUT: SW2 Taster/Jumper z.B. für Deaktivierung akustische Signalisierung (low-aktiv, hardwareentprellt)
3: INPUT: Programmiertaster (low-aktiv, hardwareentprellt)
4: INPUT: SW1 Taster zum Umschalten Bildschirm, Quittieren Signalton usw. (low-aktiv, hardwareentprellt)
5: OUTPUT: RGB LED Rot (low-aktiv, PWM möglich, 0: max, 255: min=aus)
6: OUTPUT: TFT Backlight Stromversorgung (low-aktiv, PWM möglich, 0: max, 255: min=aus)
7: OUTPUT: A0 TFT
8: OUTPUT: CS TFT
9: OUTPUT: RGB LED Grün (low-aktiv, PWM möglich, 0: max, 255: min=aus)
10: OUTPUT: RGB LED Blau (low-aktiv, PWM möglich, 0: max, 255: min=aus)
11: OUTPUT: SDA TFT
12: OUTPUT: RESET TFT 
13: OUTPUT: SCK TFT
A0: INPUT: Ambient Light Sensor (TEPT4400, 0: dunkel, 1023: hell)
A1: OUTPUT: Summer (low-aktiv)
A2: OUTPUT: AMS VOC Stromversorgung (low-aktiv)
A3: OUTPUT: TFT Elektronik Stromversorgung (high-aktiv)
A4: INPUT: I2C SDA
A5: OUTPUT: I2C SDL
A6: NC
A7: NC
Anmerkung: anstelle des Summers kann in V1.4 auch ein zweiter Taster bestückt werden (gleiche Funktion wie Jumper SW2 am Input-Portpin 2).
Der Jumper rechts kann bei Bestückung des zweiten Tasters also nicht separat benutzt werden!
*/
//****************************************************************************
#include <TFT_ST7735.h> // Achtung: Lib angepasst an das 1,44" TFT
#include <Adafruit_SHT31.h>
#include <KnxTpUart.h>

//USER Setup                                                      //KNX Messwerte/Betriebswerte GAs müssen auf die Gegenheiten angepasst werden
#define KNX_PA               "1.0.180"                            //KNX PA
#define KNX_GA_STATUS        "6/3/8"                              //Status IAQ
#define KNX_GA_CO2           "4/0/100"                            //CO2-Messwert
#define KNX_GA_TVOC          "4/0/101"                            //VOC-Messwert
#define KNX_GA_TEMP          "3/1/8"                              //Temperaturmesswert
#define KNX_GA_HUM           "4/0/8"                              //Feuchtemesswert

#if (RAMEND < 1000)                                               //Serial Buffer bei genügend RAM vergrößern
    #define SERIAL_BUFFER_SIZE 64
#else
    #define SERIAL_BUFFER_SIZE 256
#endif


bool Taup_Anz=false;                                              //Taupunkt in Sektion 5 dauerhaft anzeigen
bool Lux_Anz=false;                                               //Helligkeitswert anzeigen, Wert wird reziprog angezeigt 
bool StatusiAQ_Anz=false;                                         //Status iAQ anzeigen 
bool LEDs_Anz=false;                                              //RGB LED anzeigen für Luftqualität CO2 nach Grenzwerten
bool LEDB_Anz=false;                                              //Blaue LED anzeigen für Feuchtigkeit > 65%
bool KNX_receive=true;                                            //KNX Empfang einschalten
byte map_LED_min=240;                                             //LED minimale Helligkeit
byte map_LED_max=50;                                              //LED maximale Helligkeit
byte map_TFT_min=250;                                             //TFT minimale Helligkeit
byte map_TFT_max=0;                                               //TFT maximale Helligkeit       
int brightness=900;                                               //erstmal feste Helligkeit des Backlight
unsigned long intervaliAQ =                 11500;                //Aktualisierungsintervall iAQ in ms
unsigned long intervalTempHum =              5000;                //Aktualisierungsintervall SHT31 in ms
unsigned long intervalambi =                 2000;                //Intervall Backlightregelung und LEDs
float temp_send_offset=0.2;                                       //Temperaturänderung ab wann ein neuer Wert geschrieben werden soll
float hum_send_offset=1;                                          //Temperaturänderung ab wann ein neuer Wert geschrieben werden soll
float offsetTemp=1.8;                                             //Temperatursensor Offset
float offsetHum=0;                                                //Feuchtesensor Offset
String mClass[7]={"TxT","C","H","CO2","VOC","TAU","LUX"};         //Messwertart in der jeweiligen Sektion 
int GrenzW1CO2=1000; int GrenzW2CO2=2000;                         //Grenzwerte CO2
int GrenzW1VOC=410; int GrenzW2VOC=830;                           //Grenzwerte VOC
String s_alarm = "LUEFTEN";                                       //Meldetext max. 10 Zeichen bei Größe 2
// Farben einstellen für                  Sektion 1       Sektion 2              Sektion 3                      Sektion 4            GW2       Sektion 5
unsigned int ColorSecMain[6][3]=
  {{0,0,0},
  {TFT_WHITE,0,0},
  {TFT_WHITE,TFT_BLUE,0},
  {TFT_GREEN,TFT_YELLOW,TFT_RED},
  {TFT_GREEN,TFT_YELLOW,TFT_RED},
  {TFT_RED,TFT_CYAN,TFT_GREEN}};
unsigned int BackgSecMain[6]=
{0,
TFT_BLACK,
TFT_BLACK,
TFT_BLACK,
TFT_BLACK,
TFT_BLACK};                                         //Hintergrundfarbe Messwertfenster
byte FontHMess[6][2]={{},{3,0},{3,0},{2,0},{2,0},{2,2}};                                                                    //Fontgröße Messwerte
byte FontHEinh[6]={0,1,1,1,1,1};                                                                                            //Fontgröße Einheiten
//eingebundene Bibliotheken

//Initialize KNX
KnxTpUart knx(&Serial, KNX_PA);

//Kurzdeklarationen
TFT_ST7735 tft = TFT_ST7735(); 
Adafruit_SHT31 sht31 = Adafruit_SHT31();
#define iaqaddress 0x5A
#define LED_G 9                       //LED Grün
#define LED_B 10                      //LED Blau
#define LED_R 5                       //LED Rot
#define PWM_BL_Pin 6                  //PWM Backlight
#define TFT_CS 8                      //Backlight TFT
#define SW1 4                         //Taster SW1
#define SW2 2                         //Taster SW2
#define INPUTMODE INPUT               //INPUT oder INPUT_PULLUP
int ambientlight,oldambientlight,oldambientlight2,ambi_mapped,LED_mapped,airQuality,airTvoc,oldairQuality,oldairTvoc =0;
float temp,oldTemp,minTemp,maxTemp,hum,minHum,maxHum,oldHum,tp,oldtp = 0;
bool dispRenew,result,alarm, readCO2, readVOC, readT, readH = false; 
char m[4],ppm[4]="ppm",ppb[4]="ppb", CO2[4]="CO2", VOC[4]="VOC", empty[1]="", Taup[4]="TAU", Min[4]="Min", Max[4]="Max", LUX[3]="lx", Stat[4]="iAQ", PA[3]="PA", Ven[4]="VEN";
char centi[5]="\367C"; char perc[2]="%";
unsigned long prevMillisSHT, prevMillisiAQ, prevMillisiAQAnz,  prevMillisambi,last = 0; 
byte secYPos[6][2]={{},{32,48}, {60,75}, {87,100}, {111,123}, {134,148}};                    //{1,] yPos Fläche, [,1] yPos Messwert
bool secRenew[6]={false,false,false,false,false,false};
bool secTxtRenew[7]={false,false,false,false,false,false,false};
byte oldlenFl[7]={0,0,0,0,0,0,0};
byte h,Sec5_Anz,Valve,oldSec5_Anz,iAQstatus,oldiAQstatus,oldiAQstatus2,BA,oldBA=0;
String ambi_str,temp_str,hum_str,StatiAQ_str,CO2_str,VOC_str,Tau_str,Sek5BA_str="";

void hum_send_show();  
void iAQ_send_show(); 
bool CO2LimitLow();
bool CO2LimitMiddle();
bool CO2LimitHigh();
bool VOCLimitLow();
bool VOCLimitMiddle();
bool VOCLimitHigh();
String float2str(float f, int n);


void setup() {
  setKNX();                                           //KNX Init 
  setHW();                                            //Hardware Init 
  oldiAQstatus=255;                                   //iAQStatus setzen, damit der erste Wert gesetzt wird 
  pinMode(PWM_BL_Pin, OUTPUT);                        //PWM Backlight als Output setzen 
  analogWrite(PWM_BL_Pin, brightness);                //Backlight erstmal aus lassen
  tft.begin();                                        //TFT aktivieren
  sht31.begin(0x44);                                  //SHT31 aktivieren
  initDisplay();                                      //Display initialisieren 
  oldtp=0;
}



void loop() {     
  //Temperatur und Luftfeuchte 
  if (millis() - prevMillisSHT > intervalTempHum) {     //Ausleseinterval überprüfen
            prevMillisSHT = millis();                   //aktuelle Zeit abspeichern
            temp = sht31.readTemperature()-offsetTemp ; //Temperatur lesen und Offset berücksichtigen
            if (temp >= (oldTemp + temp_send_offset) || temp <= (oldTemp - temp_send_offset)) {       //Temperatur: Wenn die Änderung mehr oder gleich 0.2°C beträgt
                temp_send_show();
            }
            hum = sht31.readHumidity()-offsetHum;    //Feuchte lesen und Offset berücksichtigen
            if (hum >= (oldHum + hum_send_offset) || hum <= (oldHum - hum_send_offset)) {        //Feuchte: Wenn die Änderung mehr oder gleich 1% beträgt
                hum_send_show();
            }         
  }    
  //iAQ auslesen: 11.5 Sekunden warten (der VOC Core P liefert nur alle 11s neue Werte)       
  if (millis() - prevMillisiAQ > intervaliAQ) {    //Ausleseinterval überprüfen
          prevMillisiAQ = millis();                //aktuelle Zeit abspeichern
          iAQ_send_show();    
  }
                                                                                                                    
  if (millis() - prevMillisambi > intervalambi) {     //Ausleseinterval überprüfen
    prevMillisambi = millis();                           
    ambientlight = analogRead(A0)*0.65;               //Helligkeitswert einlesen
    if (ambientlight >= (oldambientlight)+2 || ambientlight < (oldambientlight)-2) {      //Bei Helligkeitswertänderung von mehr oder gleich 2 Lux
        oldambientlight=ambientlight;                                                       //Helligkeitswert merken
        ambientlight_control();
    } 
  }                                                                                                                                       

// Sektion 5 beschreiben
  if ((airTvoc >= GrenzW2VOC)||(airQuality >= GrenzW2CO2)) {
    alarm=true; 
    Sec5_Anz=1; 
  }     //Bei Grenzwertüberschreitung Meldetext anzeigen    
  if ((airTvoc < GrenzW2VOC)&&(airQuality < GrenzW2CO2)&&(alarm)) Sec5_Anz=2;   //Alarm rücksetzen nach Grenzwertüberschreitungen
  if ((StatusiAQ_Anz)&&(!alarm)) Sec5_Anz=5;    //iAQ Status Anzeige freigegeben ?
  if ((Lux_Anz)&&(!alarm)) Sec5_Anz=4;  //Helligkeitswert Anzeige freigegeben ?
  if ((Taup_Anz)&&(!alarm)) Sec5_Anz=3;      //Taupunktanzeige freigegeben ?  
  if (Sec5_Anz!=oldSec5_Anz) { //Falls sich die Anzeige für Sektion 5 geändert, Text/Feld neu zeichen
    oldSec5_Anz=Sec5_Anz; 
    memset(secTxtRenew,0,sizeof(secTxtRenew));
  }     
    
  switch (Sec5_Anz) {       //Abfrage welche Anzeige Sektion 5
    case 1: 
      if (!secTxtRenew[1]) {secRenew[5]=false;       //Alarm entstanden
        if (alarm) {
            showSection(5,mClass[0],s_alarm,ColorSecMain[3][2],BackgSecMain[5],FontHMess[5][0],empty,FontHEinh[5],2,empty); //Sektion 5 mit Alarm beschreiben
            secTxtRenew[1]=true;         //Text neu zeichnen
        }
      }
      break;
    case 2: 
      if (! secTxtRenew[2]) {         //Feld leeren  
        tft.fillRect(0,133,128,26,BackgSecMain[5]);
        secTxtRenew[2]=true;           //beschrieben setzen
        alarm=false;
      }
      break;        
    case 3: 
      if (!secTxtRenew[3]) secRenew[5]=false;            //Taupunktanzeige freigegebwn
      if ((tp!=oldtp)&&(tp>0)&&(Taup_Anz)) {                 
        tft.fillRect(0,133,128,26,BackgSecMain[5]);                                                                          
        showSection(5,mClass[5],Tau_str,ColorSecMain[5][1],BackgSecMain[5],FontHMess[5][1],centi,FontHEinh[5],2,Taup);//Sektion 5 mit Taupunkt beschreiben
        secTxtRenew[3]=true;      //beschrieben setzen
        oldtp=tp;
      }
      break;
    case 4: 
      if (!secTxtRenew[4]) secRenew[5]=false;           //Helligkeitsanzeige freigegeben
        if ((ambientlight >= oldambientlight2+10) || (ambientlight < oldambientlight2-10)||(Lux_Anz)) {      //Helligkeit im Intervall anzeigen
            tft.fillRect(0,133,128,26,BackgSecMain[5]); 
            showSection(5,mClass[6],ambi_str,ColorSecMain[5][1],BackgSecMain[5],FontHMess[5][0],LUX,FontHEinh[5],1,empty);//Sektion 5 mit Helligkeitswert beschreiben
            secTxtRenew[4]=true;            //beschrieben setzen
            oldambientlight2=ambientlight;
        }
      break;
    case 5: 
      if (!secTxtRenew[5]) secRenew[5]=false;     //iAQ Status Anzeige freigegeben
      if ((iAQstatus!=oldiAQstatus2)||(StatusiAQ_Anz)) {        //iAQ Status bei Änderung refreshen
        tft.fillRect(0,133,128,26,BackgSecMain[5]);      //Feld leeren
        switch (iAQstatus) {   //iAQ Status auf Text mappen
          case 0: StatiAQ_str="Ok";break;
          case 1: StatiAQ_str="Busy";break;
          case 16: StatiAQ_str="Warmup"; break;
          case 128: StatiAQ_str="Error"; break;
        } 
        secTxtRenew[5]=true;             //beschrieben setzen
        oldiAQstatus2 = iAQstatus; 
        showSection(5,mClass[0],StatiAQ_str,ColorSecMain[5][2],BackgSecMain[5],FontHMess[5][0],Stat,FontHEinh[5],1,empty); //Sektion 5 mit iAQ Status beschreiben
      } 
      break;                       
  }   
} 

/////////////////////////////////////////////Routinen
void initDisplay() {
  //Achtung: durch einen Fehler in der LIB ist der Nullpunkt für X/Y bei 0/32).
  if (!dispRenew) {   //Wenn TFT initialisieren nicht erfolgt, auf einmalig setzen
    tft.fillScreen(TFT_BLACK);     //Schwarz füllen 
    tft.setRotation(0);           //hochkant 
    dispRenew=true;               //Initialisierung erfolgt 
  } 
}  
//showSection(Sektion Nr, Messwertart, Var, Vordergrundfarbe, Hintergrundfarbe, Schriftgröße, Einheit, Schriftgröße Einheit, Einheiten 1/2, Einheit Zeile 2);   
void showSection(byte sec_nr,String m,String s1,uint16_t color, uint16_t backg, byte t_size,char sym[], byte s_size, byte units, char sym2[]) { 
  char tch[s1.length()+1];
  byte lens1;
  if ((sec_nr==1) or (sec_nr==2)) {
    h=28;                       //Für Sektion 1 und 2 Höhe von 28 Pixel festlegen
    lens1=s1.length();          //Längenänderung prüfen
    if (lens1!=oldlenFl[1]) {   //Bei Stringlängenänderung Sektion beschreiben
        secRenew[sec_nr]=false;
        oldlenFl[1]=lens1;
    }
  }                                                                                                                                    
  if ((sec_nr==3) or (sec_nr==4)) h=23;  //Für Sektion 3 und 4 Höhe von 23 Pixel festlegen  
  if (sec_nr==5) h=26;                   //Für Sektion 5 Höhe von 25 festlegen
  if (m=="CO2") {                        //Wenn Messwert CO2  
      lens1=s1.length();                 //Stringlänge merken
      if (lens1!=oldlenFl[3]) {          //Bei Stringlängenänderung Sektion beschreiben 
          secRenew[sec_nr]=false;
          oldlenFl[3]=lens1;
      }
  }        
  if (m=="VOC") {                        //Wenn Messwert VOC
        lens1=s1.length();               //Stringlänge merken
        if (lens1!=oldlenFl[4]) {        //Bei Stringlängenänderung Sektion beschreiben   
            secRenew[sec_nr]=false;
            oldlenFl[4]=lens1;
        } 
  }
  if (m=="LUX") {                       //Wenn Messwert Helligkeit   
        lens1=s1.length();              //Stringlänge merken 
        if (lens1!=oldlenFl[6]) {       //Bei Stringlängenänderung Sektion beschreiben 
            secRenew[sec_nr]=false;
            oldlenFl[6]=lens1;
        } 
  }
  if (!secRenew[sec_nr]) {             //Falls die Sektion neu beschrieben werden soll
      tft.fillRect(0,secYPos[sec_nr][0],128,h,backg);    //Sektion leeren 
      secRenew[sec_nr]=true;            //beschrieben setzen   
  }  
  tft.setTextColor(color,backg);       //Textfarbe setzen 
  if (units==2) {s_size=1;}           //Bei 2 Einheiten Textgröße auf 1 fixieren
  tft.setTextSize(s_size);              //Textgröße setzen
  if (sec_nr==1) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);}  //Sektion 1 nach Größe beschreiben 
                                  if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                  if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);} }
                  if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-13),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])+1),1);}}
  if (sec_nr==2) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);} //Sektion 2 nach Größe beschreiben  
                                  if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                  if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);}  } 
                  if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-13),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])+1),1);}}
  if (sec_nr==3) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);} //Sektion 3 nach Größe beschreiben 
                                  if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                  if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);}  }
                  if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-11),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-1),1);}}
  if (sec_nr==4) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);} //Sektion 4 nach Größe beschreiben  
                                  if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                  if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);}  }
                  if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-11),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-1),1);}}
  if (sec_nr==5) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-4), 1);} //Sektion 5 nach Größe beschreiben      
                                  if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-8), 1);} 
                                  if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-10), 1);} }
                  if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-11),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-1),1);}}                    
  
  tft.setTextColor(color,backg);                                                                                                        
  tft.setTextSize(t_size);
  tft.setTextDatum(CC_DATUM);                                                                                         
  s1.toCharArray(tch, s1.length() + 1);      //String in array of char umwandeln
  tft.setTextWrap(false);                     //Textumbruch ausschalten
  if (t_size==1) {tft.drawCentreString(tch, 60, ((secYPos[sec_nr][1])-5), 1);}   //Text nach horizontal zentrieren entsprechend der Texthöhe 
  if (t_size==2) {tft.drawCentreString(tch, 60, ((secYPos[sec_nr][1])-9), 1);} 
  if (t_size==3) {tft.drawCentreString(tch, 60, ((secYPos[sec_nr][1])-12), 1);}
  
}
 
void setHW() {
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);      //TFT Elektronik ein
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);         //ADC für Helligkeitssensor aktivieren
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);      //VOC einschalten
  delay(300);
  pinMode(7, OUTPUT); 
  digitalWrite(7, HIGH);       //A0 am TFT high setzen
  pinMode(8, OUTPUT);            //CS für TFT  
  pinMode(4, INPUT);
}

void setKNX() {
  //initialize connection to KNX BCU
  Serial.begin(19200,SERIAL_8E1);   // Even parity
  while (!Serial) {
    ;                               // wait for serial port to connect
  }
                               // Reset UART
  if (Serial.available()) {
    knx.uartReset();
  }
  knx.addListenGroupAddress(KNX_GA_CO2);     //Gruppenadressen registrieren
  knx.addListenGroupAddress(KNX_GA_TVOC);
  knx.addListenGroupAddress(KNX_GA_TEMP);
  knx.addListenGroupAddress(KNX_GA_HUM);
}

void serialEvent() { //respond to Knx
  KnxTpUartSerialEventType eType = knx.serialEvent();
  if (eType == KNX_TELEGRAM) {
    KnxTelegram* telegram = knx.getReceivedTelegram();

    String target =
      String(0 + telegram->getTargetMainGroup())   + "/" +
      String(0 + telegram->getTargetMiddleGroup()) + "/" +
      String(0 + telegram->getTargetSubGroup());

    if (telegram->getCommand() == KNX_COMMAND_READ) {
     readT=true; 
     if (strcmp(target.c_str(),KNX_GA_TEMP)==0) {
          knx.groupAnswer2ByteFloat(KNX_GA_TEMP, temp);
          //result = knx.groupWrite2ByteFloat(KNX_GA_TEMP, temp);
          result = knx.groupWrite2ByteFloat(KNX_GA_HUM, hum); 
          result = knx.groupWrite2ByteInt(KNX_GA_CO2, airQuality);
          result = knx.groupWrite2ByteInt(KNX_GA_TVOC, airTvoc); 
     } 
     readT=false; 
    }
  }  
}  

void readiAQ() {
  int iAQRes;
  Wire.requestFrom(iaqaddress, 9);

  airQuality = (Wire.read()<< 8 | Wire.read()); 
  iAQstatus = Wire.read();
  iAQRes = (Wire.read()& 0x00)| (Wire.read()<<8)| (Wire.read()<<8| Wire.read());
  airTvoc = (Wire.read()<<8 | Wire.read());
}

void ambientlight_control() { 
  ambi_mapped=map(ambientlight,0,1023,map_TFT_min,map_TFT_max);   //Helligkeitswert mappen 
  analogWrite(PWM_BL_Pin, ambi_mapped);                           //Helligkeit nach Mapping regeln                              
  ambi_str=float2str(ambientlight,0);                  //Helligkeitswert in String umwandeln
  if (LEDs_Anz) {                                      //Wenn LED-Anzeigen aktiv
    if (CO2LimitLow()) {
      LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);
      analogWrite(LED_G, LED_mapped);
      analogWrite(LED_R, 255);
    }   //Wenn unter Grenzwert -> Grün anzeigen
    if (CO2LimitMiddle()) {
      LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);
      analogWrite(LED_G, LED_mapped);
      LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);
      analogWrite(LED_R, LED_mapped);
    }   //Wenn Grenzwert 1 erreicht Grün und Rot auf Gelb mischen 
    if (CO2LimitHigh()) {
      LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);
      analogWrite(LED_R, LED_mapped);
      analogWrite(LED_G, 255);
    }    //Wenn Grenzwert 2 erreicht Rot anzeigen
  }
  if (LEDB_Anz) {   //Wenn blaue LED aktiv 
    if (hum>65) {
      LED_mapped=map(ambientlight,0,1023,255,50);
      analogWrite(LED_B, LED_mapped);
    } else {
      analogWrite(LED_B, 255);
    }    //Blaue LED bei Feuchtewert >65% anzeigen
  } 
}

String float2str(float f, int n) {  //Funktion zur Umwandlung der Messwerte in Strings
  String fs="12345.67890";
  int i, k, st=0;
  float teiler;
  double d,e;
  char c;

  fs = "";
  d = f;
  for(i=10000; i >0; i/=10) {// 10000er 1000er 100er 10er 1er
    k=(int)d/i; 
    if((k>0) | st) {
      st=1;
      c=k+48; // ASCII
      fs.concat(c);
    }
    d = d - (k*i);
  }
  if(st==0) fs.concat("0"); // wenigstens 0 ausgeben
  if(n>0) fs.concat("."); // Dezimalpunkt
  teiler = 0.1;
  for(i=0; i< n; i++) {
    e = d / teiler; // 0.1er 0.01er 0.001er 0.0001er
    k=(int)e;
    c=k+48; // ASCII
    fs.concat(c);
    d = d - (k*teiler);
    teiler = teiler / 10.0;
  }
  return fs;
} 



void temp_send_show() {
  if (minTemp==0) minTemp=temp;   //MinTemp setzen  
  oldTemp = temp;  //Temperatur merken
  temp_str=float2str(temp,1);    //Temperatur in String umwandeln
  result = knx.groupWrite2ByteFloat(KNX_GA_TEMP, temp);    //Temperatur auf den KNX-Bus senden
  //showSection(Sektion Nr, Messwertart, Var, Vordergrundfarbe, Hintergrundfarbe, Schriftgröße, Einheit, Schriftgröße Einheit, Einheiten 1/2, Einheit Zeile 2); 
  showSection(1,mClass[1],temp_str,ColorSecMain[1][0],BackgSecMain[1],FontHMess[1][0],centi,FontHEinh[1],1,empty); 
                    
  if (Taup_Anz) {                                                                                                   //Taupunktberechnung schnelle Methode
     float a1 = 17.271;
     float b1 = 237.7;
     float temp1 = (a1 * temp) / (b1 + temp) + log(hum/100);
     tp = (b1 * temp1) / (a1 - temp1);
     if (tp<0) tp=0;
     Tau_str=float2str(tp,1);   
  }
}

void hum_send_show() {
  if (hum>99) hum=99;    //Feuchte auf 99% begenzen
  oldHum = hum;          //Feuchte merken
  hum_str=float2str(hum,0);   //Feuchtewert in String umwandeln
  result = knx.groupWrite2ByteFloat(KNX_GA_HUM, hum); //Feuchte auf den KNX-Bus senden
  showSection(2,mClass[2],hum_str,ColorSecMain[2][0],BackgSecMain[2],FontHMess[2][0],perc,FontHEinh[2],1,empty);//Sektion schreiben
}  

void iAQ_send_show() {
  byte i=0;
  readiAQ(); //iAQ: VOC und CO2 lesen
  
  if (iAQstatus != oldiAQstatus) {    //Hat sich der Status des iAQ geändert
       oldiAQstatus = iAQstatus;      //iAQ Status nerken
       result = knx.groupWrite1ByteInt(KNX_GA_STATUS, iAQstatus);
  }
  if (airQuality >= (oldairQuality + 50) || airQuality <= (oldairQuality - 50)) {     //Bei Änderung des CO2-Wertes um mehr oder gleich 50ppm  
       oldairQuality = airQuality;                                   //CO2-Wert merken  
       CO2_str=String(airQuality);                                  //CO2-Wert in String umwandeln
       result = knx.groupWrite2ByteInt(KNX_GA_CO2, airQuality);     //Co2-Wert auf KNX-Bus senden
       if (CO2LimitLow()) i=0;                                       //CO2-Wert unter Grenzwert     
       if (CO2LimitMiddle()) i=1;                                    //CO2-Wert Grenzwert 1 überschritten 
       if (CO2LimitHigh()) i=2;                                     //CO2-Wert Grenzwert 2 überschritten 
     //showSection(Sektion Nr, Variable String, Vordergrundfarbe, Hintergrundfarbe, Schriftgröße, Einheit, Schriftgröße Einheit, Einheiten 1 oder 2, 2.Einheit);
       showSection(3,mClass[3],CO2_str,ColorSecMain[3][i],BackgSecMain[3],FontHMess[3][0],CO2,FontHEinh[3],2,ppm);     //Sektion 3 schreiben
  }
  if (airTvoc >= (oldairTvoc + 30) || airTvoc <= (oldairTvoc - 30)) {   //Bei Änderung des VOC-Wertes um mehr oder gleich 30ppb
       oldairTvoc = airTvoc;         //VOC-Wert merken
       VOC_str=String(airTvoc);     //VOC-Wert in String umwandeln
       result = knx.groupWrite2ByteInt(KNX_GA_TVOC, airTvoc);    //VOC-Wert auf KNX-Bus senden  
       if (VOCLimitLow()) i=0;        //VOC-Wert unter Grenzwert     
       if (VOCLimitMiddle()) i=1;     //VOC-Wert Grenzwert 1 überschritten 
       if (VOCLimitHigh()) i=2;       //VOC-Wert Grenzwert 2 überschritten
       showSection(4,mClass[4],VOC_str,ColorSecMain[4][i],BackgSecMain[4],FontHMess[4][0],VOC,FontHEinh[4],2,ppb);     //Sektion 4 schreiben                                      
  }
  if (LEDs_Anz) {
    ambientlight_control();
  } 
}

bool CO2LimitLow() {
  if (airQuality<=GrenzW1CO2) {return true;}else{return false;}
}
bool CO2LimitMiddle() {
  if ((airQuality > GrenzW1CO2) && (airQuality < GrenzW2CO2)) {return true;}else{return false;}
}
bool CO2LimitHigh() {
  if (airQuality >= GrenzW2CO2) {return true;}else{return false;}
}
bool VOCLimitLow() {
  if (airTvoc<=GrenzW1VOC) {return true;}else{return false;}
}
bool VOCLimitMiddle() {
  if ((airTvoc > GrenzW1VOC) && (airTvoc < GrenzW2VOC)) {return true;}else{return false;}
}
bool VOCLimitHigh() {
  if (airTvoc >= GrenzW2VOC) {return true;}else{return false;}
}
