//***********************************************************************
//**************************  SMELLY ONE  *******************************
//***********************************************************************
//Hardwaredesign: dreamy1 /// Softwaredesign: Wagoklemme
//Aktuelle Hardware-Revision: V1.4
//Aktuelle Software-Revision: Vx.x
//***********************************************************************
//Hardware-Changelog:
//V1.4: SW2 kann auch als Taster bestückt werden, Summer dann nicht verwendbar. Kleine Designänderungen. 
//V1.3: CS TFT und RGB LED Rot getauscht, jetzt auch Farbe Rot dimmbar. Kleine Designänderungen.
//V1.2: Designänderungen
//V1.1: Thermische Entkopplung SHT31 verbessert, Designänderungen
//V1.0: Initial Release

//Software-Changelog:
//V1.3: Offset für Temperaturänderung und Feuchte hinzugefügt. Bug bei Temperaturänderung beseitigt.
//V1.2: Taupunktanzeige geändert
//V1.1: RTR hinzugefügt
//V1.0: Initial Release
//***********************************************************************
//Nachfolgender Code ist nur für Arduino Pro Mini 3,3V 8MHz
//Sensoren: SHT31 für Temperatur/Feuchte, AMS Core P für VOC/CO2, TEPT4400 für Helligkeit
// ACHTUNG: immer nur eine Stromversorgung anschließen - entweder BTI mit gestecktem linken Jumper oder FTDI!
// Smelly One kann durch Entfernen des Jumpers "Power" vom Bus stromlos gemacht werden, ohne diesen vom BTI abstecken zu müssen. 
// Vor dem Anschließen des FTDI immer den linken Jumper "Power" entfernen, ansonsten besteht die Gefahr dass entweder der Smelly oder der 
// Siemens irreparabel defekt ist (da zwei 3,3V-Versorgungen aktiv)! 
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

//USER Setup                                                      //KNX Messwerte/Betriebswerte GAs müssen auf die Gegenheiten angepasst werden
#define KNX_PA               "1.0.180"                            //KNX PA
#define KNX_GA_STATUS        "6/3/8"                              //Status IAQ
#define KNX_GA_CO2           "4/0/100"                            //CO2-Messwert
#define KNX_GA_TVOC          "4/0/101"                            //VOC-Messwert
#define KNX_GA_TEMP          "3/1/8"                              //Temperaturmesswert
#define KNX_GA_HUM           "4/0/8"                              //Feuchtemesswert
#define KNX_GA_VALVE         "3/3/80"                             //Ventilstellung
#define KNX_GA_SOLL          "3/2/8"                              //Sollwert
#define KNX_GA_BA            "3/4/80"                             //Betriebsart

#if (RAMEND < 1000)                                               //Serial Buffer bei genügend RAM vergrößern
    #define SERIAL_BUFFER_SIZE 64
#else
    #define SERIAL_BUFFER_SIZE 256
#endif

bool allow_MinMax=true;                                          //MinMax Fenster mit SW1 erlauben
bool allow_RTRWin=false;                                           //RTR Fenster mit SW1 erlauben. ACHTUNG: Wenn RTR gewählt, dann MUSS alles Andere wie Taup_Anz, Lux_Anz, iAQ-Status, MinMax auf false gesetzt sein.
                                                                  //Weitere Einstellungen bei RTR-Betrieb: RTR=true; P_Glied,I_Glied,D_Glied, Sollwerte vorgeben,Ti,Kp  Optionen sind:offsetBus,y_min,y_max,y
                                                                  //Kp je höher der Wert desto größer wird die Sprungantwort der Stellgröße und schneller wird der Regler
                                                                  //Ti je höher der Wert desto langsamer ist die Anstiegsgeschwindigkeit. Bei kleiner Ti neigt die Regelung zum Schwingen, bei großer Ti wird die Regelung träge.  ACHTUNG: Der Regelalgorithmus wird auch in diesem Intervall aufgerufen. D.h. z.B. bei 900s, wird der Regelalgorithmus nur in 15min Abständen aktualisiert.
                                                                  //Beide Werte müssen für die eigene Regelstrecke herausgefunden werden. Nähere Infos hier: https://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=0ahUKEwjpz5vQsMPYAhUIPRQKHWHkALoQFgguMAA&url=https%3A%2F%2Fwww.samson.de%2Fdocument%2Fl102de.pdf&usg=AOvVaw233z8eQxkRXudJkn_myFXG
bool piep_allow=false;                                            //Piepser bei Grenzwertüberschreitung zulassen (Jumper setzen !)
bool Taup_Anz=true;                                              //Taupunkt in Sektion 5 dauerhaft anzeigen
bool Lux_Anz=false;                                               //Helligkeitswert anzeigen, Wert wird reziprog angezeigt 
bool StatusiAQ_Anz=false;                                         //Status iAQ anzeigen 
bool LEDs_Anz=false;                                              //RGB LED anzeigen für Luftqualität CO2 nach Grenzwerten
bool LEDB_Anz=false;                                              //Blaue LED anzeigen für Feuchtigkeit > 65%
bool KNX_receive=true;                                            //KNX Empfang einschalten
bool RTR=false;                                                    //RTR freigeben
bool P_Glied=true;                                                //P-Glied einschalten 
bool I_Glied=true;                                                //I-Glied einschalten 
bool D_Glied=false;                                               //D-Glied einschalten
bool Windup=false;                                                //Anti-Windup einschalten
float SW_Eco=17, SW_Komf=21, SW_Frost=7;                          //Sollwerte festlegen
unsigned long Ti=2;                                               //Integrierzeit in s 
byte offsetBus=2;                                                 //Offset, ab welcher Wertänderung die Ventilstellung auf den Bus gesendet wird
float Kp=5;                                                       //Proportionalbeiwert für P-Glied
byte y=0;                                                         //Arbeitspunkt/mind. Ventilstellung
byte y_min=0;                                                     //minimale Stellgröße
byte y_max=42;                                                    //maximale Stellgröße  
float temp_send_offset=0.2;                                       //Temperaturänderung ab wann ein neuer Wert geschrieben werden soll
float hum_send_offset=1;                                          //Temperaturänderung ab wann ein neuer Wert geschrieben werden soll
float offsetTemp=1.8;                                             //Temperatursensor Offset
float offsetHum=0;                                                //Feuchtesensor Offset
byte map_LED_min=240;                                             //LED minimale Helligkeit
byte map_LED_max=50;                                              //LED maximale Helligkeit
byte map_TFT_min=250;                                             //TFT minimale Helligkeit
byte map_TFT_max=0;                                               //TFT maximale Helligkeit       
int brightness=900;                                               //erstmal feste Helligkeit des Backlight
unsigned long intervaliAQ =                 11500;                //Aktualisierungsintervall iAQ in ms
unsigned long intervalTempHum =              5000;                //Aktualisierungsintervall SHT31 in ms
unsigned long intervalMinMax =              20000;                //MinMax Fenster nach Zeit schliessen in ms
unsigned long intervalambi =                 2000;                //Intervall Backlightregelung und LEDs
String mClass[7]={"TxT","C","H","CO2","VOC","TAU","LUX"};         //Messwertart in der jeweiligen Sektion 
int GrenzW1CO2=1000; int GrenzW2CO2=2000;                         //Grenzwerte CO2
int GrenzW1VOC=410; int GrenzW2VOC=830;                           //Grenzwerte VOC
int GrenzWHell=0;                                                 //Grenzwert Helligkeit für Piepser
String s_alarm = "LUEFTEN";                                       //Meldetext max. 10 Zeichen bei Größe 2
// Farben einstellen für                  Sektion 1       Sektion 2              Sektion 3                      Sektion 4            GW2       Sektion 5
unsigned int ColorSecMain[6][3]={{0,0,0},{TFT_WHITE,0,0},{TFT_WHITE,TFT_BLUE,0},{TFT_GREEN,TFT_YELLOW,TFT_RED},{TFT_GREEN,TFT_YELLOW,TFT_RED},{TFT_RED,TFT_CYAN,TFT_GREEN}};
unsigned int BackgSecMain[6]={0,TFT_BLACK,TFT_BLACK,TFT_BLACK,TFT_BLACK,TFT_BLACK};                                         //Hintergrundfarbe Messwertfenster
byte FontHMess[6][2]={{},{3,0},{3,0},{2,0},{2,0},{2,2}};                                                                    //Fontgröße Messwerte
byte FontHEinh[6]={0,1,1,1,1,1};                                                                                            //Fontgröße Einheiten
unsigned int ColorSecMinMax[6][3]={{0,0,0},{TFT_WHITE,0,0},{TFT_WHITE,0,0},{TFT_WHITE,0,0},{TFT_WHITE,0,0},{TFT_BLUE,0,0}}; //Vordergrundfarbe MinMax-Fenster
unsigned int BackgSecMinMax[6]={0,TFT_BLACK,TFT_BLACK,TFT_BLACK,TFT_BLACK,TFT_BLACK};                                       //Hintergrundfarbe MinMax-Fenster
byte FontHMinMax[6]={0,2,2,2,2,2};                                                                                          //Fontgröße Messwerte MinMax-Fenster  
byte FontHEinhMinMax[6]={0,1,1,1,1,0};                                                                                      //Fontgröße Einheiten MinMax-Fenster
//eingebundene Bibliotheken
#include <Adafruit_SHT31.h>
//KNX
#include <KnxTpUart.h>
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
bool dispRenew,dispRenew_MinMax,RTRWin,dispRenew_RTR,result,alarm,MinMax, piep, readCO2, readVOC, readT, readH, MinMax_active, SW1_pressed, MM_closed,RTRWin_active = false; 
char m[4],ppm[4]="ppm",ppb[4]="ppb", CO2[4]="CO2", VOC[4]="VOC", empty[1]="", Taup[4]="TAU", Min[4]="Min", Max[4]="Max", LUX[3]="lx", Stat[4]="iAQ", PA[3]="PA", Ven[4]="VEN";
char centi[5]="\367C"; char perc[2]="%";
unsigned long prevMillisSHT, prevMillisiAQ, prevMillisiAQAnz, prevMillisPiep,  prevMillisambi, prevMillisRTR,last = 0; 
byte secYPos[6][2]={{},{32,48}, {60,75}, {87,100}, {111,123}, {134,148}};                    //{1,] yPos Fläche, [,1] yPos Messwert
bool secRenew[6]={false,false,false,false,false,false};
bool secTxtRenew[7]={false,false,false,false,false,false,false};
byte oldlenFl[7]={0,0,0,0,0,0,0};
byte h,Sec5_Anz,Valve,oldSec5_Anz,iAQstatus,oldiAQstatus,oldiAQstatus2,BA,oldBA=0;
String ambi_str,temp_str,hum_str,StatiAQ_str,CO2_str,VOC_str,Tau_str,minTemp_str,maxTemp_str,minHum_str,maxHum_str,Valve_str,BA_str,Soll_str,Sek5BA_str="";
byte buttonState;                   //Speichert den aktuellen HIGH/LOW Status des Pins
byte buttonChange;                  //Speichert Flankenwechsel des Pins
unsigned long buttonDownTime;       //Speichert die Zeit, wenn ein Button gedrückt wird
enum{UNCHANGED,BUTTONUP,BUTTONDOWN};//Mögliche Zustände der Buttons
//RTR
float d, e_sum, e_alt=0, Sollwert=0, Sollwert_alt=0, Ki, Kd=0;
byte yP, yI, yD, y_res, oldy_res, oldy;
unsigned long Td;

void hum_send_show();  
void iAQ_send_show(); 
bool CO2LimitLow();
bool CO2LimitMiddle();
bool CO2LimitHigh();
bool VOCLimitLow();
bool VOCLimitMiddle();
bool VOCLimitHigh();
String float2str(float f, int n);
//////////////////////////////////Setup
void setup() {
  setKNX();                                           //KNX Init 
  setHW();                                            //Hardware Init 
  oldiAQstatus=255;                                   //iAQStatus setzen, damit der erste Wert gesetzt wird
//  readiAQ;                                            //iAQ lesen   
  pinMode(PWM_BL_Pin, OUTPUT);                        //PWM Backlight als Output setzen 
  analogWrite(PWM_BL_Pin, brightness);                //Backlight erstmal aus lassen
  tft.begin();                                        //TFT aktivieren
  sht31.begin(0x44);                                  //SHT31 aktivieren
  initDisplay();                                      //Display initialisieren 
  BA=3;BA_str="ECO";Sollwert=SW_Eco;Soll_str=float2str(SW_Eco,1);
  oldtp=0;
}


////////////////////////////////////////////Loop
void loop(){
  static char outstr[4];

if (!readT){
 
    But_pressed();
    if ((SW1_pressed)&&(allow_MinMax)) {MinMax=true;dispRenew_MinMax=false;}                        //Wenn SW1 gedrückt
    if ((SW1_pressed)&&(allow_RTRWin)&&(BA!=5)&&(BA!=6)) {RTRWin=true;dispRenew_RTR=false;}                           //Wenn SW1 gedrückt
    if (MinMax) { 
         unsigned long now=millis(); 
           if ((SW1_pressed)&&(!MinMax_active)) {
              MinMax_Window_Open();                                                                 //MinMax-Fenster öffnen
           } 
           if ((SW1_pressed)&&(now-buttonDownTime>=5000)) {                                         //prüfen ob SW1 mindestens 5000ms unten ist
              minTemp=temp; maxTemp=temp; minHum=hum; maxHum=hum;                                   //MinMax-Werte zurücksetzen
              minTemp_str=float2str(minTemp,1);                                                     //MinTemp in String umwandeln
              maxTemp_str=float2str(maxTemp,1); 
              minHum_str=float2str(minHum,1);                                                       //MinFeuchte in String umwandeln
              maxHum_str=float2str(maxHum,1);
              MinMax_Window_Close(); 
           }   
        if ((!SW1_pressed)&&(MinMax_active)&&(millis() - buttonDownTime > intervalMinMax)) {        //MinMax-Fenster nach vorgegebener Zeit schliessen
            MinMax_Window_Close();                                                                  //MinMax-Fenster schliessen
        }    
    }                 
    if (RTRWin) { 
        if ((SW1_pressed)&&(!RTRWin_active)) {                                                       //RTR Fenster öffnen 
              oldBA=BA; Sollwert_alt=Sollwert;                                                       //Werte merken
              RTR_Window_Open();                                                                     //RTR-Fenster öffnen
              SW1_pressed=false;                                                                     //Tastendruck abfangen
        } 
        if ((SW1_pressed)&&(RTRWin_active)) {                                                        //Tastendruck Zeitmessung
              last = millis() - buttonDownTime;
        }
        if ((!SW1_pressed)&&(RTRWin_active)&&(last>0)&&(last<300)) {                                 //Sollwert inkrementieren
               switch (BA) {                                                                                                       
               case 1: Sollwert=Sollwert+0.5;
                       if (Sollwert>26) {Sollwert=19;}
                       break;
               case 3: Sollwert=Sollwert+0.5;
                       if (Sollwert>20) {Sollwert=10;}
                       break;
               case 4: Sollwert=Sollwert+0.5;
                       if (Sollwert>15) {Sollwert=6;}
                       break;               
               }
               Soll_str=float2str(Sollwert,1);
               tft.fillRect(0,58,128,26,BackgSecMain[2]);                                             //Zeile Sollwert löschen
               showSection(2,mClass[2],Soll_str,ColorSecMinMax[2][0],BackgSecMinMax[2],FontHMinMax[2],centi,FontHEinhMinMax[2],1,empty);    //Sektion 2 schreiben
               last=0;
        }
        if ((SW1_pressed)&&(RTRWin_active)&&(millis() - buttonDownTime>3000)&&(millis() - buttonDownTime<4000)) {                           //prüfen ob SW1 mindestens 3000ms unten ist
               switch (BA) {                                                                                                                //BA umschalten
               case 1: BA=3;BA_str="ECO"; Soll_str=float2str(SW_Eco,1);Sollwert=SW_Eco;
                      break;
               case 3: BA=4;BA_str="FROST"; Soll_str=float2str(SW_Frost,1);Sollwert=SW_Frost;
                      break;
               case 4: BA=1;BA_str="KOMFORT"; Soll_str=float2str(SW_Komf,1); Sollwert=SW_Komf;             
                      break;
               }
               tft.fillRect(0,32,128,52,BackgSecMain[1]);
               showSection(1,mClass[1],BA_str,ColorSecMinMax[1][0],BackgSecMinMax[1],FontHMinMax[1],empty,FontHEinhMinMax[1],1,empty);      //Sektion 1 schreiben          
               showSection(2,mClass[2],Soll_str,ColorSecMinMax[2][0],BackgSecMinMax[2],FontHMinMax[2],empty,FontHEinhMinMax[2],1,empty);    //Sektion 2 schreiben
               SW1_pressed=false;
           }   
        if ((!SW1_pressed)&&(RTRWin_active)&&(millis() - buttonDownTime > intervaliAQ)) {        //RTR-Fenster nach vorgegebener Zeit schliessen
            RTR_Window_Close();                                                                  //RTR-Fenster schliessen
            if (BA!=oldBA) {                                                                     //Wenn BA geändert, BA auf den Bus schreiben
                  result = knx.groupWrite1ByteInt(KNX_GA_BA,BA);
            }
            if (Sollwert!=Sollwert_alt) {                                                        //Wenn Sollwert geändert, Sollwert auf den Bus schreiben
                  result = knx.groupWrite2ByteFloat(KNX_GA_SOLL, Sollwert);                     
            }
        }    
 
    }                 

//RTR

if ((RTR)&&(Sollwert>temp)) {                                                                     //Regelung notwendig ?
    if (Sollwert!=Sollwert_alt) {
      Sollwert_alt=Sollwert;
      e_sum=0;    
      e_alt=Sollwert-temp; 
     }  
    else if ((millis() - prevMillisRTR > (Ti*1000))&&(! MinMax)) {                                //Regelalgorithmus starten   
          prevMillisRTR = millis(); 
          d=Sollwert-temp;                       

           // P-Glied:
          yP = (Kp * d);                      
          if (Kp==0) { Kp = 1; }
          // I-Glied:
          Ki = Kp / Ti;                              
          e_sum = e_sum + d;     
          if (!I_Glied) Ki=0;
          else if (Windup) {
              if (e_sum>(y_max/Ki*Ti)) {e_sum=y_max/(Ki*Ti);} 
              if (e_sum<(y_min/Ki*Ti)) {e_sum=y_min/(Ki*Ti);}                
          }
          yI = Ki * Ti * e_sum;                  
          // D-Glied:
          Td = Ti;                        
          Kd = Kp * Td;                    
          yD = Kd * (d-e_alt) / Td;              
          e_alt=d;                 

          // Gesamt-Stellgröße
          if (!P_Glied) yP=0;            // P-Anteil nullen
          if (!I_Glied) yI=0;            // I-Anteil nullen
          if (!D_Glied) yD=0;            // D-Anteil nullen
          y_res = yP + yI + yD+y;          // Stellgröße (y) = P-Anteil + I-Anteil + D-Anteil + Arbeitspunkt y
          if (y_res>y_max) y_res=y_max;    // Stellgröße begrenzen: max
          if (y_res<y_min) y_res=y_min;    // Stellgröße begrenzen: min
          Valve_str=float2str(y_res,0);
          if (y_res >= (oldy_res + offsetBus) || y_res < (oldy_res - offsetBus)) {
              oldy_res = y_res;                                                                     //Stellgröße Wert nerken
              result = knx.groupWrite1ByteInt(KNX_GA_VALVE, (y_res*2.55));
          }
    }
}          
else if (((BA==1)||(BA==3)||(BA==4))&&(y_res!=0)&&(temp>=Sollwert)&&(RTR)) {y_res=0; Valve_str=float2str(y_res,0); result = knx.groupWrite1ByteInt(KNX_GA_VALVE, y_res);} //Stellgröße 0 setzen, wenn keine Regelung
                     
          
if ((!MinMax_active)&&(!RTRWin_active)) {                                                                                               //Wenn nicht MinMax-Fenster oder RTR-Fenster aktiv
    //Temperatur und Luftfeuchte 
    if (millis() - prevMillisSHT > intervalTempHum) {                                                                                   //Ausleseinterval überprüfen
              prevMillisSHT = millis();                                                                                                 //aktuelle Zeit abspeichern
              temp = sht31.readTemperature()-offsetTemp ;                                                                               //Temperatur lesen und Offset berücksichtigen
              if (temp >= (oldTemp + temp_send_offset) || temp <= (oldTemp - temp_send_offset)) {                                                   //Temperatur: Wenn die Änderung mehr oder gleich 0.2°C beträgt
                  temp_send_show();
              }
              Serial.println(String(" ")+ " " + String(" ")+ " " + outstr + String(234)+ " " + String(123)+ "/r"); 
              hum = sht31.readHumidity()-offsetHum;                                                                                     //Feuchte lesen und Offset berücksichtigen
              if (hum >= (oldHum + hum_send_offset) || hum <= (oldHum - hum_send_offset)) {                                                                         //Feuchte: Wenn die Änderung mehr oder gleich 1% beträgt
                  hum_send_show();
              }         
    }    
    //iAQ auslesen: 11.5 Sekunden warten (der VOC Core P liefert nur alle 11s neue Werte)       
    if (millis() - prevMillisiAQ > intervaliAQ) {                                                                                       //Ausleseinterval überprüfen
            prevMillisiAQ = millis();                                                                                                   //aktuelle Zeit abspeichern
            iAQ_send_show();    
    }
                                                                                                                      
         if (millis() - prevMillisambi > intervalambi) {                                                                                //Ausleseinterval überprüfen
            prevMillisambi = millis();                           
            ambientlight = analogRead(A0)*0.65;                                                                                         //Helligkeitswert einlesen
            if (ambientlight >= (oldambientlight)+2 || ambientlight < (oldambientlight)-2) {                                            //Bei Helligkeitswertänderung von mehr oder gleich 2 Lux
                oldambientlight=ambientlight;                                                                                           //Helligkeitswert merken
                ambientlight_control();
            }
            if ((piep_allow)&&(digitalRead(SW2)== LOW)&&(ambientlight>GrenzWHell)) {                                                    //Piepser Freigabe mit SW2 geschlossen und freigegeben im Usersetup und Helligkeit
                if (airQuality >= GrenzW2CO2) {
                   piep_control();  
                }else if (airQuality < GrenzW2CO2) {
                   piep=true;
                }
            }    
         }                                                                                                                                       

    
          if ((airTvoc >= GrenzW2VOC)||(airQuality >= GrenzW2CO2)) {alarm=true; Sec5_Anz=1; }                                           //Bei Grenzwertüberschreitung Meldetext anzeigen    
          if ((airTvoc < GrenzW2VOC)&&(airQuality < GrenzW2CO2)&&(alarm)) Sec5_Anz=2;                                                   //Alarm rücksetzen nach Grenzwertüberschreitungen                                                
          if ((RTR)&&(!alarm)&&(! MinMax)) Sec5_Anz=6;                                                                                  //Bei RTR-Betrieb Anzeige aktualiesieren
          if ((StatusiAQ_Anz)&&(!alarm)&&(! MinMax)) Sec5_Anz=5;                                                                        //iAQ Status Anzeige freigegeben ?
          if ((Lux_Anz)&&(!alarm)&&(! MinMax)) Sec5_Anz=4;                                                                              //Helligkeitswert Anzeige freigegeben ?
          if ((Taup_Anz)&&(!alarm)&&(! MinMax)) Sec5_Anz=3;                                                                             //Taupunktanzeige freigegeben ?  
          if (Sec5_Anz!=oldSec5_Anz) {oldSec5_Anz=Sec5_Anz; memset(secTxtRenew,0,sizeof(secTxtRenew));}                                 //Falls sich die Anzeige für Sektion 5 geändert, Text/Feld neu zeichen
            
              switch (Sec5_Anz) {                                                                                                       //Abfrage welche Anzeige Sektion 5
              case 1: if (!secTxtRenew[1]) {secRenew[5]=false;                                                                          //Alarm entstanden
                              if (alarm) {
                                  showSection(5,mClass[0],s_alarm,ColorSecMain[3][2],BackgSecMain[5],FontHMess[5][0],empty,FontHEinh[5],2,empty);//Sektion 5 mit Alarm beschreiben
                                  secTxtRenew[1]=true;                                                                                  //Text neu zeichnen
                              }
                          }
                      break;
              case 2: if (! secTxtRenew[2]) {                                                                                           //Feld leeren  
                          tft.fillRect(0,133,128,26,BackgSecMain[5]);
                          secTxtRenew[2]=true;                                                                                          //beschrieben setzen
                          alarm=false;
                          if (RTR) {oldy=0;Sec5_Anz=6;}
                      }
                      break;        
              case 3: if (!secTxtRenew[3]) secRenew[5]=false;                                                                           //Taupunktanzeige freigegebwn
                      if ((tp!=oldtp)&&(tp>0)&&(Taup_Anz)) {                 
                          tft.fillRect(0,133,128,26,BackgSecMain[5]);                                                                          
                          showSection(5,mClass[5],Tau_str,ColorSecMain[5][1],BackgSecMain[5],FontHMess[5][1],centi,FontHEinh[5],2,Taup);//Sektion 5 mit Taupunkt beschreiben
                          secTxtRenew[3]=true;                                                                                          //beschrieben setzen
                          oldtp=tp;
                      }
                      break;
              case 4: if (!secTxtRenew[4]) secRenew[5]=false;                                                                           //Helligkeitsanzeige freigegeben
                      if ((ambientlight >= oldambientlight2+10) || (ambientlight < oldambientlight2-10)||(MM_closed && Lux_Anz)) {      //Helligkeit im Intervall anzeigen
                          tft.fillRect(0,133,128,26,BackgSecMain[5]); 
                          showSection(5,mClass[6],ambi_str,ColorSecMain[5][1],BackgSecMain[5],FontHMess[5][0],LUX,FontHEinh[5],1,empty);//Sektion 5 mit Helligkeitswert beschreiben
                          secTxtRenew[4]=true;                                                                                          //beschrieben setzen
                          oldambientlight2=ambientlight;
                          MM_closed=false;
                      }
                      break;
              case 5: if (!secTxtRenew[5]) secRenew[5]=false;                                                                           //iAQ Status Anzeige freigegeben
                      if ((iAQstatus!=oldiAQstatus2)||(MM_closed && StatusiAQ_Anz)) {                                                   //iAQ Status bei Änderung refreshen
                          tft.fillRect(0,133,128,26,BackgSecMain[5]);                                                                   //Feld leeren
                          switch (iAQstatus) {                                                                                          //iAQ Status auf Text mappen
                            case 0: StatiAQ_str="Ok";break;
                            case 1: StatiAQ_str="Busy";break;
                            case 16: StatiAQ_str="Warmup"; break;
                            case 128: StatiAQ_str="Error"; break;
                          } 
                          secTxtRenew[5]=true;                                                                                          //beschrieben setzen
                          oldiAQstatus2 = iAQstatus; 
                          showSection(5,mClass[0],StatiAQ_str,ColorSecMain[5][2],BackgSecMain[5],FontHMess[5][0],Stat,FontHEinh[5],1,empty); //Sektion 5 mit iAQ Status beschreiben
                          MM_closed=false;
                      }
                      break;
              case 6: if (!secTxtRenew[6]) secRenew[5]=false;                                                                           //Stellungsanzeige und Betriebsartanzeige freigegeben
                      if (RTR) {
                        if ((oldBA!=BA)||(y_res!=oldy)) {                                                                               //nur bei Änderung aktualisieren
                          tft.fillRect(0,133,128,26,BackgSecMain[5]); 
                          if (BA==1) {Sek5BA_str="Komf "+Valve_str;}
                          if (BA==3) {Sek5BA_str="Eco "+Valve_str;}
                          if (BA==4) {Sek5BA_str="Frost";}
                          if (BA==5) {Sek5BA_str="Restw";}
                          if (BA==6) {Sek5BA_str="Fenst";}
                          showSection(5,mClass[5],Sek5BA_str,ColorSecMain[5][1],BackgSecMain[5],FontHMess[5][1],perc,FontHEinh[5],2,Ven);//Sektion 5 mit Taupunkt beschreiben
                          secTxtRenew[6]=true;                                                                                          //beschrieben setzen
                        }    
                        oldBA=BA;
                        oldy=y_res;
                      }
                      break;                              
              case 7: break;                      
              }   
    } 
}
}
/////////////////////////////////////////////Routinen
void initDisplay() {
  //Achtung: durch einen Fehler in der LIB ist der Nullpunkt für X/Y bei 0/32).
  if (! dispRenew) {                                                                                                                     //Wenn TFT initialisieren nicht erfolgt, auf einmalig setzen
    tft.fillScreen(TFT_BLACK);                                                                                                           //Schwarz füllen 
    tft.setRotation(0);                                                                                                                  //hochkant 
    dispRenew=true;                                                                                                                      //Initialisierung erfolgt 
  } 
}  
void initDisplay_MinMax() {                                                                                                              //siehe Messwertfenster 
   if (! dispRenew_MinMax) {
    tft.fillScreen(TFT_BLACK);
    dispRenew_MinMax=true;
  } 
}  
void initDisplay_RTR() {                                                                                                                //siehe Messwertfenster 
   if (! dispRenew_RTR) {
    tft.fillScreen(TFT_BLACK);
    dispRenew_RTR=true;
  } 
}  
void showSection(byte sec_nr,String m,String s1,uint16_t color, uint16_t backg, byte t_size,char sym[], byte s_size, byte units, char sym2[]) { 
char tch[s1.length()+1];
byte lens1;
    if ((sec_nr==1) or (sec_nr==2)) h=28;                                                                                                //Für Sektion 1 und 2 Höhe von 28 Pixel festlegen 
    if ((sec_nr==3) or (sec_nr==4)) h=23;                                                                                                //Für Sektion 3 und 4 Höhe von 23 Pixel festlegen  
    if (sec_nr==5) h=26;                                                                                                                 //Für Sektion 5 Höhe von 25 festlegen
    if (m=="CO2") {                                                                                                                      //Wenn Messwert CO2  
        lens1=s1.length();                                                                                                               //Stringlänge merken
        if (lens1!=oldlenFl[3]) {                                                                                                        //Bei Stringlängenänderung Sektion beschreiben 
            secRenew[sec_nr]=false;
            oldlenFl[3]=lens1;
        }
    }        
    if (m=="VOC") {                                                                                                                      //Wenn Messwert VOC
          lens1=s1.length();                                                                                                             //Stringlänge merken
          if (lens1!=oldlenFl[4]) {                                                                                                      //Bei Stringlängenänderung Sektion beschreiben   
              secRenew[sec_nr]=false;
              oldlenFl[4]=lens1;
          } 
    }
    if (m=="LUX") {                                                                                                                      //Wenn Messwert Helligkeit   
          lens1=s1.length();                                                                                                             //Stringlänge merken 
          if (lens1!=oldlenFl[6]) {                                                                                                      //Bei Stringlängenänderung Sektion beschreiben 
              secRenew[sec_nr]=false;
              oldlenFl[6]=lens1;
          } 
    }
    if (! secRenew[sec_nr]) {                                                                                                            //Falls die Sektion neu beschrieben werden soll
       tft.fillRect(0,secYPos[sec_nr][0],128,h,backg);                                                                                   //Sektion leeren 
       secRenew[sec_nr]=true;                                                                                                            //beschrieben setzen   
    }  
    tft.setTextColor(color,backg);                                                                                                       //Textfarbe setzen 
    if (units==2) {s_size=1;}                                                                                                            //Bei 2 Einheiten Textgröße auf 1 fixieren
    tft.setTextSize(s_size);                                                                                                             //Textgröße setzen
    if (sec_nr==1) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);}                          //Sektion 1 nach Größe beschreiben 
                                   if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                   if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);} }
                    if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-13),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])+1),1);}}
    if (sec_nr==2) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);}                          //Sektion 2 nach Größe beschreiben  
                                   if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                   if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);}  } 
                    if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-13),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])+1),1);}}
    if (sec_nr==3) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);}                          //Sektion 3 nach Größe beschreiben 
                                   if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                   if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);}  }
                    if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-11),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-1),1);}}
    if (sec_nr==4) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-5), 1);}                          //Sektion 4 nach Größe beschreiben  
                                   if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-9), 1);} 
                                   if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-12), 1);}  }
                    if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-11),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-1),1);}}
    if (sec_nr==5) {if (units==1) {if (s_size==1) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-4), 1);}                          //Sektion 5 nach Größe beschreiben      
                                   if (s_size==2) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-8), 1);} 
                                   if (s_size==3) {tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-10), 1);} }
                    if (units==2) {tft.drawRightString(sym2, 125, ((secYPos[sec_nr][1])-11),1); tft.drawRightString(sym, 125, ((secYPos[sec_nr][1])-1),1);}}                    
   
    tft.setTextColor(color,backg);                                                                                                        
    tft.setTextSize(t_size);
    tft.setTextDatum(CC_DATUM);                                                                                         
    s1.toCharArray(tch, s1.length() + 1);                                                                                                //String in array of char umwandeln
    tft.setTextWrap(false);                                                                                                              //Textumbruch ausschalten
    if (t_size==1) {tft.drawCentreString(tch, 60, ((secYPos[sec_nr][1])-5), 1);}                                                         //Text nach horizontal zentrieren entsprechend der Texthöhe 
    if (t_size==2) {tft.drawCentreString(tch, 60, ((secYPos[sec_nr][1])-9), 1);} 
    if (t_size==3) {tft.drawCentreString(tch, 60, ((secYPos[sec_nr][1])-12), 1);}
    
}
 
void showSerial() {
  Serial.print("CO2:");
  Serial.print(airQuality);
  Serial.print(", TVoC:");
  Serial.println(airTvoc);
  Serial.println();
}

/////////////////////////////////////////////Initialisierung/Test HW
void setHW() {
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);                                                                                                                //TFT Elektronik ein
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);                                                                                                                //Piepser
  
  pinMode(A0, INPUT);                                                                                                                    //ADC für Helligkeitssensor aktivieren
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);                                                                                                                 //VOC einschalten
  delay(300);
  pinMode(7, OUTPUT); 
  digitalWrite(7, HIGH);                                                                                                                 //A0 am TFT high setzen
  pinMode(8, OUTPUT);                                                                                                                    //CS für TFT  
  pinMode(4, INPUT);
}
////////////////////////////////////////////////////////////////////KNX
void setKNX() {
  //initialize connection to KNX BCU
  Serial.begin(19200,SERIAL_8E1); // Even parity);
  while (!Serial) {
    ; // wait for serial port to connect
  }
//  UCSR0C = UCSR0C | B00100000;
// CTRLA = CTRLA | B00100000;

  // Reset UART
  if (Serial.available()) {
    knx.uartReset();
  }
  knx.addListenGroupAddress(KNX_GA_CO2);                                                                                 //Gruppenadressen registrieren
  knx.addListenGroupAddress(KNX_GA_TVOC);
  knx.addListenGroupAddress(KNX_GA_TEMP);
  knx.addListenGroupAddress(KNX_GA_HUM);
  if (KNX_receive) {knx.addListenGroupAddress(KNX_GA_VALVE); knx.addListenGroupAddress(KNX_GA_SOLL);
                    knx.addListenGroupAddress(KNX_GA_BA);}                                                            //Bei KNX-Receive
  }
///////////////////////////////////////////////////////KNX Bus Readanfrage beantworten
void serialEvent() {
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
          if (KNX_receive) {  
            result = knx.groupWrite2ByteFloat(KNX_GA_SOLL, Sollwert);
            result = knx.groupWrite1ByteInt(KNX_GA_VALVE, (y_res*2.55));           
            result = knx.groupWrite1ByteInt(KNX_GA_BA, BA);
          }  
     } 
//     if (strcmp(target.c_str(),KNX_GA_HUM)==0) {
//          knx.groupAnswer2ByteFloat(KNX_GA_HUM, hum);
//     } 
//     if (strcmp(target.c_str(),KNX_GA_CO2)==0) {
//          knx.groupAnswer2ByteInt(KNX_GA_CO2, airQuality);
//     } 
//     if (strcmp(target.c_str(),KNX_GA_TVOC)==0) {
//          knx.groupAnswer2ByteInt(KNX_GA_TVOC, airTvoc);
//     }
//     if (strcmp(target.c_str(),KNX_GA_BA)==0) {
//          knx.groupAnswer1ByteInt(KNX_GA_BA, BA);
//     } 
//     if (strcmp(target.c_str(),KNX_GA_SOLL)==0) {
//          knx.groupAnswer2ByteFloat(KNX_GA_SOLL, Sollwert);
//     } 
     readT=false; 
    }
    if ((telegram->getCommand() == KNX_COMMAND_WRITE)&&(KNX_receive)) {
      readT=true; 
      if (strcmp(target.c_str(),KNX_GA_SOLL)==0) {
          Sollwert = telegram->get2ByteFloatValue(); 
      }  
      if (strcmp(target.c_str(),KNX_GA_BA)==0) {
          BA = telegram->get1ByteIntValue();
      }
      if (strcmp(target.c_str(),KNX_GA_VALVE)==0) {
          y_res = telegram->get1ByteIntValue();
          y_res = y_res/2.55;
      }    
     readT=false; 
    }
  }  
}  
////////////////////////////////////////////////////////////////////VOC auslesen
void readiAQ() {
int iAQRes;
  Wire.requestFrom(iaqaddress, 9);

  airQuality = (Wire.read()<< 8 | Wire.read()); 
  iAQstatus = Wire.read();
  iAQRes = (Wire.read()& 0x00)| (Wire.read()<<8)| (Wire.read()<<8| Wire.read());
  airTvoc = (Wire.read()<<8 | Wire.read());
}

void MinMax_Window_Open() {
   MinMax_active=true; 
   initDisplay_MinMax();                                                                                                         //MinMax-Fenster initialisiern
   oldSec5_Anz=Sec5_Anz; Sec5_Anz=7;                                                                                             //Sektion 5 Variante merken
   showSection(1,mClass[1],minTemp_str,ColorSecMinMax[1][0],BackgSecMinMax[1],FontHMinMax[1],centi,FontHEinhMinMax[1],2,Min);    //Sektion 1 schreiben          
   showSection(2,mClass[2],maxTemp_str,ColorSecMinMax[2][0],BackgSecMinMax[2],FontHMinMax[2],centi,FontHEinhMinMax[2],2,Max);    //Sektion 2 schreiben
   showSection(3,mClass[3],minHum_str,ColorSecMinMax[3][0],BackgSecMinMax[3],FontHMinMax[3],perc,FontHEinhMinMax[3],2,Min);      //Sektion 3 schreiben        
   showSection(4,mClass[4],maxHum_str,ColorSecMinMax[4][0],BackgSecMinMax[4],FontHMinMax[4],perc,FontHEinhMinMax[4],2,Max);      //Sektion 4 schreiben
   showSection(5,mClass[5],KNX_PA,ColorSecMinMax[5][0],BackgSecMinMax[5],FontHMinMax[5],PA,FontHEinh[4],1,empty);   //Sektion 5 schreiben
}

void RTR_Window_Open() {
   RTRWin_active=true; 
   initDisplay_RTR();                                                                                                            //RTR-Fenster initialisiern
   oldSec5_Anz=Sec5_Anz; Sec5_Anz=6;                                                                                             //Sektion 5 Variante merken
   switch (BA) {                                                                                                                 //Abfrage welche Anzeige Sektion 5
      case 1: BA_str="KOMFORT"; 
              break;
      case 3: BA_str="ECO"; 
              break;
      case 4: BA_str="FROST";                
              break;
   }
   Soll_str=float2str(Sollwert,1);
   showSection(1,mClass[1],BA_str,ColorSecMinMax[1][0],BackgSecMinMax[1],FontHMinMax[1],empty,FontHEinhMinMax[1],1,empty);      //Sektion 1 schreiben          
   showSection(2,mClass[2],Soll_str,ColorSecMinMax[2][0],BackgSecMinMax[2],FontHMinMax[1],centi,FontHEinhMinMax[2],1,empty);    //Sektion 2 schreiben
   showSection(3,mClass[3],"",ColorSecMinMax[3][0],BackgSecMinMax[3],FontHMinMax[3],empty,FontHEinhMinMax[3],1,empty);          //Sektion 3 schreiben        
   showSection(4,mClass[4],"",ColorSecMinMax[4][0],BackgSecMinMax[4],FontHMinMax[4],empty,FontHEinhMinMax[4],1,empty);          //Sektion 4 schreiben
   showSection(5,mClass[5],Valve_str,ColorSecMinMax[5][0],BackgSecMinMax[5],FontHMinMax[5],perc,FontHEinh[4],2,Ven);          //Sektion 5 schreiben
}

void MinMax_Window_Close() {
byte i=0;  
   MinMax=false; MinMax_active=false;                                                                                            //MinMax rücksetzen
   dispRenew=false;                                                                                                              //Vorgabe Neuaufbau Display           
   initDisplay();                                                                                                                //Messwert-Fenster initialisieren
   secRenew[1]=false; secRenew[2]=false; secRenew[3]=false; secRenew[4]=false;secRenew[5]=false;                                 //Vorgabe Neuaufbau Sektionen
   showSection(1,mClass[1],temp_str,ColorSecMain[1][0],BackgSecMain[1],FontHMess[1][0],centi,FontHEinh[1],1,empty);              //Sektion 1 schreiben  
   showSection(2,mClass[2],hum_str,ColorSecMain[2][0],BackgSecMain[2],FontHMess[2][0],perc,FontHEinh[2],1,empty);                //Sektion 2 schreiben 
   if (CO2LimitLow()) i=0;                                                                                                       //CO2-Wert unter Grenzwert     
   if (CO2LimitMiddle()) i=1;                                                                                                    //CO2-Wert Grenzwert 1 überschritten 
   if (CO2LimitHigh()) i=2;                                                                                                      //CO2-Wert Grenzwert 2 überschritten
   showSection(3,mClass[3],CO2_str,ColorSecMain[3][i],BackgSecMain[3],FontHMess[3][0],CO2,FontHEinh[3],2,ppm);                   //Sektion 3 schreiben 
   if (VOCLimitLow()) i=0;                                                                                                       //VOC-Wert unter Grenzwert     
   if (VOCLimitMiddle()) i=1;                                                                                                    //VOC-Wert Grenzwert 1 überschritten 
   if (VOCLimitHigh()) i=2;                                                                                                      //VOC-Wert Grenzwert 2 überschritten
   showSection(4,mClass[4],VOC_str,ColorSecMain[4][i],BackgSecMain[4],FontHMess[4][0],VOC,FontHEinh[4],2,ppb);                   //Sektion 4 schreiben          
   showSection(5,mClass[5],"               ",ColorSecMain[5][0],BackgSecMain[5],FontHMess[5][0],empty,FontHEinh[4],1,empty);     //Sektion 5 schreiben 
   ambientlight_control();
   Sec5_Anz=oldSec5_Anz;                                                                                                         //Sektion 5 Variante wiederherstellen
   if (Sec5_Anz=5) {oldiAQstatus=255;}                                                                                           //Wenn Variante Sektion 5 Status iAQ, dann fiktiven Status setzen
   MM_closed=true;
   hum_send_show(); 
}
void RTR_Window_Close() {
byte i=0;
   RTRWin=false; RTRWin_active=false;                                                                                                                  //RTR rücksetzen
   dispRenew=false;                                                                                                              //Vorgabe Neuaufbau Display           
   initDisplay();                                                                                                                //Messwert-Fenster initialisieren
   secRenew[1]=false; secRenew[2]=false; secRenew[3]=false; secRenew[4]=false;secRenew[5]=false;                                 //Vorgabe Neuaufbau Sektionen
   showSection(1,mClass[1],temp_str,ColorSecMain[1][0],BackgSecMain[1],FontHMess[1][0],centi,FontHEinh[1],1,empty);              //Sektion 1 schreiben  
   showSection(2,mClass[2],hum_str,ColorSecMain[2][0],BackgSecMain[2],FontHMess[2][0],perc,FontHEinh[2],1,empty);                //Sektion 2 schreiben 
   if (CO2LimitLow()) i=0;                                                                                                       //CO2-Wert unter Grenzwert     
   if (CO2LimitMiddle()) i=1;                                                                                                    //CO2-Wert Grenzwert 1 überschritten 
   if (CO2LimitHigh()) i=2;                                                                                                      //CO2-Wert Grenzwert 2 überschritten   
   showSection(3,mClass[3],CO2_str,ColorSecMain[3][i],BackgSecMain[3],FontHMess[3][0],CO2,FontHEinh[3],2,ppm);                   //Sektion 3 schreiben 
   if (VOCLimitLow()) i=0;                                                                                                       //VOC-Wert unter Grenzwert     
   if (VOCLimitMiddle()) i=1;                                                                                                    //VOC-Wert Grenzwert 1 überschritten 
   if (VOCLimitHigh()) i=2;                                                                                                      //VOC-Wert Grenzwert 2 überschritten
   showSection(4,mClass[4],VOC_str,ColorSecMain[4][i],BackgSecMain[4],FontHMess[4][0],VOC,FontHEinh[4],2,ppb);                   //Sektion 4 schreiben                   
   showSection(5,mClass[5],"               ",ColorSecMain[5][0],BackgSecMain[5],FontHMess[5][0],empty,FontHEinh[4],1,empty);     //Sektion 5 schreiben 
   ambientlight_control();
   Sec5_Anz=oldSec5_Anz;                                                                                                         //Sektion 5 Variante wiederherstellen
   if (Sec5_Anz=5) {oldiAQstatus=255;}                                                                                           //Wenn Variante Sektion 5 Status iAQ, dann fiktiven Status setzen
   hum_send_show(); 
}

void But_pressed(){
    byte curState=digitalRead(SW1);                                                                                              //Aktueller Buttonstatus
    if (curState!=buttonState)                                                                                                   //Flankenwechsel erkannt
    {
      if (curState==LOW) { 
          buttonChange=BUTTONDOWN; buttonDownTime = millis(); SW1_pressed=true;                                                  //Wenn LOW, Zeit messen und Tastendruck registrieren 
      }    
      else {buttonChange=BUTTONUP; SW1_pressed=false; }
    }
    buttonState=curState;                                                                                                        //den jetzigen Buttonzustand speichern
}

void piep_control() {
if (piep) {
    digitalWrite(A1, LOW);                                                                                               //dreimal kurz piepsen
    delay(20);
    digitalWrite(A1, HIGH);
    delay(100);
    digitalWrite(A1, LOW);
    delay(20);
    digitalWrite(A1, HIGH);
    delay(100);
    digitalWrite(A1, LOW);
    delay(20);
    digitalWrite(A1, HIGH);
    piep=false;
}else{digitalWrite(A1, HIGH);}
}

void ambientlight_control() {
  
ambi_mapped=map(ambientlight,0,1023,map_TFT_min,map_TFT_max);                                                           //Helligkeitswert mappen 
analogWrite(PWM_BL_Pin, ambi_mapped);                                                                                   //Helligkeit nach Mapping regeln                              
ambi_str=float2str(ambientlight,0);                                                                                     //Helligkeitswert in String umwandeln
  if (LEDs_Anz) {                                                                                                       //Wenn LED-Anzeigen aktiv
    if (CO2LimitLow()) {LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);analogWrite(LED_G, LED_mapped);analogWrite(LED_R, 255);}                                                                         //Wenn unter Grenzwert -> Grün anzeigen
    if (CO2LimitMiddle()) {LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);analogWrite(LED_G, LED_mapped);LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);analogWrite(LED_R, LED_mapped);}   //Wenn Grenzwert 1 erreicht Grün und Rot auf Gelb mischen 
    if (CO2LimitHigh()) {LED_mapped=map(ambientlight,0,1023,map_LED_min,map_LED_max);analogWrite(LED_R, LED_mapped);analogWrite(LED_G, 255);}                                                                        //Wenn Grenzwert 2 erreicht Rot anzeigen
  }
  if (LEDB_Anz) {                                                                                                                                                                                                    //Wenn blaue LED aktiv 
    if (hum>65) {LED_mapped=map(ambientlight,0,1023,255,50);analogWrite(LED_B, LED_mapped);}else{analogWrite(LED_B, 255);}                                                                                           //Blaue LED bei Feuchtewert >65% anzeigen
  } 
}

String float2str(float f, int n) {                                                                                    //Funktion zur Umwandlung der Messwerte in Strings
String fs="12345.67890";
int i, k, st=0;
float teiler;
double d,e;
char c;

  fs = "";
  d = f;
  for(i=10000; i >0; i/=10) {
    // 10000er 1000er 100er 10er 1er
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



void temp_send_show(){
if (minTemp==0) minTemp=temp;                                                                                       //MinTemp setzen  
if (temp<minTemp) minTemp=temp;
if (temp>maxTemp) maxTemp=temp;                                                                                     //MaxTemp setzen
oldTemp = temp;                                                                                                     //Temperatur merken
temp_str=float2str(temp,1);                                                                                         //Temperatur in String umwandeln
minTemp_str=float2str(minTemp,1);                                                                                   //MinTemp in String umwandeln
maxTemp_str=float2str(maxTemp,1);                                                                                   //MaxTemp in String umwandeln
result = knx.groupWrite2ByteFloat(KNX_GA_TEMP, temp);                                                               //Temperatur auf den KNX-Bus senden
showSection(1,mClass[1],temp_str,ColorSecMain[1][0],BackgSecMain[1],FontHMess[1][0],centi,FontHEinh[1],1,empty);    //showSection(Sektion Nr, Messwertart, Var, Vordergrundfarbe, Hintergrundfarbe, Schriftgröße, Einheit, Schriftgröße Einheit, Einheiten 1/2, Einheit Zeile 2); 
                  
  if (Taup_Anz) {                                                                                                   //Taupunktberechnung schnelle Methode
     float a1 = 17.271;
     float b1 = 237.7;
     float temp1 = (a1 * temp) / (b1 + temp) + log(hum/100);
     tp = (b1 * temp1) / (a1 - temp1);
     if (tp<0) tp=0;
     Tau_str=float2str(tp,1); 
//     if (tp!=oldtp) {                                                                                               //Änderung Taupunkt
//         Tau_str=float2str(tp,1);                                                                                   //Taupunkttemperatur in String umwandeln
//         oldtp=tp;                                                                                                  //Taupunkt merken  
//     }    
  }
}

void hum_send_show(){

if (hum>99) hum=99;                                                                                                 //Feuchte auf 99% begenzen
if (minHum==0) minHum=hum;                                                                                          //MinFeuchte setzen
if (hum<minHum) minHum=hum;
if (hum>maxHum) maxHum=hum;                                                                                         //MaxFeuchte setzen
oldHum = hum;                                                                                                       //Feuchte merken
hum_str=float2str(hum,0);                                                                                           //Feuchtewert in String umwandeln
minHum_str=float2str(minHum,1);                                                                                     //MinFeuchte in String umwandeln
maxHum_str=float2str(maxHum,1);                                                                                     //MaxFeuchte in String umwandeln  
result = knx.groupWrite2ByteFloat(KNX_GA_HUM, hum);                                                                 //Feuchte auf den KNX-Bus senden
byte i=0;
  if (hum>65) {i=1;}else{i=0;}                                                                                      //Wenn Feuchte > 65% dann Farbwechsel auf Blau
      showSection(2,mClass[2],hum_str,ColorSecMain[2][i],BackgSecMain[2],FontHMess[2][0],perc,FontHEinh[2],1,empty);//Sektion schreiben
}  

void iAQ_send_show(){
byte i=0;
readiAQ();                                                                                                           //iAQ: VOC und CO2 lesen

if (iAQstatus != oldiAQstatus) {                                                                                     //Hat sich der Status des iAQ geändert
     oldiAQstatus = iAQstatus;                                                                                       //iAQ Status nerken
     result = knx.groupWrite1ByteInt(KNX_GA_STATUS, iAQstatus);
}
if (airQuality >= (oldairQuality + 50) || airQuality <= (oldairQuality - 50)) {                                      //Bei Änderung des CO2-Wertes um mehr oder gleich 50ppm  
     oldairQuality = airQuality;                                                                                     //CO2-Wert merken  
     CO2_str=String(airQuality);                                                                                     //CO2-Wert in String umwandeln
     result = knx.groupWrite2ByteInt(KNX_GA_CO2, airQuality);                                                        //Co2-Wert auf KNX-Bus senden
     if (CO2LimitLow()) i=0;                                                                                         //CO2-Wert unter Grenzwert     
     if (CO2LimitMiddle()) i=1;                                                                                      //CO2-Wert Grenzwert 1 überschritten 
     if (CO2LimitHigh()) i=2;                                                                                        //CO2-Wert Grenzwert 2 überschritten 
   //showSection(Sektion Nr, Variable String, Vordergrundfarbe, Hintergrundfarbe, Schriftgröße, Einheit, Schriftgröße Einheit, Einheiten 1 oder 2, 2.Einheit);
     showSection(3,mClass[3],CO2_str,ColorSecMain[3][i],BackgSecMain[3],FontHMess[3][0],CO2,FontHEinh[3],2,ppm);     //Sektion 3 schreiben
}
if (airTvoc >= (oldairTvoc + 30) || airTvoc <= (oldairTvoc - 30)) {                                                  //Bei Änderung des VOC-Wertes um mehr oder gleich 30ppb
     oldairTvoc = airTvoc;                                                                                           //VOC-Wert merken
     VOC_str=String(airTvoc);                                                                                        //VOC-Wert in String umwandeln
     result = knx.groupWrite2ByteInt(KNX_GA_TVOC, airTvoc);                                                          //VOC-Wert auf KNX-Bus senden  
     if (VOCLimitLow()) i=0;                                                                                         //VOC-Wert unter Grenzwert     
     if (VOCLimitMiddle()) i=1;                                                                                      //VOC-Wert Grenzwert 1 überschritten 
     if (VOCLimitHigh()) i=2;                                                                                        //VOC-Wert Grenzwert 2 überschritten
     showSection(4,mClass[4],VOC_str,ColorSecMain[4][i],BackgSecMain[4],FontHMess[4][0],VOC,FontHEinh[4],2,ppb);     //Sektion 4 schreiben                                      
}
if (LEDs_Anz) {ambientlight_control();} 
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


