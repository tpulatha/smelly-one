This software is the brain of the Smelly One KNX DIY project from the KNX User Forums. You can find more information [here](https://knx-user-forum.de/forum/%C3%B6ffentlicher-bereich/knx-eib-forum/diy-do-it-yourself/1138070-smelly-one#post1469614)

German description:

# Highlights Software:
maximale Benutzerfreundlichkeit durch User-Setup im Sketch für:

* Gruppenadressen und PA
* Freigabemöglichkeiten für: RTR, MinMax-Fenster, Piepser, Taupunkt, Luxwertanzeige, Status iAQ, RGB-LED, Blaue LED (Feuchte).
* Helligkeitsgrenzwerte für LED und TFT
* Abfragezeiten für iAQ, T/H, MinMax, Helligkeitsregelung
* Grenzwerte CO2 und VOC
* Freie Farbeinstellung Vordergrund/Hintergrund je Zeile
* Einstellung Schriftgröße

# Displaygestaltung:
## Hauptfenster:

* Strukturierter Aufbau durch Einteilung in 5 Zeilen.
* Die Messwerte und Einheiten können individuell platziert werden.
* Eine Meldezeile für Text und sonstige Messwerte (Lux, Taupunkt,..) in Zeile 5

## MinMax-Fenster:

* Erreichbar durch Druck auf den Button
* Werte werden zurückgesetzt durch einen langen Druck auf den Button
* Fenster schließt automatisch nach einstellbarem Zeitintervall
* Anzeige von Min/Max-Werten von Temperatur und Feuchte
* Anzeige der PA
* Anzeige anderer Werte möglich

## Smelly als RTR:

* Betriebsartenumschaltung und Sollwertänderung durch Druck auf den Button
* Smelly als PID-Regler, einzelne Regelglieder aktivierbar. Freie Kombinationsmöglichkeit.
* Einstellparameter Proportionalbeiwert, Integrierzeit, Arbeitspunkt, Stellungsbegrenzung Min/Max,
* Anzeige von Betriebsart und Ventilstellung im Regelbetrieb
* Fernsteuerbar durch KNX: Betriebsart, Sollwert


Ein Einsatz des Smelly One ohne Programmierkenntnisse ist möglich. Im User-Setup (selbsterklärend) sind die individuellen Einstellungen vorzunehmen. Arduino IDE zur Programmierung wird vorausgesetzt.

Installation:
- Arduino IDE lauffähig installieren
- Bibliotheken aus zip in das Verzeichnis libraries kopieren
- Smelly-Datei in IDE laden
- PA und GAs im Sketch mit eigenen Adressen füllen
- User-Setup: Gewünschte Anzeigeoptionen entsprechend anpassen

## ETS Config 

ETS-Dummy:
Status-iAQ Byte
Stellwert Byte DPT5.001
Betriebsart Byte
Feuchte 2Byte Gleitkommawert DPT9.007
Temperatur 2 Byte Gleitkommawert DPT9.001
VOC 2-Byte vorzeichenlos DPT7.*
CO2 2-Byte vorzeichenlos DPT7.*
Sollwert 2Byte Gleitkommawert DPT9.001

Stellwert, BA, Sollwert nur für RTR.


## TODO:

* Code Optimierung
* Libraries zum Github Repo hinzufügen/Linken
* Documentation