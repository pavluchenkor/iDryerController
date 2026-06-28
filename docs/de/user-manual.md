### Gerätezweck und allgemeine Informationen

Das Gerät ist zum Trocknen und Lagern von Filamenten für 3D-Druck bestimmt. Das Gerät wird entweder als KIT-Set oder in montierter Form geliefert. Das vormontierte Gerät ist bereits konfiguriert und erfordert keine speziellen Einstellungen.

### Gerätebedienung

#### Ein- und Ausschalten des Geräts

Das Gerät wird durch einen Schalter auf der Vorderseite ein- und ausgeschaltet. Beim Einschalten wird die aktuelle Firmware-Version des Geräts angezeigt, und der Lüfter kann sich einschalten, wenn die Temperatur in der Kammer 40°C überschreitet. Dieses Verhalten ist normal. Das Ausschalten des Trockners erfordert keine vorherigen Maßnahmen.

#### Menü

Die gesamte Gerätebedienung erfolgt über das Menü. Die Menünavigation erfolgt mit einem Drehschalter (Encoder).

* Drehung im Uhrzeigersinn - Wert erhöhen oder nächsten Menüpunkt wählen  
* Drehung gegen den Uhrzeigersinn - Wert verringern oder vorherigen Menüpunkt wählen  
* Kurzes Drücken - gewählten Menüpunkt aktivieren  
* Langes Drücken - zum vorherigen Menü zurückkehren oder den aktuellen Betriebsmodus beenden (Trocknung oder Lagerung)

### Filament-Trocknungsmodus

Laden Sie die erforderliche Anzahl von Filamentriolen in den Trockner und führen Sie das Filament optional durch PTFE-Rohre, falls der Druck direkt aus dem Trockner erfolgen soll. Stellen Sie im Menü DRYING die Temperatur (`TEMPERATURE`) und die Zeit in Minuten (`TIME`) ein. Starten Sie den Trocknungsmodus durch Auswahl von `START`. Falls nötig, beenden Sie das Trocknen durch langes Drücken des Encoders.

Im Trocknungsmodus wechselt das Display periodisch zwischen Skalenablesung und Trocknungsparametern:

* Die obere Menüzeile zeigt die Zieltemperatur und die verbleibende Zeit an  
* `AIR` - Lufttemperatur in der Kammer  
* `HEATER` - Temperatur des Heizelementes  
* `HUMIDITY` - relative Luftfeuchte in der Kammer

Der Betriebsalgorithmus des Geräts im Trocknungsmodus:

1. Aufheizen auf Zieltemperatur mit Lüfter auf 100%  
2. Temperatur halten mit reduzierter Lüfterdrehzahl für die angegebene Zeit  
3. Übergang in den Lagerungsmodus gemäß den Einstellungen

### Filament-Lagerungsmodus

Der Filament-Lagerungsmodus wird entweder manuell über das Menü STORAGE gestartet oder automatisch nach Abschluss des Trocknungsvorgangs. In diesem Modus hält das Gerät die Zielrelativfeuchte in der Kammer aufrecht, sodass Sie jederzeit mit dem Druck beginnen können, ohne das Filament vorher zu trocknen. Stellen Sie im Menü `STORAGE` die Zielrelativfeuchte `HUMIDITY` und die maximal zulässige Temperatur `TEMPERATURE` ein. Starten Sie den Lagerungsmodus durch Auswahl von START. Beenden Sie den Lagerungsmodus durch langes Halten des Encoders.

Im Lagerungsmodus wechselt das Display periodisch zwischen Skalenablesung und Lagerungsparametern:

* Die obere Menüzeile zeigt die Zieltemperatur und die verbleibende Zeit an  
* `AIR` - Lufttemperatur in der Kammer  
* `HEATER` - Temperatur des Heizelementes  
* `HUMIDITY` - relative Luftfeuchte in der Kammer

### 

### Einstellung von Parametern, die das Trocknen beeinflussen

Die Einstellungen befinden sich im Menü `SETTINGS`. Sie können die Einstellungen für Luftstrom, den Unterschied zwischen Kammertemperatur und Heizelement-Temperatur, Klappen und PID ändern. Nach dem Ändern der Parameter müssen diese durch Aufrufen von `SETTINGS -> SAVE` gespeichert werden

#### Lüfterkonfiguration

`AIRFLOW`  
Stellt den Prozentsatz des Lüfterbatriebs nach Erreichen der Betriebstemperatur ein, um den Geräteschall zu reduzieren. Der empfohlene Wert beträgt mindestens 70.

#### Einstellung des Temperaturdifferenzes

`DELTA`  
Stellt den Zielwert des Unterschieds zwischen der Heizelement-Temperatur und der Temperatur in der Trocknerkammer ein. Der empfohlene Wert beträgt 20. Ein zu großer Unterschied kann zu ungleichmäßiger Erwärmung und beispielsweise zum Schmelzen von Filament führen. Ein zu kleiner Unterschied führt zu langen Aufheizzeiten.

#### Einstellungen für das Verschluss-Servoangetrieb

`SETTINGS -> SERVO` 
Der Trockner ist mit einer Belüftungsklappe ausgestattet, die über einen Servomotor angetrieben wird, der feuchter Luft ermöglicht, aus der Trocknerkammer auszutreten. Während des Betriebs öffnet der Trockner die Klappe zur Belüftung und schließt sie danach regelmäßig.

* `CLOSED` - Zeit in Minuten, wenn die Klappe geschlossen ist  
* `OPEN` - Zeit in Minuten, wenn die Klappe offen ist  
* `CORNER` - Winkel, um den die Klappe geöffnet wird (empfohlener Wert - 85)  
* `TEST` - Klappe öffnen/schließen zum Testen

#### PID-Regler-Einstellungen

`PID`  
PID-Einstellungen (proportional-integral-differential controller). Dieser Bereich ist für fortgeschrittene Benutzer gedacht und erfordert in der Regel keine Anpassung

* `kP`  
  Verstärkungskoeffizient der proportionalen Komponente des Reglers  
* `kI`  
  Verstärkungskoeffizient der integralen Komponente des Reglers  
* `kD`  
  Verstärkungskoeffizient der differentiellen Komponente des Reglers  
* `AUTOPID`  
  automatische PID-Einstellung

### Skaleneinstellung

#### Skalenkalibrierung

Die Skaleneinstellung erfolgt separat für jede Spule. Bereiten Sie ein Gewicht von genau 1 kg vor, das stabil auf der Spulenplattform stehen kann.

Gehen Sie zum Menü `SCALE -> SPOOLX -> SET`. Entfernen Sie alles von der Skalenplattform, wenn es auf dem Display angezeigt wird. Stellen Sie dann das vorbereitete Gewicht von 1 kg auf die Plattform, wenn die entsprechende Meldung auf dem Display angezeigt wird.

#### Skalentarierung

Wenn Sie Filamenten auf identischen Spulen verwenden, ist es oft praktisch, eine "Tara" einzustellen - das Gewicht einer leeren Spule, sodass die Waage das Nettogewicht des verbleibenden Filaments anzeigt. Diese Operation ersetzt die Skalenkalibrierung nicht.

Stellen Sie eine leere Spule auf die Plattform, warten Sie, bis ihr Gewicht angezeigt wird, und stellen Sie es im entsprechenden Menü `SCALE -> SPOOLX -> TARE` ein
 

### Probleme und Lösungen

#### Der Trockner zeigt beim Einschalten eine Fehlermeldung an und/oder piept

Drücken Sie einmal den Drehschalter, um zum Hauptmenü zu gehen.   
Wenn das Gerät weiterhin piept - starten Sie es durch Ein- und Ausschalten neu. Versuchen Sie bei wiederholten Fällen, sich die Fehlernummer zu merken und informieren Sie den Entwickler.

#### Der Trockner beendet den Trocknungsmodus unerwartet

Dieses Problem kann durch zu starke Verlangsamung des Lüfters im Trocknungsmodus und die entsprechende Belastung der Stromversorgung verursacht werden. Stellen Sie den empfohlenen Wert im Menü `SETTINGS -> AIRFLOW` auf 75 oder höher ein. Wenn das Problem weiterhin besteht, stellen Sie diesen Wert auf 100.

#### Der Lüfter macht im Trocknungsmodus unhealthy Geräusche

Dieses Problem kann durch zu starke Verlangsamung des Lüfters im Trocknungsmodus verursacht werden. Stellen Sie den empfohlenen Wert im Menü `SETTINGS -> AIRFLOW` auf 75 oder höher ein

#### Die Waage zeigt unangemessene Werte

Überprüfen Sie die Straffheit der Wagenplattform zum Trockneraboden und ziehen Sie sie bei Bedarf mit einem Kreuzschlitzschraubenzieher an. Kalibrieren Sie die Waage nach der Anleitung

#### Die Einstellungen werden nicht angewendet, obwohl ich sie geändert habe

Nach dem Ändern von Einstellungen im Menü SETTINGS müssen diese im Menü `SETTINGS -> SAVE` gespeichert werden

#### Die Trocknerdisplay geht nicht sofort nach dem Ausschalten aus

Dieses Verhalten ist normal


### Anlage 1: empfohlene Trocknungsparameter

|  | Trocknung |  | Lagerung |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Temperatur, ℃ | Zeit, h | Temperatur, ℃ | Feuchte |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 60-70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80-110 | 12 | 65 | 10 |
| PA-CF, Ultran | 90-110 2-4h, dann 80 4h | 6 | 70 | 10 |
 
!!! warning "Haftungsausschluss"
    Dieses Anhang bietet allgemeine Empfehlungen. Überprüfen Sie vor dem Trocknen unbedingt die vom Filamenthersteller angegebenen Parameter und beurteilen Sie das Ergebnis nach der Druckqualität.

    Denken Sie daran: Einige unredliche Hersteller können die empfohlenen Temperaturen für einfache Bedienung unterschätzen, anstatt für optimale Qualität.


### Anlage 2: Menüstruktur

* `DRYING`  
  * `TEMPERATURE`  
  * `TIME`  
  * `START`  
* `STORAGE`  
  * `TEMPERATURE`  
  * `HUMIDITY`  
  * `START`  
* `PRESETS`  
  * `PLA`  
  * `PETG`  
  * `PA6`  
* `SETTINGS`  
  * `PID`  
    * `kP`  
    * `kI`  
    * `KD`  
    * `AUTOPID`  
  * `AIRFLOW`  
  * `DELTA`  
  * `SERVO`  
    * `CLOSED`  
    * `OPEN`  
    * `CORNER`  
    * `TEST`  
  * `SAVE`  
* `SCALE`  
  * `SPOOL1`  
    * `TARE`  
    * `SET`  
  * `SPOOL2`  
    * `TARE`  
    * `SET`
