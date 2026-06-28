## Kalibrierung des Lufttemperatursensors

Die verwendeten Lufttemperatursensoren vom Typ BME280 zeigen Nichtlinearität der Messwerte an den Rändern des Bereichs. Um dies auszugleichen, wird ein linearer Korrekturalgorithmus verwendet.

Der Algorithmus erhält Daten aus dem Konfigurationsabschnitt configuration.h

```
#define MIN_CALIB_TEMP 70

#define MAX_CALIB_TEMP CE

#define REAL_CALIB_TEMP_MIN 70

#define REAL_CALIB_TEMP_MAX 110

```

wobei:

- MIN_CALIB_TEMP	Temperatur, unter der die Messwerte nicht korrigiert werden
- MAX_CALIB_TEMP	Obergrenze des Bereichs, bei der die Kalibrierung angewendet wird
- REAL_CALIB_TEMP_MIN	Temperatur laut Referenzsensor bei MIN_CALIB_TEMP
- REAL_CALIB_TEMP_MAX	Temperatur laut Referenzsensor bei MAX_CALIB_TEMP

### Vorbereitung zum Test
Konfigurieren Sie das System mit angeschlossenem bmeTemp-Sensor und Referenz-Thermistor.

Vergewissern Sie sich, dass die Kalibrierparameter eingestellt sind:
```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 oder 110 (abhängig von der Konfiguration)
REAL_CALIB_TEMP_MAX = 110 (abhängig von der Konfiguration)
```
Möglichkeit haben, genaue Temperaturwerte am Eingang festzulegen


### Testschritte
Überprüfung des Bereichs ohne Kalibrierung:

Stellen Sie 65°C ein und überprüfen Sie mit dem Referenzsensor

Wiederholen Sie dies für Werte von 70, 75 usw. Suchen Sie die Temperatur, bei der die Daten des Referenzmessgeräts und des eingebauten Sensors beginnen abzuweichen, geben Sie diese Temperatur in MIN_CALIB_TEMP und REAL_CALIB_TEMP_MIN ein

Überprüfung der Kalibrierungsgrenze:

- Stellen Sie die maximale Temperatur auf CE ein
- Warten Sie, bis die eingestellte Temperatur erreicht ist
- Tragen Sie die Messwerte des Referenzsensors in REAL_CALIB_TEMP_MAX ein
- Speichern Sie die Änderungen
- Flashen Sie den Controller mit dem Befehl

```
pio run -t clean -t upload
```

## Waagenkalibrierung

### Installation der Waage

Für den ordnungsgemäßen Betrieb der Waage müssen folgende Bedingungen erfüllt sein:
- ordnungsgemäße Installation
- ordnungsgemäße Verkabelung

#### Installation
Bei der Installation muss sichergestellt werden, dass das Wagenmodul 1,5-2 mm von der Platte entfernt ist und das Verbundmaterial die Platte nicht berührt.

<!-- #### Коммутация
Как правило у весов черный провод(-), красный(+) подключаются к E+ и E- соответственно. Остальные два провода подключаются к A+ и А-. 
Чтобы понять какой куда, необходимо подключить черный и красный провода к питанию, перевести мультиметр в режим измерения постоянного напряжения и соединить черный провод(GND/-) мультиметра с черным проводом тензодатчика, красный провод соединить щуп мультиметра с белым проводам, измерить напряжение и надавить на тензодатчик, показания мультиметра изменятся на доли вольта, если изменились вверх, этот провод подключается к А+, вниз - А-. -->

#### Kalibrierung

Entfernen Sie die Spule aus dem zu kalibrierenden Modul.
Wählen Sie im Menü SCALE den Punkt SET aus.
Warten Sie auf die Anzeige PUT WEIGHT 1000G und platzieren Sie das Kalibriergewicht von 1000 g auf dem Spulenhalter.
Wiederholen Sie den Vorgang für jedes Modul.

Wenn die Waage das falsche Gewicht anzeigt, überprüfen Sie:
- Korrektheit der Verkabelung
- Korrektheit der Befestigung
- Position des Nullpotentiometers im hx711-Modul
- Temperatur des ADC-Chips (sollte nicht heiß werden)
