Verkabelung:

Verbinden Sie Arduino Nano mit dem USBASP-Programmierer
(ArduinoISP wurde nicht mit den vorhandenen platformio.ini-Abschnitten getestet, Sie müssen es selbst schreiben)
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Beachten Sie, dass die Stiftbelegung der Anschlüsse unterschiedlich sein kann

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Kompilierung und Programmierung:

!!! info annotate "configuration.h"

    :arrow_forward:</i> Konfigurieren Sie die Firmware in der Datei configuration.h.
    

!!! warning annotate "Kompilierungsfehler"  
    Wenn bei der Kompilierung Fehler auftreten, die auf eine Nichtübereinstimmung zwischen der Firmware-Größe und der Speichergröße des Mikrocontrollers hindeuten, helfen oft die folgenden Befehle
    ```
    pio update
    pio pkg update
    ```

### Auswahl der Anzahl der Wägemodule

=== "2 Module"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3 Module"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4 Module"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. Programmierung des MiniCore-Kernels

Sie benötigen einen USBASP-Programmierer

2.1 Fuse-Wechsel


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 EEPROM-Programmierung

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### Programmierung des Mikrocontrollers

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### Auto-PID (PID-Kalibrierung)

Wird nach Bedarf durchgeführt

Der Prozess verläuft in zwei Stufen:
In configuration.h wird folgendes gesetzt
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = Anzahl der Waagen)
```
Die Programmierung des Punktes 2.3 wird durchgeführt
Nach der Programmierung beginnt die automatische PID-Einstellung. Am Ende wird auf dem Display "Прошей часть 2" angezeigt
ändern Sie dies zu #define AUTOPID_RUN 0
und führen Sie Punkt 2.3 durch

### Lüfter-Testprogrammierung

Wird nach Bedarf durchgeführt

In configuration.h kommentieren Sie auf
```
// #define PWM_TEST
```
führen Sie Punkt 2.3 durch
Der Lüfter-Test wird auf allen verfügbaren Frequenzen mit PWM-Auslastung von 100-10% durchgeführt und der Betriebsmodus wird auf dem Display angezeigt. Nach Abschluss des Tests sollten Sie basierend auf Ihren Vorlieben für Lärmunterdrückung und Lüfterleistung in configuration.h die gewünschte Frequenz einstellen
```
#define PWM_11_FREQUENCY
```
kommentieren Sie aus
```
#define PWM_TEST
```

programmieren Sie den Mikrocontroller mit Punkt 2.3

!!! warning annotate "Fehler beim Start"
    Wenn nach der Programmierung oder während des Betriebs Fehler auf dem Display angezeigt werden, lesen Sie bitte in der Datei configuration.h nach

## Schulungsvideo
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Links zum Video:

[VS Code](https://code.visualstudio.com/Download)

[Treiber](https://zadig.akeo.ie/)

[Firmware](https://github.com/pavluchenkor/iDryerController)


[Telegram-Gruppe :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Beiträge :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
