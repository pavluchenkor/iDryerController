Zapojení:

Připojte Arduino Nano k programátoru USBASP
(ArduinoISP nebylo testováno se stávajícími sekcemi platformio.ini, je třeba napsat vlastní) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Mějte na paměti, že zapojení konektorů se může lišit

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Kompilace a nahrání firmware:

!!! info annotate "configuration.h"

    :arrow_forward:</i> V souboru configuration.h nakonfigurujte firmware.
    

!!! warning annotate "Chyby během kompilace"  
    Pokud během kompilace dojde k chybám oznamujícím nesoulad velikosti firmware s velikostí paměti mikrokontroléru, obvykle pomůže postupné zadání příkazů
    ```
    pio update
    pio pkg update
    ```

### Výběr počtu modulů vah

=== "2 moduly"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3 moduly"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4 moduly"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. Nahrání jádra MiniCore

Bude potřebný programátor USBASP

2.1 změna fuse bitů


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 nahrání EEPROM

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### Nahrání mikrokontroléru

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### Automatické PID (Pid calibrate)

Provádí se v případě potřeby

Probíhá ve dvou fázích:
V configuration.h se nastaví
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = počet vah)
```
Provede se nahrání bodu 2.3
Po nahrání se začne automatická kalibrace PID, po dokončení se na obrazovce objeví nápis "Proshey chast 2"
změňte na #define AUTOPID_RUN 0
a proveďte bod 2.3

### Nahrání testu ventilátoru

Provádí se v případě potřeby

V configuration.h odkomentujte
```
// #define PWM_TEST
```
proveďte bod 2.3
Začne test ventilátoru na všech dostupných frekvencích s vyplněním PWM 100-10% a výstupem režimu práce na obrazovku. Po skončení testu, na základě svých preferencí týkajících se úrovně hluku a kvality provozu ventilátoru, nastavte v configuration.h
```
#define PWM_11_FREQUENCY
```
požadovanou frekvenci, odkomentujte
```
#define PWM_TEST
```

nahrát mikrokontrolér bod 2.3

!!! warning annotate "Chyby při spuštění"
    Pokud se po nahrání a během provozu na obrazovce objeví chyby, obraťte se na soubor configuration.h


## Výukové video
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Odkazy k videu:

[VS Code](https://code.visualstudio.com/Download)

[Ovladač](https://zadig.akeo.ie/)

[Firmware](https://github.com/pavluchenkor/iDryerController)


[Skupina v telegramu :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contributing :material-file-edit:](https://github.com/pavluchenkor/iDryerController)

