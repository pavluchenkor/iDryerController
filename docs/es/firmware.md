Conexión:

Conecte Arduino Nano con el programador USBASP
(ArduinoISP no ha sido probado con las secciones existentes en platformio.ini, es necesario escribirlo por su cuenta) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Tenga en cuenta que el diagrama de pines de los conectores puede diferir

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Compilación y programación:

!!! info annotate "configuration.h"

    :arrow_forward:</i> Configure el firmware en el archivo configuration.h.
    

!!! warning annotate "Errores de compilación"  
    Si se producen errores durante la compilación indicando una discrepancia entre el tamaño del firmware y la memoria del microcontrolador, generalmente ayuda ejecutar secuencialmente los comandos
    ```
    pio update
    pio pkg update
    ```

### Selección de la cantidad de módulos de pesaje

=== "2 módulos"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3 módulos"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4 módulos"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. Programación del núcleo MiniCore

Se requiere un programador USBASP

2.1 cambio de fusibles


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 Programación de EEPROM

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### Programación del microcontrolador

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### Calibración automática PID (Pid calibrate)

Se ejecuta cuando es necesario

Se realiza en dos etapas:
En configuration.h establezca
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = cantidad de módulos de pesaje)
```
Ejecute la programación del paso 2.3
Después de la programación comenzará la configuración automática del PID, al finalizar aparecerá en la pantalla "Прошей часть 2"
cambie a #define AUTOPID_RUN 0
y ejecute el paso 2.3

### Programación de la prueba del ventilador

Se ejecuta cuando es necesario

En configuration.h descomente 
```
// #define PWM_TEST
```
ejecute el paso 2.3
Comenzará la prueba del ventilador en todas las frecuencias disponibles con un llenado PWM del 100-10% y mostrará el modo de funcionamiento en la pantalla. Al finalizar la prueba, según sus preferencias sobre el nivel de ruido y la calidad del funcionamiento del ventilador, establezca en configuration.h 
```
#define PWM_11_FREQUENCY
```
la frecuencia deseada, comente 
```
#define PWM_TEST
```

programe el microcontrolador paso 2.3

!!! warning annotate "Errores al iniciar"
    Si después de la programación y durante la operación aparecen errores en la pantalla, consulte el archivo configuration.h


## Video educativo
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Enlaces del video:

[VS Code](https://code.visualstudio.com/Download)

[Controlador](https://zadig.akeo.ie/)

[Firmware](https://github.com/pavluchenkor/iDryerController)


[Grupo en Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribuir :material-file-edit:](https://github.com/pavluchenkor/iDryerController)

