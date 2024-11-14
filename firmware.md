Коммутация:

Соедините Arduino Nano с программатором USBASP
(ArduinoISP не тестировалось с существующими секциями platfornio.ini, необходимо писать самостоятельно) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Обратите внимание, что цоколевка разъемов может отличаться

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Компиляция и прошивка:

!!! info annotate "configuration.h"

    :arrow_forward:</i> В файле configuration.h сконфигурируйте прошивку.
    

!!! warning annotate "Ошибки при компиляции"  
    Если при компиляции возникают ошибки указывающие на несоответсвие размера прошивки размеру памяти МК чаще всего помогает последовательный ввод команд
    ```
    pio update
    pio pkg update
    ```



1. Прошивка ядра MiniCore
Потребуется программатор USBASP

2.1 смена фьюзов
```

```

=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```


2.2 прошивка EEPROM<br>

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


###Прошивка общая
2.3 прошивка МК<br>

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```


### Прошивка для работы с модулями весов

=== "2 модуля"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3 модуля"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4 модуля"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### Автопид (Pid calibrate)

Выполняется в два этапа:
В configuration.h устанавливается
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = количество весов)
```
Выполняется прошивка п.2.3
После прошивки начнется автоматическая настройдка PID, по окончании на экране появится надпись "Прошей часть 2"
сменить на #define AUTOPID_RUN 0
и выполнить п.2.3

### Прошивка теста кулера

Выполняется при необходимости
В configuration.h раскомментировать 
```
// #define PWM_TEST
```
выполнить п.2.3
Начнется тест кулера на всех доступных частотах с заполнением ШИМ 100-10% и выводом режима работы на экран. По окончании теста, ориентируясь на свои предпочтения по уровню шума и качеству работы кулера установить в configuration.h 
```
#define PWM_11_FREQUENCY
```
желаемую частоту, закомментировать 
```
#define PWM_TEST
```

прошить МК п.2.3

!!! warning annotate "Ошибки при старте"
    Если после прошивки и в процессе эксплуатации на экране появятся ошибки, обратитесь к файлу configuration.h

## Учебное видео
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Ссылки к видео:

[VS Code](https://code.visualstudio.com/Download)

[Драйвер](https://zadig.akeo.ie/)

[Прошивка](https://github.com/pavluchenkor/iDryerController)


[группа в телеграмм :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contributing :material-file-edit:](https://github.com/pavluchenkor/iDryerController)


