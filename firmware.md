Коммутация:

Соедините Arduino Nano с программатором USBASP
(ArduinoISP не тестировалось с существующими секциями platfornio.ini, необходимо писать самостоятельно) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Обратите внимание, что цоколевка разъемов может отличаться

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Компиляция и прошивка:

!!! warning annotate "Внимание"

    :arrow_forward:</i> В файле configuration.h сконфигурируйте прошивку.
    
    :arrow_forward:</i> в файле platformio.ini указать USB порт к которому подключена ардуина


1. Прошивка ядра MiniCore
Потребуется программатор USBASP

2.1 смена фьюзов
```
pio run -e fuses -t fuses
```

2.2 прошивка EEPROM<br>

```
pio run -e EEP -t uploadeep
```
если используется Atmega328PB
```
pio run -e EEPPB -t uploadeep
```

###Прошивка общая
2.3 прошивка МК<br>
```
pio run -e EEP -t upload
```

если используется Atmega328PB
    
```
pio run -e EEPPB -t upload 
```

###Прошивка для работы с модулями весов
Выполняется в два этапа:
В configuration.h устанавливается
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM Х (Х = количество весов)
```
Выполняется прошивка п.2.3
После прошивки начнется автоматическая настройдка PID, по окончании на экране появится надпись "Прошей часть 2"
сменить на #define AUTOPID_RUN 0
и выполнить п.2.3

###Прошивка теста кулера
В configuration.h раскомментировать // #define PWM_TEST
выполнить п.2.3
Начнется тест кулера на всех доступных частотах с заполнением ШИМ 100-10% и выводом режима работы на экран. По окнчании теста, ориентируясь на свои предпочтения по уровню шума и качеству работы кулера установить в configuration.h #define PWM_11_FREQUENCY желаемую частоту,  закмментировать #define PWM_TEST, прошить МК п.2.3


> Если после прошивки и в процессе эксплуатации на экране появятся ошибки, обратитесь к файлу configuration.h

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
