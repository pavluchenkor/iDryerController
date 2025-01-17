### Назначение устройства и общая информация

Устройство предназначено для сушки и хранения филамента для 3d-печати. Устройство поставляется либо в наборе KIT, либо в собранном виде. Устройство в собранном виде уже настроено и не требует каких-то специфических настроек. 

### Управление устройством

#### Включение и выключение устройства

Включение выключение производится переключателем на передней панели. Во время включения отображается текущая версия прошивки устройства, может включаться вентилятор при превышении температуры внутри камеры выше 40℃. Это поведение является нормой. Выключение сушилки не требует каких-либо предварительных действий.

#### Меню

Все управление устройством осуществляется через меню. Навигация по меню осуществляется при помощи поворотной кнопки (энкодера).

* Вращение по часовой стрелке – увеличение значения или выбор следующего пункта меню  
* Вращение против часовой стрелки – уменьшение значения или выбор предыдущего пункта меню  
* Краткое нажатие – активация выбранного пункта меню  
* Длинное нажатие – вернуться в предыдущее меню или остановить текущий режим работы (сушка или хранение)

### Режим сушка филамента

Загрузите в сушилку нужное количество катушек с филаментом, опционально заправив филамент в PTFE трубки, если подразумевается печать из сушилки. В меню DRYING выставьте температуру (`TEMPERATURE`) и время в минутах (`TIME`). Запустите режим сушки, выбрав пункт `START`. В случае необходимости, остановите сушку долгим нажатием энкодера.

В режиме сушки дисплей периодически переключается между показаниями весов и параметрами сушки:

* Верхняя строка меню указывает целевую температуру и оставшееся время  
* `AIR` – температура воздуха в камере  
* `HEATER` – температура нагревательного элемента  
* `HUMIDITY` – относительная влажность в камере

Алгоритм работы устройства в режиме сушки:

1. Нагрев до целевой температуры с включенным на 100% вентилятором  
2. Удержание температуры с пониженными оборотами вентилятора указанное время  
3. Переход в режим хранения в соответствии с настройками

### Режим хранения филамента

Режим хранения филамента запускается либо вручную через меню STORAGE, либо автоматически по окончанию процесса сушки. В этом режиме устройство поддерживает целевую относительную влажность в камере, что позволяет в любой момент начать печать без предварительной сушки филамента. В меню `STORAGE` установите целевую относительную влажность `HUMIDITY` и максимальную дозволенную температуру `TEMPERATURE`. Запустите режим хранения выбрав START. Остановите режим хранения долгим удержанием энкодера.

В режиме хранения дисплей периодически переключается между показаниями весов и параметрами хранения:

* Верхняя строка меню указывает целевую температуру и оставшееся время  
* `AIR` – температура воздуха в камере  
* `HEATER` – температура нагревательного элемента  
* `HUMIDITY` – относительная влажность в камере

### 

### Настройка параметров, влияющих на сушку

Настройки находятся в меню `SETTINGS`. Возможно поменять настройки потока воздуха, разницы между температурой в камере и температурой нагревательного элемента, шторки и PID. После изменения параметров необходимо их сохранить вызвав `SETTINGS -> SAVE`

#### Настройка вентилятора

`AIRFLOW`  
Устанавливает процент работы вентилятора после выхода на рабочую температуру для уменьшения шума устройства. Рекомендуемое значение – не меньше 70\.

#### Настройка разницы температур

`DELTA`  
Устанавливает целевое значение разницы между температурой нагревательного элемента и температурой в камере сушилки. Рекомендуемое значение – 20\. Слишком большая разница может привести к неравномерному нагреву и, например, оплавлению филамента. Слишком маленькая – к долгому времени нагрева.

#### Настройки сервопривода заслонки

`SETTINGS -> SERVO` 
Сушилка оборудована заслонкой для проветривания с сервоприводом, что позволяет влажному воздуху выходить из камеры сушилки. Во время работы сушилка периодически открывает заслонку для проветривания и закрывает ее обратно.

* `CLOSED` – время в минутах, когда заслонка закрыта  
* `OPEN` – время в минутах, когда заслонка открыта  
* `CORNER` – угол, на который открывается заслонка (рекомендуемое значение – 85\)  
* `TEST` – открыть/закрыть заслонка для проверки

#### Настройки PID-регулятора

`PID`  
Настройки PID (пропорционально-интегрально-дифференциального регулятора). Данный раздел предназначен для продвинутых пользователей, и как правило не требует настройки

* `kP`  
  коэффициент усиления пропорциональной составляющей регулятора  
* `kI`  
  коэффициент усиления интегральной составляющей регулятора  
* `kD`  
  коэффициент усиления дифференциальной составляющей регулятора  
* `AUTOPID`  
  автоматическая настройка PID

### Настройка весов

#### Калибровка весов

Настройка весов производится для каждой катушки отдельно. Для настройки весов приготовьте вес ровно  в 1кг, который будет устойчиво стоять на платформе катушки. 

Перейдите в меню `SCALE -> SPOOLX -> SET`. Уберите все с платформы весов, когда это будет указано на дисплее. Затем поставьте подготовленный вес в 1кг на платформу, когда на дисплее отобразится соответствующее сообщение.

#### Тарирование весов

Если вы используете филамент на одинаковых катушках, бывает удобно настроить “тару” – массу пустой катушки, чтобы весы отображали чистую массу оставшегося филамента. Это операция не заменяет калибровку весов. 

Поставьте пустую катушку на платформу, подождите пока отобразится ее вес, и установите его в соответствующем меню `SCALE -> SPOOLX -> TARE`
 

### Проблемы и способы их решения

#### При включении сушилка отображает сообщение об ошибке и/или пищит

Нажмите один раз вращающуюся кнопку для выхода в основное меню.   
Если устройство продолжает пищать – перезагрузите его путем включения-выключения. При повторных случаях постарайтесь запомнить номер ошибки и сообщить разработчику.

#### Сушилка самопроизвольно выходит из режима сушки

Данная проблема может вызвана излишним замедлением вентилятора в режиме сушки и соответствующей нагрузкой на блок питания. Установите рекомендуемое значение в меню `SETTINGS -> AIRFLOW` в 75 или более. Если проблема сохраняется, поставьте это значение в 100\.

#### В режиме сушки вентилятор издает нездоровые звуки

Данная проблема может вызвана излишним замедлением вентилятора в режиме сушки. Установите рекомендуемое значение в меню `SETTINGS -> AIRFLOW` в 75 или более

#### Весы показывают неадекватные значения

Проверьте затяжку платформы весов к полу сушилки, и в случае неплотной затяжки подтяните крестовой отверткой. Откалибруйте весы в соответствии с инструкцией

#### Настройки не применяются, хотя я их поменял

После изменения любых настроек, находящихся под меню SETTINGS необходимо сохранить их в меню `SETTINGS -> SAVE`

#### Дисплей сушилки не гаснет сразу после выключения питания

Данное поведение является нормой


### Приложение 1: рекомендуемые параметры сушки

|  | Сушка |  | Хранение |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Температура, ℃ | Время, ч | Температура, ℃ | Влажность |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80 | 12 | 65 | 10 |
| PA-CF, Ultran | 90 2ч, затем 70 4ч | 6 | 70 | 10 |
 

### Приложение 2: структура меню

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

