### Propósito del dispositivo e información general

El dispositivo está diseñado para secar y almacenar filamento para impresión 3D. El dispositivo se suministra en forma de kit o ya montado. El dispositivo montado ya está configurado y no requiere configuraciones específicas.

### Control del dispositivo

#### Encendido y apagado del dispositivo

El encendido y apagado se realiza mediante el interruptor en el panel frontal. Durante el encendido se muestra la versión actual del firmware del dispositivo, el ventilador puede encenderse si la temperatura dentro de la cámara supera 40℃. Este comportamiento es normal. El apagado del secador no requiere acciones previas.

#### Menú

Todo el control del dispositivo se realiza a través del menú. La navegación por el menú se realiza mediante un botón rotatorio (codificador).

* Giro en el sentido de las agujas del reloj: aumento de valor o selección del siguiente elemento del menú
* Giro en sentido contrario a las agujas del reloj: disminución de valor o selección del elemento anterior del menú
* Presión breve: activación del elemento de menú seleccionado
* Presión prolongada: volver al menú anterior o detener el modo de funcionamiento actual (secado o almacenamiento)

### Modo de secado de filamento

Cargue la cantidad necesaria de carretes de filamento en el secador, opcionalmente pasando el filamento a través de tubos de PTFE si se planea imprimir desde el secador. En el menú DRYING, configure la temperatura (`TEMPERATURE`) y el tiempo en minutos (`TIME`). Inicie el modo de secado seleccionando el elemento `START`. Si es necesario, detenga el secado presionando prolongadamente el codificador.

En modo de secado, la pantalla cambia periódicamente entre las lecturas de la escala y los parámetros de secado:

* La línea superior del menú indica la temperatura objetivo y el tiempo restante
* `AIR`: temperatura del aire en la cámara
* `HEATER`: temperatura del elemento calefactor
* `HUMIDITY`: humedad relativa en la cámara

Algoritmo de funcionamiento del dispositivo en modo de secado:

1. Calentamiento a la temperatura objetivo con el ventilador encendido al 100%
2. Mantenimiento de temperatura con velocidad reducida del ventilador durante el tiempo especificado
3. Transición al modo de almacenamiento de acuerdo con la configuración

### Modo de almacenamiento de filamento

El modo de almacenamiento de filamento se inicia manualmente a través del menú STORAGE o automáticamente al finalizar el proceso de secado. En este modo, el dispositivo mantiene la humedad relativa objetivo en la cámara, lo que permite comenzar la impresión en cualquier momento sin necesidad de secar previamente el filamento. En el menú `STORAGE`, configure la humedad relativa objetivo `HUMIDITY` y la temperatura máxima permitida `TEMPERATURE`. Inicie el modo de almacenamiento seleccionando START. Detenga el modo de almacenamiento presionando prolongadamente el codificador.

En modo de almacenamiento, la pantalla cambia periódicamente entre las lecturas de la escala y los parámetros de almacenamiento:

* La línea superior del menú indica la temperatura objetivo y el tiempo restante
* `AIR`: temperatura del aire en la cámara
* `HEATER`: temperatura del elemento calefactor
* `HUMIDITY`: humedad relativa en la cámara

### 

### Configuración de parámetros que afectan el secado

La configuración se encuentra en el menú `SETTINGS`. Es posible cambiar la configuración del flujo de aire, la diferencia entre la temperatura de la cámara y la temperatura del elemento calefactor, la compuerta y el PID. Después de cambiar los parámetros, es necesario guardarlos yendo a `SETTINGS -> SAVE`

#### Configuración del ventilador

`AIRFLOW`  
Establece el porcentaje de funcionamiento del ventilador después de alcanzar la temperatura de trabajo para reducir el ruido del dispositivo. El valor recomendado es no menos de 70\.

#### Configuración de la diferencia de temperatura

`DELTA`  
Establece el valor objetivo de la diferencia entre la temperatura del elemento calefactor y la temperatura en la cámara del secador. El valor recomendado es 20\. Una diferencia demasiado grande puede resultar en calentamiento desigual y, por ejemplo, fusión del filamento. Una diferencia demasiado pequeña resultará en un tiempo de calentamiento prolongado.

#### Configuración del servomotor de la compuerta de ventilación

`SETTINGS -> SERVO` 
El secador está equipado con una compuerta de ventilación con servomotor, lo que permite que el aire húmedo salga de la cámara del secador. Durante la operación, el secador abre periódicamente la compuerta de ventilación y luego la cierra.

* `CLOSED`: tiempo en minutos cuando la compuerta está cerrada
* `OPEN`: tiempo en minutos cuando la compuerta está abierta
* `CORNER`: ángulo en el que se abre la compuerta (valor recomendado: 85\)
* `TEST`: abrir/cerrar la compuerta para verificación

#### Configuración del regulador PID

`PID`  
Configuración del PID (regulador proporcional-integral-derivativo). Esta sección está destinada a usuarios avanzados y generalmente no requiere ajuste

* `kP`  
  coeficiente de ganancia del componente proporcional del regulador
* `kI`  
  coeficiente de ganancia del componente integral del regulador
* `kD`  
  coeficiente de ganancia del componente derivativo del regulador
* `AUTOPID`  
  ajuste automático del PID

### Configuración de la balanza

#### Calibración de la balanza

La configuración de la balanza se realiza para cada carrete individualmente. Para configurar la balanza, prepare un peso exacto de 1 kg que se mantenga estable en la plataforma del carrete.

Vaya al menú `SCALE -> SPOOLX -> SET`. Retire todo de la plataforma de la balanza cuando se indique en la pantalla. Luego coloque el peso preparado de 1 kg en la plataforma cuando aparezca el mensaje correspondiente en la pantalla.

#### Taraje de la balanza

Si utiliza filamento en carretes idénticos, a menudo es conveniente configurar la "tara": la masa del carrete vacío, para que la balanza muestre la masa neta del filamento restante. Esta operación no reemplaza la calibración de la balanza.

Coloque un carrete vacío en la plataforma, espere a que se muestre su peso, y configure este valor en el menú correspondiente `SCALE -> SPOOLX -> TARE`
 

### Problemas y soluciones

#### El secador muestra un mensaje de error al encender y/o emite un sonido

Presione el botón rotatorio una vez para ir al menú principal.   
Si el dispositivo continúa emitiendo sonidos: reinicie el dispositivo apagándolo y encendiéndolo. En casos repetidos, intente recordar el número de error e informe al desarrollador.

#### El secador sale espontáneamente del modo de secado

Este problema puede ser causado por una reducción excesiva de la velocidad del ventilador en modo de secado y la correspondiente carga en la fuente de alimentación. Configure el valor recomendado en el menú `SETTINGS -> AIRFLOW` a 75 o superior. Si el problema persiste, configure este valor a 100\.

#### En modo de secado, el ventilador emite sonidos poco saludables

Este problema puede ser causado por una reducción excesiva de la velocidad del ventilador en modo de secado. Configure el valor recomendado en el menú `SETTINGS -> AIRFLOW` a 75 o superior

#### La balanza muestra valores inadecuados

Verifique la sujeción de la plataforma de la balanza al piso del secador y, si no está firmemente asegurada, apriétela con un destornillador de cruz. Calibre la balanza de acuerdo con las instrucciones

#### La configuración no se aplica aunque la haya cambiado

Después de cambiar cualquier configuración en el menú SETTINGS, es necesario guardarla en el menú `SETTINGS -> SAVE`

#### La pantalla del secador no se apaga inmediatamente después de desconectar la alimentación

Este comportamiento es normal


### Apéndice 1: parámetros de secado recomendados

|  | Secado |  | Almacenamiento |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Temperatura, ℃ | Tiempo, h | Temperatura, ℃ | Humedad |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 60-70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80-110 | 12 | 65 | 10 |
| PA-CF, Ultran | 90-110 2-4ч, затем 80 4ч | 6 | 70 | 10 |
 
!!! warning "Limitación de responsabilidad"
    Este apéndice proporciona recomendaciones generales. Antes de secar, consulte obligatoriamente los parámetros indicados por el fabricante del filamento y evalúe el resultado por la calidad de los productos impresos.

    Recuerde: algunos fabricantes sin escrúpulos pueden subestimar las temperaturas recomendadas en favor de la simplicidad de uso en lugar de la calidad óptima.


### Apéndice 2: estructura del menú

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
