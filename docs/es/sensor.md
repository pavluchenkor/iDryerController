## Calibración del sensor de temperatura del aire

Los sensores de temperatura del aire utilizados del tipo BME280 se caracterizan por la no linealidad de las lecturas en los extremos del rango. Para compensar esto, se utiliza un algoritmo de corrección lineal.

El algoritmo obtiene los datos de la sección configuration.h

```
#define MIN_CALIB_TEMP 70

#define MAX_CALIB_TEMP CE

#define REAL_CALIB_TEMP_MIN 70

#define REAL_CALIB_TEMP_MAX 110

```

donde: 

- MIN_CALIB_TEMP	Temperatura por debajo de la cual las lecturas no se corrigen
- MAX_CALIB_TEMP	Límite superior del rango, durante el cual se aplica la calibración
- REAL_CALIB_TEMP_MIN	Temperatura según el sensor de control de referencia en MIN_CALIB_TEMP
- REAL_CALIB_TEMP_MAX	Temperatura según el sensor de control de referencia en MAX_CALIB_TEMP

### Preparación para la prueba
Configurar el sistema con el sensor bmeTemp conectado y el termistor de control de referencia.

Asegurarse de que los parámetros de calibración estén configurados:
```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 o 110 (según la configuración)
REAL_CALIB_TEMP_MAX = 110 (según la configuración)
```
Tener la capacidad de establecer valores de temperatura precisos en la entrada 


### Pasos de prueba
Verificación de la zona sin calibración:

Establecer 65°C y verificar con el sensor de control

Repetir para valores de 70, 75, etc. Encontrar la temperatura en la que los datos del dispositivo de medición de control y del sensor incorporado comienzan a divergir, escribir esta temperatura en MIN_CALIB_TEMP y REAL_CALIB_TEMP_MIN

Verificación del límite de inicio de calibración:

- establecer la temperatura máxima igual a CE
- esperar a alcanzar la temperatura establecida
- escribir en REAL_CALIB_TEMP_MAX las lecturas del sensor de control
- guardar los cambios
- programar el controlador con el comando

```
pio run -t clean -t upload
```

## Calibración de las balanzas

### Instalación de las balanzas

Para el correcto funcionamiento de las balanzas, es necesario cumplir las siguientes condiciones:
- instalación correcta
- conmutación correcta

#### Instalación
Durante la instalación, es necesario asegurarse de que el módulo de balanzas esté separado de la placa por 1,5-2 mm y de que el compuesto no toque la placa.

<!-- #### Coммутация
Como regla general, el cable negro (-) y rojo (+) de la balanza se conectan a E+ y E- respectivamente. Los otros dos cables se conectan a A+ y A-. 
Para entender cuál va dónde, es necesario conectar los cables negro y rojo a la alimentación, cambiar el multímetro al modo de medición de voltaje continuo y conectar el cable negro (GND/-) del multímetro con el cable negro de la galga extensométrica, conectar el cable rojo a la sonda del multímetro con los cables blancos, medir el voltaje y presionar la galga extensométrica, las lecturas del multímetro cambiarán por fracciones de voltio, si cambiaron hacia arriba, este cable se conecta a A+, hacia abajo a A-. -->

#### Calibración

Retirar la bobina del módulo a calibrar. 
En el menú SCALE, seleccionar el elemento SET.
Esperar el mensaje PUT WEIGHT 1000G e instalar el peso de calibración de 1000 g en el soporte de bobina.
Repetir la operación para cada módulo.

Si la balanza muestra un peso incorrecto, verificar:
- la correctitud de la conmutación
- la correctitud del montaje
- la posición de la resistencia cero en el módulo hx711
- la temperatura del microchip ADC (no se calienta)
