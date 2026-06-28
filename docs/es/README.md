# Firmware

Solución universal para el secado de filamento. Se puede construir con materiales disponibles y económicos, por ejemplo un viejo calentador de un bloque de calor, un ventilador innecesario y un disipador de calor, y usar una caja de una tienda de electrónica y cualquier aislante como carcasa. La lista de piezas utilizadas está limitada solo por la imaginación.

Capacidades:

- Secar a temperaturas de hasta 110°C
- Secar a temperatura seleccionada durante un tiempo especificado
- Regular la potencia del calentador y mantener con precisión suficiente la temperatura del aire necesaria en el secador
- Ventilar la cámara en intervalos especificados
- Mostrar la temperatura del aire, del elemento calefactor, humedad
- Al terminar el secado, cambiar al modo de almacenamiento y mantener almacenado con un ligero calor indefinidamente
- Almacenar a temperatura seleccionada
- Al alcanzar la humedad especificada, ahorrar energía eléctrica, encender/apagar el secado según el parámetro HUMIDITY_HYSTERESIS
- Dispone de presets para plásticos de uso frecuente
- Ajustar automáticamente el PID
- Pesar el carrete y escribir el plástico restante en la pantalla
- Generar una señal alta en el pin para conectarse con la entrada del sensor de filamento de la placa de la impresora

Características del hardware:

- Control de casi cualquier elemento calefactor de 24V/220V
- Control del ventilador de circulación de la cámara por PWM
- Versión para trabajar con voltaje de 24V (también 12V) hasta la versión 4.3
- Versión para trabajar con voltaje de 220V
- Uso de módulos de balanza chinos económicos para obtener resultados de peso bastante confiables del carrete

Diseños de placa ubicados en [jlcpcb.com](https://oshwlab.com/svet_team/idryer)

Características del firmware:

- El archivo de firmware contiene todas las opciones posibles, la configuración del firmware se edita en el archivo configuration.h
- Firmware para el bootloader Arduino / no es la cosa más segura a pesar de muchas verificaciones y protecciones
  - A partir de v0.5.*, el soporte ha finalizado, la funcionalidad no se ha probado, todas las referencias en el firmware son atavismo
- Firmware para el núcleo MiniCore
  - Funciona con temporizador watchdog / en caso de falla de software reiniciará el firmware
  - Idiomas ruso e inglés
  - Regulación PID
  - Modos de secado y almacenamiento
  - Soporte para selección de frecuencias PWM del ventilador
  - Prueba de frecuencias óptimas del ventilador
  - Soporte de hasta 4 módulos de balanza en el conversor analógico-digital HX711
  - Encuesta de balanzas 10 y 80 veces por segundo
  - Pantalla de peso del carrete
  - Entrada de peso de tara en el menú
  - Señalización de fin del filamento visual, sonora, en la salida del sensor de filamento
  - [Soporte para servomotor](https://t.me/iDryer/361) para ventilación regular de la cámara
  - Nombres de presets personalizados
  - Menú expandido y funciones
  - Seguridad del firmware

!!! warning annotate "IMPORTANTE"

    :arrow_forward:</i> Durante la operación, es necesario recordar las reglas para el uso de equipos eléctricos domésticos y dispositivos de calentamiento eléctrico.

    :arrow_forward:</i> Los algoritmos utilizados no pueden garantizar una seguridad del 100% del dispositivo.


[Firmware](https://github.com/pavluchenkor/iDryerController)

[Grupo en Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribución :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
