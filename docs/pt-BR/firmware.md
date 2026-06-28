Comutação:

Conecte o Arduino Nano ao programador USBASP
(ArduinoISP não foi testado com as seções existentes do platfornio.ini, é necessário escrever manualmente) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Observe que o pinout dos conectores pode variar

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Compilação e gravação:

!!! info annotate "configuration.h"

    :arrow_forward:</i> Configure o firmware no arquivo configuration.h.
    

!!! warning annotate "Erros durante compilação"  
    Se surgirem erros durante a compilação indicando incompatibilidade entre o tamanho do firmware e a memória do microcontrolador, geralmente ajuda executar sequencialmente os comandos
    ```
    pio update
    pio pkg update
    ```

### Seleção do número de módulos de balança

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

### 1. Gravação do núcleo MiniCore

Um programador USBASP é necessário

2.1 alteração dos fusíveis


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 gravação da EEPROM

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### Gravação do microcontrolador

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### PID automático (Pid calibrate)

Executado quando necessário

Realizado em duas etapas:
Em configuration.h, defina
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = número de balanças)
```
Execute a gravação item 2.3
Após a gravação, a calibração automática do PID iniciará, e quando terminar, a mensagem "Parte 2 gravada" aparecerá na tela
mude para #define AUTOPID_RUN 0
e execute o item 2.3

### Gravação do teste do cooler

Executado quando necessário

Em configuration.h, descomente 
```
// #define PWM_TEST
```
execute o item 2.3
O teste do cooler começará em todas as frequências disponíveis com preenchimento PWM de 100-10% e modo de operação exibido na tela. Ao final do teste, com base em suas preferências de nível de ruído e qualidade de funcionamento do cooler, defina em configuration.h 
```
#define PWM_11_FREQUENCY
```
a frequência desejada, comente 
```
#define PWM_TEST
```

grave o microcontrolador item 2.3

!!! warning annotate "Erros durante inicialização"
    Se após a gravação e durante a operação erros aparecerem na tela, consulte o arquivo configuration.h


## Vídeo tutorial
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Links para o vídeo:

[VS Code](https://code.visualstudio.com/Download)

[Driver](https://zadig.akeo.ie/)

[Firmware](https://github.com/pavluchenkor/iDryerController)


[Grupo no Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contributing :material-file-edit:](https://github.com/pavluchenkor/iDryerController)

