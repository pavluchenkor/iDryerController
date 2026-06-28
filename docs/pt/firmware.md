Conexão:

Conecte o Arduino Nano ao programador USBASP
(ArduinoISP não foi testado com as seções existentes do platformio.ini, será necessário configurar manualmente) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Observe que a pinagem dos conectores pode variar

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Compilação e gravação:

!!! info annotate "configuration.h"

    :arrow_forward:</i> Configure a firmware no arquivo configuration.h.
    

!!! warning annotate "Erros de compilação"  
    Se ocorrerem erros de compilação indicando incompatibilidade entre o tamanho da firmware e a memória do microcontrolador, geralmente ajuda executar sequencialmente os comandos
    ```
    pio update
    pio pkg update
    ```

### Seleção da quantidade de módulos de pesagem

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

### 1. Gravação do core MiniCore

Será necessário um programador USBASP

2.1 Alteração de fusíveis


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 Gravação da EEPROM

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

### Autopid (calibração PID)

Executada quando necessário

O procedimento ocorre em duas etapas:
Em configuration.h configure
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = quantidade de pesagens)
```
Execute a gravação da seção 2.3
Após a gravação, a calibração automática de PID iniciará e ao concluir exibirá "Firmware parte 2" na tela
altere para #define AUTOPID_RUN 0
e execute a seção 2.3

### Gravação do teste de cooler

Executada quando necessário

Em configuration.h descomente 
```
// #define PWM_TEST
```
execute a seção 2.3
O teste do cooler executará em todas as frequências disponíveis com preenchimento PWM de 100-10% e exibirá o modo de operação na tela. Ao final do teste, com base em suas preferências de nível de ruído e qualidade de funcionamento do cooler, defina em configuration.h 
```
#define PWM_11_FREQUENCY
```
a frequência desejada, comente 
```
#define PWM_TEST
```

grave o microcontrolador na seção 2.3

!!! warning annotate "Erros na inicialização"
    Se após a gravação e durante a operação erros aparecerem na tela, consulte o arquivo configuration.h


## Vídeo educativo
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Links do vídeo:

[VS Code](https://code.visualstudio.com/Download)

[Driver](https://zadig.akeo.ie/)

[Firmware](https://github.com/pavluchenkor/iDryerController)


[Grupo no Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribuindo :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
