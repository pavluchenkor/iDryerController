Connexions :

Connectez Arduino Nano au programmateur USBASP
(ArduinoISP n'a pas été testé avec les sections existantes de platformio.ini, vous devez écrire vos propres configurations)
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

Notez que la broche de connecteurs peut différer

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

Compilation et programmation :

!!! info annotate "configuration.h"

    :arrow_forward:</i> Configurez le micrologiciel dans le fichier configuration.h.
    

!!! warning annotate "Erreurs de compilation"  
    Si des erreurs de compilation apparaissent indiquant une incompatibilité entre la taille du firmware et la mémoire du microcontrôleur, l'exécution successive des commandes suivantes aide généralement
    ```
    pio update
    pio pkg update
    ```

### Sélection du nombre de modules de balance

=== "2 modules"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3 modules"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4 modules"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. Programmation du noyau MiniCore

Un programmateur USBASP est nécessaire

2.1 changement des fusibles


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 Programmation de l'EEPROM

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### Programmation du microcontrôleur

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### PID automatique (Pid calibrate)

Effectué si nécessaire

Le processus se déroule en deux étapes :
Définissez dans configuration.h
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = nombre de balances)
```
Effectuez la programmation selon la section 2.3
Après la programmation, l'ajustement automatique du PID commencera, et à la fin, le message "Proshei chast 2" apparaîtra à l'écran
remplacez par #define AUTOPID_RUN 0
et effectuez la section 2.3

### Test du ventilateur de programmation

Effectué si nécessaire

Dans configuration.h, décommentez
```
// #define PWM_TEST
```
effectuez la section 2.3
Le test du ventilateur commencera à toutes les fréquences disponibles avec un remplissage PWM de 100-10 % et une sortie du mode de fonctionnement sur l'écran. À la fin du test, en fonction de vos préférences concernant le niveau de bruit et la qualité du fonctionnement du ventilateur, définissez dans configuration.h
```
#define PWM_11_FREQUENCY
```
la fréquence souhaitée, décommentez
```
#define PWM_TEST
```

programmez le microcontrôleur selon la section 2.3

!!! warning annotate "Erreurs au démarrage"
    Si des erreurs apparaissent sur l'écran après la programmation et pendant le fonctionnement, veuillez consulter le fichier configuration.h


## Vidéo pédagogique
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
Liens vers la vidéo :

[VS Code](https://code.visualstudio.com/Download)

[Pilote](https://zadig.akeo.ie/)

[Micrologiciel](https://github.com/pavluchenkor/iDryerController)


[Groupe Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribution :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
