## Étalonnage du capteur de température de l'air

Les capteurs de température de l'air utilisés de type BME280 se distinguent par une non-linéarité des lectures aux extrémités de la plage. Pour compenser cela, un algorithme de correction linéaire est appliqué.

L'algorithme reçoit les données de la section configuration.h

```
#define MIN_CALIB_TEMP 70

#define MAX_CALIB_TEMP CE

#define REAL_CALIB_TEMP_MIN 70

#define REAL_CALIB_TEMP_MAX 110

```

où : 

- MIN_CALIB_TEMP	Température en dessous de laquelle les lectures ne sont pas corrigées
- MAX_CALIB_TEMP	Limite supérieure de la plage à laquelle l'étalonnage est appliqué
- REAL_CALIB_TEMP_MIN	Température selon le capteur de contrôle à MIN_CALIB_TEMP
- REAL_CALIB_TEMP_MAX	Température selon le capteur de contrôle à MAX_CALIB_TEMP

### Préparation au test
Configurez le système avec le capteur bmeTemp connecté et un thermistor de contrôle de référence.

Assurez-vous que les paramètres d'étalonnage sont définis :
```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 ou 110 (selon la configuration)
REAL_CALIB_TEMP_MAX = 110 (selon la configuration)
```
Avoir la possibilité de définir des valeurs de température précises à l'entrée 


### Étapes de test
Vérification de la zone sans étalonnage :

Définissez 65°C et vérifiez avec le capteur de contrôle

Répétez pour les valeurs 70, 75, etc. Trouvez la température à laquelle les données du capteur de mesure de contrôle et du capteur intégré commencent à diverger, entrez cette température dans MIN_CALIB_TEMP et REAL_CALIB_TEMP_MIN

Vérification de la limite de début d'étalonnage :

- définir la température maximale égale à CE
- attendre la stabilisation à la température définie
- entrer les lectures du capteur de contrôle dans REAL_CALIB_TEMP_MAX
- enregistrer les modifications
- flasher le contrôleur avec la commande

```
pio run -t clean -t upload
```

## Étalonnage de la balance

### Installation de la balance

Pour le fonctionnement correct de la balance, les conditions suivantes doivent être respectées :
- installation correcte
- commutation correcte

#### Installation
Lors de l'installation, assurez-vous que le module de balance est à distance de la plaque de 1,5 à 2 mm et que le composé ne touche pas la plaque.

<!-- #### Коммутация
Как правило у весов черный провод(-), красный(+) подключаются к E+ и E- соответственно. Остальные два провода подключаются к A+ и А-. 
Чтобы понять какой куда, необходимо подключить черный и красный провода к питанию, перевести мультиметр в режим измерения постоянного напряжения и соединить черный провод(GND/-) мультиметра с черным проводом тензодатчика, красный провод соединить щуп мультиметра с белым проводам, измерить напряжение и надавить на тензодатчик, показания мультиметра изменятся на доли вольта, если изменились вверх, этот провод подключается к А+, вниз - А-. -->

#### Étalonnage

Retirez la bobine du module à étalonner. 
Dans le menu SCALE, sélectionnez l'option SET.
Attendez le message PUT WEIGHT 1000G et placez le poids d'étalonnage de 1000g sur le porte-bobine.
Répétez l'opération pour chaque module.

Si la balance indique un poids incorrect, vérifiez :
- la correction de la commutation
- la correction du montage
- la position de la résistance zéro du module hx711
- la température de la puce CAN (ne doit pas chauffer)
