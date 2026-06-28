### Destination de l'appareil et informations générales

L'appareil est conçu pour sécher et stocker le filament pour l'impression 3D. L'appareil est fourni soit sous forme de kit, soit entièrement assemblé. L'appareil entièrement assemblé est déjà configuré et ne nécessite aucun réglage spécifique.

### Contrôle de l'appareil

#### Allumage et extinction de l'appareil

L'allumage et l'extinction se font à l'aide du commutateur situé sur le panneau avant. Lors du démarrage, la version actuelle du micrologiciel de l'appareil s'affiche, et le ventilateur peut s'activer si la température à l'intérieur de la chambre dépasse 40℃. Ce comportement est normal. L'arrêt du sécheur ne nécessite aucune action préalable.

#### Menu

Tous les contrôles de l'appareil se font via le menu. La navigation dans le menu s'effectue à l'aide d'un bouton rotatif (encodeur).

* Rotation dans le sens horaire - augmentation de la valeur ou sélection de l'élément de menu suivant
* Rotation dans le sens antihoraire - diminution de la valeur ou sélection de l'élément de menu précédent
* Appui court - activation de l'élément de menu sélectionné
* Appui long - retour au menu précédent ou arrêt du mode de fonctionnement actuel (séchage ou stockage)

### Mode de séchage du filament

Chargez le nombre nécessaire de bobines de filament dans le sécheur, en enfilant optionnellement le filament dans des tubes PTFE si vous envisagez d'imprimer à partir du sécheur. Dans le menu DRYING, définissez la température (`TEMPERATURE`) et le temps en minutes (`TIME`). Lancez le mode séchage en sélectionnant `START`. Si nécessaire, arrêtez le séchage en appuyant longtemps sur l'encodeur.

En mode séchage, l'affichage bascule périodiquement entre les lectures de la balance et les paramètres de séchage :

* La ligne supérieure du menu indique la température cible et le temps restant
* `AIR` - température de l'air dans la chambre
* `HEATER` - température de l'élément chauffant
* `HUMIDITY` - humidité relative dans la chambre

Algorithme de fonctionnement de l'appareil en mode séchage :

1. Chauffage jusqu'à la température cible avec le ventilateur fonctionnant à 100%
2. Maintien de la température avec le ventilateur à régime réduit pour la durée spécifiée
3. Transition vers le mode de stockage conformément aux paramètres

### Mode de stockage du filament

Le mode de stockage du filament est lancé soit manuellement via le menu STORAGE, soit automatiquement après la fin du processus de séchage. Dans ce mode, l'appareil maintient l'humidité relative cible dans la chambre, ce qui permet de commencer à imprimer à tout moment sans séchage préalable du filament. Dans le menu `STORAGE`, définissez l'humidité relative cible `HUMIDITY` et la température maximale autorisée `TEMPERATURE`. Lancez le mode de stockage en sélectionnant START. Arrêtez le mode de stockage en appuyant longtemps sur l'encodeur.

En mode stockage, l'affichage bascule périodiquement entre les lectures de la balance et les paramètres de stockage :

* La ligne supérieure du menu indique la température cible et le temps restant
* `AIR` - température de l'air dans la chambre
* `HEATER` - température de l'élément chauffant
* `HUMIDITY` - humidité relative dans la chambre

###

### Configuration des paramètres affectant le séchage

Les paramètres se trouvent dans le menu `SETTINGS`. Il est possible de modifier les paramètres du flux d'air, la différence entre la température de la chambre et la température de l'élément chauffant, le volet et le PID. Après modification des paramètres, vous devez les enregistrer en accédant à `SETTINGS -> SAVE`

#### Configuration du ventilateur

`AIRFLOW`
Définit le pourcentage de fonctionnement du ventilateur après atteinte de la température de fonctionnement pour réduire le bruit de l'appareil. La valeur recommandée est d'au moins 70.

#### Configuration de la différence de température

`DELTA`
Définit la valeur cible de la différence entre la température de l'élément chauffant et la température dans la chambre de séchage. La valeur recommandée est 20. Une différence trop grande peut entraîner un chauffage inégal et, par exemple, la fusion du filament. Une différence trop faible prolongera le temps de chauffage.

#### Configuration du servomoteur du volet

`SETTINGS -> SERVO`
Le sécheur est équipé d'un volet de ventilation avec servomoteur, ce qui permet à l'air humide de s'échapper de la chambre de séchage. Pendant le fonctionnement, le sécheur ouvre périodiquement le volet pour aérer et le referme.

* `CLOSED` - temps en minutes pendant lequel le volet est fermé
* `OPEN` - temps en minutes pendant lequel le volet est ouvert
* `CORNER` - angle d'ouverture du volet (valeur recommandée - 85)
* `TEST` - ouvrir/fermer le volet pour test

#### Configuration du régulateur PID

`PID`
Paramètres du PID (régulateur proportionnel-intégral-différentiel). Cette section est destinée aux utilisateurs avancés et ne nécessite généralement pas de configuration

* `kP`
  coefficient de gain de la composante proportionnelle du régulateur
* `kI`
  coefficient de gain de la composante intégrale du régulateur
* `kD`
  coefficient de gain de la composante différentielle du régulateur
* `AUTOPID`
  réglage automatique du PID

### Configuration de la balance

#### Étalonnage de la balance

La configuration de la balance se fait pour chaque bobine séparément. Pour configurer la balance, préparez un poids d'exactement 1 kg qui sera stable sur la plateforme de la bobine.

Accédez au menu `SCALE -> SPOOLX -> SET`. Enlevez tout de la plateforme de la balance quand cela s'affiche sur l'écran. Puis placez le poids préparé de 1 kg sur la plateforme quand le message correspondant s'affiche.

#### Tare de la balance

Si vous utilisez du filament sur des bobines identiques, il est pratique de configurer la "tare" - la masse de la bobine vide, afin que la balance affiche la masse nette du filament restant. Cette opération ne remplace pas l'étalonnage de la balance.

Placez une bobine vide sur la plateforme, attendez que son poids s'affiche, et réglez-le dans le menu correspondant `SCALE -> SPOOLX -> TARE`


### Problèmes et solutions

#### Au démarrage, le sécheur affiche un message d'erreur et/ou émet un bip

Appuyez une fois sur le bouton rotatif pour accéder au menu principal.
Si l'appareil continue à émettre des bips - redémarrez-le en l'éteignant et en l'allumant. En cas de répétitions, essayez de retenir le numéro d'erreur et signalez-le au développeur.

#### Le sécheur quitte spontanément le mode de séchage

Ce problème peut être causé par un ralentissement excessif du ventilateur en mode séchage et une charge correspondante sur l'alimentation électrique. Réglez la valeur recommandée dans le menu `SETTINGS -> AIRFLOW` à 75 ou plus. Si le problème persiste, réglez cette valeur à 100.

#### En mode séchage, le ventilateur émet des bruits anormaux

Ce problème peut être causé par un ralentissement excessif du ventilateur en mode séchage. Réglez la valeur recommandée dans le menu `SETTINGS -> AIRFLOW` à 75 ou plus

#### La balance affiche des valeurs inadéquates

Vérifiez le serrage de la plateforme de la balance au sol du sécheur, et en cas de serrage insuffisant, resserrez-la avec un tournevis cruciforme. Étalonez la balance conformément aux instructions

#### Les paramètres ne s'appliquent pas bien que je les aie modifiés

Après modification de tous les paramètres situés sous le menu SETTINGS, vous devez les enregistrer via le menu `SETTINGS -> SAVE`

#### L'écran du sécheur ne s'éteint pas immédiatement après la coupure de l'alimentation

Ce comportement est normal


### Appendice 1 : paramètres de séchage recommandés

|  | Séchage |  | Stockage |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Température, ℃ | Temps, h | Température, ℃ | Humidité |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 60-70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80-110 | 12 | 65 | 10 |
| PA-CF, Ultran | 90-110 2-4h, puis 80 4h | 6 | 70 | 10 |

!!! warning "Exonération de responsabilité"
    Cet appendice fournit des recommandations générales. Avant le séchage, vérifiez toujours les paramètres spécifiés par le fabricant du filament et évaluez le résultat en fonction de la qualité des pièces imprimées.

    Rappelez-vous : certains fabricants peu scrupuleux peuvent sous-estimer les températures recommandées pour des raisons de simplicité d'utilisation plutôt que pour une qualité optimale.


### Appendice 2 : structure du menu

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