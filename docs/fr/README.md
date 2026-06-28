# Firmware

Solution universelle pour sécher le filament. Il est possible de l'assembler à partir de composants disponibles et peu coûteux, par exemple un ancien radiateur de bloc d'impression, un ventilateur inutilisé et un dissipateur thermique, et d'utiliser une boîte d'un magasin de bricolage et un isolant quelconque comme boîtier. La liste des pièces utilisées n'est limitée que par l'imagination.

Capacités :
- sécher à une température jusqu'à 110°C ;
- sécher à la température sélectionnée pendant une durée spécifiée ;
- réguler la puissance du radiateur et maintenir suffisamment précisément la température d'air requise dans le sécheur ;
- ventiler la chambre à intervalles réguliers spécifiés ;
- afficher la température de l'air, de l'élément chauffant, l'humidité ;
- passer en mode stockage à la fin du séchage et stocker avec un léger chauffage indéfiniment ;
- stocker à la température sélectionnée ;
- économiser l'électricité lors de l'atteinte de l'humidité spécifiée, allumer/éteindre le séchage selon le paramètre HUMIDITY_HYSTERESIS ;
- dispose de présets pour les plastiques couramment utilisés ;
- configurer automatiquement le PID ;
- peser la bobine et afficher la quantité restante de filament à l'écran ;
- générer un signal élevé sur la broche pour connecter à l'entrée du capteur de filament de la carte de l'imprimante.

Caractéristiques matérielles :
- contrôle de pratiquement n'importe quel élément chauffant 24V/220V ;
- contrôle du ventilateur de soufflage de chambre par PWM ;
- version pour fonctionnement avec une tension de 24V (peut aussi être 12V) jusqu'à la version 4.3 ;
- version pour fonctionnement avec une tension de 220V ;
- utilisation de modules de balance chinois bon marché pour obtenir des résultats de pesée fiables ;
les conceptions PCB sont hébergées sur [jlcpcb.com](https://oshwlab.com/svet_team/idryer)


Caractéristiques du firmware :
- le fichier de firmware contient toutes les variantes possibles, la configuration du firmware est éditée dans le fichier configuration.h ;
- firmware pour le bootloader Arduino / pas très sûr malgré de nombreuses vérifications et protections
  - à partir de v0.5.*, le support est terminé, la fonctionnalité n'a pas été testée, toutes les références du firmware sont des anachronismes ;
- firmware pour le noyau MiniCore
  - fonctionne avec le minuteur de surveillance / en cas de défaillance logicielle, redémarrera le firmware ;
  - langues russe et anglaise ;
  - régulation PID ;
  - modes séchage et stockage ;
  - support de la sélection des fréquences PWM du ventilateur ;
  - test des fréquences optimales du ventilateur ;
  - support jusqu'à 4 modules de balance sur l'ADC HX711 ;
  - interrogation de la balance 10 et 80 fois par seconde ;
  - affichage du poids de la bobine à l'écran ;
  - entrée du poids de la tare dans le menu ;
  - signalisation visuelle et sonore de la fin du filament, sortie sur le capteur de filament ;
  - [support du servo-moteur](https://t.me/iDryer/361) pour ventiler régulièrement la chambre ;
  - noms de présélections personnalisés ;
  - menu étendu et fonctions ;
  - sécurité du firmware ;
  
!!! warning annotate "IMPORTANT !"

    :arrow_forward:</i> Lors de l'utilisation, il est nécessaire de se souvenir des règles de manipulation des appareils électroménagers et des appareils chauffants électriques.

    :arrow_forward:</i> Les algorithmes utilisés ne peuvent pas garantir une sécurité à 100% de l'appareil.
    
    
[Firmware](https://github.com/pavluchenkor/iDryerController)

[groupe Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contributing :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
