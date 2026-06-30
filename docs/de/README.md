# Über das Projekt

![PCB](../img/IMG_3080.png)

Eine universelle Lösung zum Trocknen von Filamenten. Sie können es aus kostengünstigen und verfügbaren Komponenten zusammenbauen – beispielsweise aus einem alten Heizblockheizer, einem unnötigen Lüfter und einem Kühlkörper. Als Gehäuse können Sie eine einfache Kiste und eine beliebige Isolierung verwenden. Die Liste der verwendeten Komponenten ist nur durch Ihre Fantasie begrenzt.

Fähigkeiten:
- Trocknen bei Temperaturen bis 110°C
- Trocknen bei einer gewählten Temperatur für eine festgelegte Zeit
- Regulierung der Heizleistung und genaue Beibehaltung der erforderlichen Lufttemperatur in der Trocknungskammer
- Belüftung der Kammer in festgelegten Zeitintervallen
- Anzeige von Lufttemperatur, Heizungstemperatur und Luftfeuchte
- Übergang in den Speichermodus nach dem Trocknen und unbegrenzte Lagerung mit leichter Erwärmung
- Speicherung bei einer gewählten Temperatur
- Energieeinsparung bei Erreichen einer eingestellten Luftfeuchte, Ein-/Ausschalten des Trocknens nach dem Parameter HUMIDITY_HYSTERESIS
- Voreinstellungen für häufig verwendete Kunststoffe
- Automatische PID-Einstellung
- Wiegen der Spule und Anzeige des verbleibenden Kunststoffs auf dem Bildschirm
- Erzeugung eines hohen Signals auf dem Pin zur Verbindung mit dem Filament-Sensor-Eingang der Druckerplatine

Hardware-Besonderheiten:
- Steuerung von fast jedem Heizelement 24V/220V
- PWM-Steuerung des Kammer-Lüfters
- Version für 24V-Betrieb (auch 12V möglich) bis Version 4.3
- Version für 220V-Betrieb
- Verwendung günstiger chinesischer Wägemodule für zuverlässige Spulengewichtsmessungen
Platinen-Designs befinden sich auf [jlcpcb.com](https://oshwlab.com/svet_team/idryer)


Firmware-Besonderheiten:
- Die Firmware-Datei enthält alle möglichen Varianten, die Firmware-Konfiguration wird in der Datei configuration.h bearbeitet
- Firmware für Arduino Bootloader / nicht die sicherste, trotz vieler Überprüfungen und Schutzmaßnahmen
  - ab v0.5.* wird die Unterstützung beendet, die Funktionalität wurde nicht getestet, alle Hinweise in der Firmware sind Überbleibsel
- Firmware für MiniCore Kernel
  - Watch Dog Timer funktioniert / bei Softwarefehlern wird die Firmware neu gestartet
  - Russisch und Englisch
  - PID-Regelung
  - Trocknungs- und Speichermodi
  - Unterstützung für die Auswahl von PWM-Lüfterfrequenzen
  - Test für optimale Lüfterfrequenzen
  - Unterstützung für bis zu 4 Wägemodule an ADC HX711
  - Gewichtsabfrage 10 und 80 mal pro Sekunde
  - Anzeige des Spulengewichts auf dem Bildschirm
  - Eingabe des Tarengewichts im Menü
  - Filament-Ende-Warnung visuell, akustisch und am Filament-Sensor-Ausgang
  - [Servo-Antrieb-Unterstützung](https://t.me/iDryer/361) für regelmäßige Kammerlüftung
  - Benutzerdefinierte Presetnamen
  - Erweitertes Menü und Funktionen
  - Firmware-Sicherheit
  
!!! warning annotate "WICHTIG!"

    :arrow_forward:</i> Bei der Bedienung müssen die Sicherheitsvorschriften für Haushaltsgeräte und elektrische Heizgeräte beachtet werden.

    :arrow_forward:</i> Die verwendeten Algorithmen können keine 100%ige Gerätesicherheit garantieren.
    
    
[Firmware](https://github.com/pavluchenkor/iDryerController)

[Telegram-Gruppe :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Beitragen :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
