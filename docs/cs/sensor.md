## Kalibrace senzoru teploty vzduchu

Používané senzory teploty vzduchu typu BME280 se vyznačují nelinearitou měření na okrajích rozsahu. K vykompenzování tohoto jevu se používá algoritmus lineární korekce.

Algoritmus získává data ze sekce configuration.h

```
#define MIN_CALIB_TEMP 70

#define MAX_CALIB_TEMP CE

#define REAL_CALIB_TEMP_MIN 70

#define REAL_CALIB_TEMP_MAX 110

```

kde:

- MIN_CALIB_TEMP	Teplota, pod kterou nejsou údaje korigovány
- MAX_CALIB_TEMP	Horní hranice rozsahu, při které se aplikuje kalibrace
- REAL_CALIB_TEMP_MIN	Teplota podle kontrolního senzoru při MIN_CALIB_TEMP
- REAL_CALIB_TEMP_MAX	Teplota podle kontrolního senzoru při MAX_CALIB_TEMP

### Příprava na test
Nastavte systém s připojeným senzorem bmeTemp a referenčním kontrolním termistorem.

Ujistěte se, že parametry kalibrace jsou nastaveny:
```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 nebo 110 (v závislosti na konfiguraci)
REAL_CALIB_TEMP_MAX = 110 (v závislosti na konfiguraci)
```
Mít možnost nastavit přesné hodnoty teploty na vstupu


### Kroky testování
Ověření zóny bez kalibrace:

Nastavte 65°C a zkontrolujte podle kontrolního senzoru

Opakujte pro hodnoty 70, 75 atd. Najděte teplotu, při které se údaje kontrolního měřidla a vestavěného senzoru začnou lišit, zadejte tuto teplotu do MIN_CALIB_TEMP a REAL_CALIB_TEMP_MIN

Ověření hranice zahájení kalibrace:

- nastavte maximální teplotu rovnou CE
- čekejte na dosažení nastavené teploty
- zadejte do REAL_CALIB_TEMP_MAX údaje kontrolního senzoru
- uložte změny
- naprogramujte řadič příkazem

```
pio run -t clean -t upload
```

## Kalibrace vah

### Instalace vah

Aby váhy fungovaly správně, musí být splněny následující podmínky:
- správná instalace
- správné zapojení

#### Instalace
Při instalaci se ujistěte, že modul vah je umístěn ve vzdálenosti 1,5-2 mm od desky a sloučenina se nedotýká desky.

<!-- #### Zapojení
Obvykle má váha černý drát (-), červený drát (+) připojený k E+ a E- v tomto pořadí. Zbývající dva vodiče jsou připojeny k A+ a A-. 
Chcete-li zjistit, který kam patří, připojte černý a červený drát k napájení, přepněte multimetr do režimu měření stejnosměrného napětí a spojte černý drát (GND/-) multimetru s černým drátem tenzosenzoru, červený drát spojte se sondou multimetru s bílým drátem, změřte napětí a zatlačte na tenzosenzor, údaje multimetru se změní o zlomky voltu, pokud se zvýšily, připojte tento drát k A+, pokud se snížily - k A-. -->

#### Kalibrace

Odstraňte cívku z kalibrovaného modulu. 
V nabídce SCALE vyberte položku SET.
Čekejte na nápis PUT WEIGHT 1000G a umístěte kalibrační váhu 1000g na držák cívky.
Opakujte operaci pro každý modul.

Pokud váhy ukazují nesprávnou hmotnost, zkontrolujte:
- správnost zapojení
- správnost montáže
- polohu nulového rezistoru na modulu hx711
- teplotu čipu ADC (bez přehřívání)
