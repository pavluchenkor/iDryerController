### Účel zařízení a všeobecné informace

Zařízení je určeno pro sušení a skladování vláken pro 3D tisk. Zařízení se dodává buď v sadě KIT, nebo v sestavené formě. Zařízení v sestavené formě je již nakonfigurováno a nevyžaduje žádné specifické nastavení.

### Ovládání zařízení

#### Zapnutí a vypnutí zařízení

Zapnutí a vypnutí se provádí přepínačem na přední straně. Během spuštění se zobrazuje aktuální verze firmware zařízení, může se zapnout ventilátor, pokud teplota uvnitř komory překročí 40 °C. Toto chování je normální. Vypnutí sušičky nevyžaduje žádné předchozí kroky.

#### Nabídka

Veškeré ovládání zařízení se provádí přes nabídku. Navigace v nabídce se provádí pomocí rotačního tlačítka (enkodera).

* Otáčení ve směru hodinových ručiček - zvýšení hodnoty nebo výběr příští položky nabídky
* Otáčení proti směru hodinových ručiček - snížení hodnoty nebo výběr předchozí položky nabídky
* Krátké stisknutí - aktivace vybrané položky nabídky
* Dlouhé stisknutí - návrat do předchozí nabídky nebo zastavení aktuálního provozního režimu (sušení nebo skladování)

### Režim sušení vláken

Načtěte potřebné množství cívek s vláknem do sušičky, volitelně provlékněte vlákno přes PTFE trubice, pokud se předpokládá tisk přímo ze sušičky. V nabídce DRYING nastavte teplotu (`TEMPERATURE`) a čas v minutách (`TIME`). Spusťte režim sušení výběrem položky `START`. V případě potřeby zastavte sušení dlouhým stisknutím enkodera.

V režimu sušení se displej pravidelně přepíná mezi čteními z vah a parametry sušení:

* Horní řádek nabídky ukazuje cílovou teplotu a zbývající čas
* `AIR` - teplota vzduchu v komoře
* `HEATER` - teplota topného prvku
* `HUMIDITY` - relativní vlhkost v komoře

Algoritmus provozu zařízení v režimu sušení:

1. Ohřev na cílovou teplotu s ventilátor spuštěný na 100 %
2. Udržování teploty se sníženými otáčkami ventilátoru po dobu zadaného času
3. Přechod do režimu skladování podle nastavení

### Režim skladování vláken

Režim skladování vláken se spustí buď ručně přes nabídku STORAGE, nebo automaticky po ukončení procesu sušení. V tomto režimu zařízení udržuje cílovou relativní vlhkost v komoře, což umožňuje kdykoli začít tisk bez předchozího sušení vláken. V nabídce `STORAGE` nastavte cílovou relativní vlhkost `HUMIDITY` a maximální povolenou teplotu `TEMPERATURE`. Spusťte režim skladování výběrem START. Zastavte režim skladování dlouhým držením enkodera.

V režimu skladování se displej pravidelně přepíná mezi čteními z vah a parametry skladování:

* Horní řádek nabídky ukazuje cílovou teplotu a zbývající čas
* `AIR` - teplota vzduchu v komoře
* `HEATER` - teplota topného prvku
* `HUMIDITY` - relativní vlhkost v komoře

### 

### Nastavení parametrů ovlivňujících sušení

Nastavení se nachází v nabídce `SETTINGS`. Je možné změnit nastavení toku vzduchu, rozdílu mezi teplotou v komoře a teplotou topného prvku, klapky a PID. Po změně parametrů je nutné je uložit v nabídce `SETTINGS -> SAVE`

#### Nastavení ventilátoru

`AIRFLOW`
Nastavuje procento provozu ventilátoru po dosažení pracovní teploty, aby se snížil hluk zařízení. Doporučená hodnota je minimálně 70.

#### Nastavení rozdílu teplot

`DELTA`
Nastavuje cílovou hodnotu rozdílu mezi teplotou topného prvku a teplotou v komoře sušičky. Doporučená hodnota je 20. Příliš velký rozdíl může vést k nerovnoměrnému ohřevu a například k roztavení vláken. Příliš malý rozdíl vede k dlouhé době ohřevu.

#### Nastavení servopohonu vzduchovací klapky

`SETTINGS -> SERVO`
Sušička je vybavena vzduchovací klapkou se servpohonem, která umožňuje vlhkému vzduchu vycházet z komory sušičky. Během provozu sušička pravidelně otevírá a zavírá klapku.

* `CLOSED` - čas v minutách, kdy je klapka zavřena
* `OPEN` - čas v minutách, kdy je klapka otevřena
* `CORNER` - úhel, pod kterým se klapka otevírá (doporučená hodnota je 85)
* `TEST` - otevřít/zavřít klapku pro testování

#### Nastavení PID regulátoru

`PID`
Nastavení PID (proporcionálně-integrálně-derivačního regulátoru). Tato sekce je určena pro pokročilé uživatele a obvykle nevyžaduje nastavení

* `kP`
  koeficient zesílení proporcionální složky regulátoru
* `kI`
  koeficient zesílení integrální složky regulátoru
* `kD`
  koeficient zesílení derivační složky regulátoru
* `AUTOPID`
  automatické nastavení PID

### Nastavení vah

#### Kalibrace vah

Nastavení vah se provádí pro každou cívku zvlášť. Pro nastavení vah si připravte přesně 1 kg váhy, která bude stabilně stát na platformě cívky.

Přejděte do nabídky `SCALE -> SPOOLX -> SET`. Odstraňte vše z platformy vah, když bude na displeji uvedeno. Poté položte připravenou 1 kg váhu na platformu, když se na displeji zobrazí odpovídající zpráva.

#### Vynulování vah

Pokud používáte vlákno na stejných cívkách, je někdy užitečné nastavit "taru" - hmotnost prázdné cívky, aby váhy zobrazovaly čistou hmotnost zbývajícího vláken. Tato operace nenahrazuje kalibraci vah.

Položte prázdnou cívku na platformu, počkejte, až se zobrazí její hmotnost, a nastavte ji v příslušné nabídce `SCALE -> SPOOLX -> TARE`


### Problémy a způsoby jejich řešení

#### Při spuštění sušička zobrazuje chybovou zprávu a/nebo pípá

Stiskněte jednou otočné tlačítko pro návrat do hlavní nabídky.
Pokud zařízení stále pípá - restartujte jej vypnutím a zapnutím. V případě opakovaných výskytů si zapamatujte číslo chyby a informujte vývojáře.

#### Sušička se samovolně vypne z režimu sušení

Tento problém může být způsoben nadměrným zpomalením ventilátoru v režimu sušení a odpovídající zátěží na zdroj napájení. V nabídce `SETTINGS -> AIRFLOW` nastavte doporučenou hodnotu na 75 nebo více. Pokud problém přetrvává, nastavte tuto hodnotu na 100.

#### V režimu sušení ventilátor vydává zvláštní zvuky

Tento problém může být způsoben nadměrným zpomalením ventilátoru v režimu sušení. V nabídce `SETTINGS -> AIRFLOW` nastavte doporučenou hodnotu na 75 nebo více

#### Váhy zobrazují neadekvátní hodnoty

Zkontrolujte utažení platformy vah k podlaze sušičky a v případě volného utažení ji utáhněte křížovým šroubovákem. Zkalibrujte váhy podle pokynů

#### Nastavení se nepoužívají, i když jsem je změnil(a)

Po změně jakýchkoli nastavení umístěných v nabídce SETTINGS je nutné je uložit v nabídce `SETTINGS -> SAVE`

#### Displej sušičky se nevypne ihned po vypnutí napájení

Toto chování je normální


### Příloha 1: doporučené parametry sušení

|  | Sušení |  | Skladování |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Teplota, °C | Čas, h | Teplota, °C | Vlhkost |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 60-70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80-110 | 12 | 65 | 10 |
| PA-CF, Ultran | 90-110 2-4h, pak 80 4h | 6 | 70 | 10 |
 
!!! warning "Zřeknutí se odpovědnosti"
    Tato příloha poskytuje obecná doporučení. Před sušením se vždy poraďte s parametry určenými výrobcem vláken a vyhodnoťte výsledek podle kvality vytištěných předmětů.

    Pamatujte: někteří bezohlední výrobci mohou snižovat doporučené teploty s cílem jednoduché použití, nikoli s cílem na optimální kvalitu.


### Příloha 2: struktura nabídky

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
