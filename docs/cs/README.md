# Firmware

Univerzální řešení pro sušení filamentu. Lze postavit z dostupného a levného zařízení, například starého topného prvku z heat-blocku, nepotřebného ventilátoru a chladiče, a jako skříň použít krabici z obchodu a libovolný izolační materiál. Seznam používaných součástek je omezen pouze fantazií.

Funkce:
- sušení při teplotě až do 110°C
- sušení na vybrané teplotě po zadanou dobu
- regulace výkonu topného prvku a dostatečně přesná údržba potřebné teploty vzduchu v sušičce
- větrání komory v daných intervalech
- zobrazení teploty vzduchu, topného prvku, vlhkosti
- přechod do režimu skladování po skončení sušení a skladování s mírným ohřevem neomezeně dlouho
- skladování na vybrané teplotě
- úspory energie při dosažení zadané vlhkosti, zapínání/vypínání sušení podle parametru HUMIDITY_HYSTERESIS
- obsahuje předvolby pro běžně používané plasty
- automatické ladění PID
- vážení cívky a zobrazení zbytku filamentu na displeji
- generování vysokého signálu na pinu pro připojení ke vchodu senzoru filamentu desky tiskárny

Vlastnosti hardwaru:
- řízení téměř libovolného topného prvku 24V/220V
- řízení ventilátoru komory pomocí PWM
- verze pro práci s napětím 24V (lze také 12V) do verze 4.3
- verze pro práci s napětím 220V
- použití levných čínských váhových modulů pro získání zcela spolehlivých výsledků hmotnosti cívky

Návrhy PCB jsou umístěny na [jlcpcb.com](https://oshwlab.com/svet_team/idryer)


Vlastnosti firmware:
- soubor firmware obsahuje všechny možné varianty, konfigurace firmware se upravuje v souboru configuration.h
- firmware pro bootloader Arduino / není nejbezpečnější řešení přes spoustu kontrol a ochran
  - od v0.5.* je podpora ukončena, funkčnost nebyla testována, všechny odkazy v firmware jsou anachronismy
- firmware pro jádro MiniCore
  - funguje s hlídací časovačem / v případě softwarové chyby restartuje firmware
  - ruština a angličtina
  - PID regulace
  - režimy sušení a skladování
  - podpora výběru frekvencí PWM ventilátoru
  - test optimálních frekvencí ventilátoru
  - podpora až 4 váhových modulů na ADC HX711
  - dotazování vah 10 a 80 krát za sekundu
  - výstup hmotnosti cívek na displej
  - vstup hmotnosti tára v menu
  - signalizace konce filamentu vizuální, zvuková, na výstupu senzoru filamentu
  - [podpora servopohonu](https://t.me/iDryer/361) pro pravidelné větrání komory
  - vlastní názvy předvoleb
  - rozšířené menu a funkce
  - bezpečnost firmware
  
!!! warning annotate "DŮLEŽITÉ!"

    :arrow_forward:</i> Při použití je nutné mít na paměti pravidla bezpečné manipulace s domácími elektrickými přístroji a elektrickými topnými přístroji.

    :arrow_forward:</i> Použité algoritmy nemohou zaručit 100% bezpečnost zařízení.
    
    
[Firmware](https://github.com/pavluchenkor/iDryerController)

[Telegramová skupina :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Přispívání :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
