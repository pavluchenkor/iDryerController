## Calibração do sensor de temperatura do ar

Os sensores de temperatura do ar do tipo BME280 utilizados apresentam não-linearidade nas leituras nas extremidades da faixa. Para compensar isto, utiliza-se um algoritmo de correção linear.

O algoritmo obtém os dados da seção configuration.h

```
#define MIN_CALIB_TEMP 70

#define MAX_CALIB_TEMP CE

#define REAL_CALIB_TEMP_MIN 70

#define REAL_CALIB_TEMP_MAX 110

```

onde: 

- MIN_CALIB_TEMP	Temperatura abaixo da qual as leituras não são corrigidas
- MAX_CALIB_TEMP	Limite superior da faixa no qual a calibração é aplicada
- REAL_CALIB_TEMP_MIN	Temperatura do sensor de referência no MIN_CALIB_TEMP
- REAL_CALIB_TEMP_MAX	Temperatura do sensor de referência no MAX_CALIB_TEMP

### Preparação para o teste
Configure o sistema com o sensor bmeTemp conectado e um termistor de referência padrão.

Certifique-se de que os parâmetros de calibração estão definidos:
```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 ou 110 (dependendo da configuração)
REAL_CALIB_TEMP_MAX = 110 (dependendo da configuração)
```
Tenha a capacidade de definir valores de temperatura precisos na entrada 


### Etapas de teste
Verificação da zona sem calibração:

Defina 65°C e verifique com o sensor de referência

Repita para valores de 70, 75, etc. Encontre a temperatura em que os dados do dispositivo de medição de referência e do sensor integrado começam a divergir, insira esta temperatura em MIN_CALIB_TEMP e REAL_CALIB_TEMP_MIN

Verificação do limite de início da calibração:

- defina a temperatura máxima igual a CE
- aguarde a estabilização na temperatura definida
- insira em REAL_CALIB_TEMP_MAX as leituras do sensor de referência
- salve as alterações
- programar o controlador com o comando

```
pio run -t clean -t upload
```

## Calibração das células de carga

### Instalação das células de carga

Para o funcionamento correto das células de carga, as seguintes condições devem ser atendidas:
- instalação correta
- conexão correta

#### Instalação
Durante a instalação, certifique-se de que o módulo da célula de carga está afastado 1,5-2mm da placa e que o composto não toca a placa.

<!-- #### Conexão
Geralmente os fios preto (-) e vermelho (+) das células de carga são conectados a E+ e E- respectivamente. Os outros dois fios são conectados a A+ e A-. 
Para entender qual vai onde, é necessário conectar os fios preto e vermelho à fonte de alimentação, mudar o multímetro para modo de medição de tensão contínua e conectar o fio preto (GND/-) do multímetro ao fio preto da célula de carga, conectar a sonda do multímetro ao fio branco, medir a tensão e pressionar na célula de carga, as leituras do multímetro mudarão algumas frações de volt, se aumentarem, este fio é conectado a A+, se diminuírem - a A-. -->

#### Calibração

Remova o carretel do módulo a ser calibrado. 
No menu SCALE, selecione SET.
Aguarde a mensagem PUT WEIGHT 1000G e coloque o peso de calibração de 1000g no suporte de carretel.
Repita a operação para cada módulo.

Se a célula de carga mostrar peso incorreto, verifique:
- correção das conexões
- correção da fixação
- posição do resistor de zeragem no módulo hx711
- temperatura do chip ADC (não deve estar superaquecido)
