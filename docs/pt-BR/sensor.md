## Calibração do sensor de temperatura do ar

Os sensores de temperatura do ar tipo BME280 utilizados apresentam não-linearidade nas leituras nas extremidades da faixa. Para compensar isso, é utilizado um algoritmo de correção linear.

Este algoritmo obtém dados da seção configuration.h

```
#define MIN_CALIB_TEMP 70

#define MAX_CALIB_TEMP CE

#define REAL_CALIB_TEMP_MIN 70

#define REAL_CALIB_TEMP_MAX 110

```

onde: 

- MIN_CALIB_TEMP	Temperatura abaixo da qual as leituras não são corrigidas
- MAX_CALIB_TEMP	Limite superior da faixa na qual a calibração é aplicada
- REAL_CALIB_TEMP_MIN	Temperatura do sensor de referência na MIN_CALIB_TEMP
- REAL_CALIB_TEMP_MAX	Temperatura do sensor de referência na MAX_CALIB_TEMP

### Preparação para o teste
Configure o sistema com o sensor bmeTemp conectado e um termistor de referência padrão.

Certifique-se de que os parâmetros de calibração foram definidos:
```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 ou 110 (conforme a configuração)
REAL_CALIB_TEMP_MAX = 110 (conforme a configuração)
```
Tenha a capacidade de definir valores de temperatura precisos na entrada 


### Passos de teste
Verificação da zona sem calibração:

Defina 65°C e verifique com o sensor de referência

Repita para valores de 70, 75, etc. Encontre a temperatura em que os dados do dispositivo de medição de referência e do sensor integrado começarem a divergir, insira esta temperatura em MIN_CALIB_TEMP e REAL_CALIB_TEMP_MIN

Verificação do limite de início da calibração:

- defina a temperatura máxima igual a CE
- aguarde a estabilização na temperatura definida
- insira em REAL_CALIB_TEMP_MAX as leituras do sensor de referência
- salve as alterações
- carregue o controlador com o comando

```
pio run -t clean -t upload
```

## Calibração da balança

### Instalação da balança

Para o funcionamento adequado da balança, as seguintes condições devem ser observadas:
- instalação adequada
- conexão adequada

#### Instalação
Durante a instalação, certifique-se de que o módulo da balança está a 1,5-2 mm da placa e que o composto não toca a placa.

<!-- #### Conexão
Como regra, o fio preto (-) e vermelho (+) da balança são conectados a E+ e E- respectivamente. Os outros dois fios são conectados a A+ e A-. 
Para determinar qual é qual, conecte os fios preto e vermelho à alimentação, coloque um multímetro no modo de medição de tensão CC e conecte o fio preto (GND/-) do multímetro ao fio preto do sensor de tensão, conecte o fio vermelho à ponta de prova do multímetro com os fios brancos, meça a tensão e pressione o sensor de tensão, as leituras do multímetro mudarão de frações de volt, se aumentarem, este fio é conectado a A+, se diminuírem - A-. -->

#### Calibração

Remova o carretel do módulo a ser calibrado. 
No menu SCALE, selecione a opção SET.
Aguarde a mensagem PUT WEIGHT 1000G e coloque o peso de calibração de 1000g no suporte do carretel.
Repita a operação para cada módulo.

Se a balança mostrar peso incorreto, verifique:
- a correção da conexão
- a correção da fixação
- a posição do resistor de zero no módulo hx711
- a temperatura do chip do conversor AD (não deve estar aquecido)
