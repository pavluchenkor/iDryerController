### Finalidade do dispositivo e informações gerais

O dispositivo é destinado à secagem e armazenamento de filamento para impressão 3D. O dispositivo é fornecido em um kit ou já montado. Se fornecido montado, já está configurado e não requer configurações específicas.

### Controle do dispositivo

#### Ligação e desligamento do dispositivo

A ligação e desligamento são feitos pelo interruptor no painel frontal. Durante a ligação, a versão atual do firmware do dispositivo é exibida; o ventilador pode ser ativado se a temperatura dentro da câmara exceder 40℃. Este comportamento é normal. Desligar o secador não requer ações prévias.

#### Menu

Todo controle do dispositivo é feito através do menu. A navegação do menu é feita usando um botão giratório (encoder).

* Girar no sentido horário - aumentar valor ou selecionar o próximo item do menu  
* Girar no sentido anti-horário - diminuir valor ou selecionar o item anterior do menu  
* Pressão breve - ativar o item selecionado do menu  
* Pressão longa - voltar ao menu anterior ou parar o modo atual (secagem ou armazenamento)

### Modo de secagem de filamento

Carregue no secador a quantidade necessária de carretilhas com filamento, opcionalmente passando o filamento por tubos PTFE se a impressão diretamente do secador for necessária. No menu DRYING, defina a temperatura (`TEMPERATURE`) e o tempo em minutos (`TIME`). Inicie o modo de secagem selecionando `START`. Se necessário, pare a secagem pressionando o encoder por tempo prolongado.

No modo de secagem, o display muda periodicamente entre a leitura da balança e os parâmetros de secagem:

* A linha superior do menu mostra a temperatura alvo e o tempo restante  
* `AIR` - temperatura do ar na câmara  
* `HEATER` - temperatura do elemento aquecedor  
* `HUMIDITY` - umidade relativa na câmara

O algoritmo de funcionamento do dispositivo no modo de secagem:

1. Aquecimento até a temperatura alvo com o ventilador ligado a 100%  
2. Manutenção da temperatura com velocidade reduzida do ventilador pelo tempo especificado  
3. Transição para modo de armazenamento de acordo com as configurações

### Modo de armazenamento de filamento

O modo de armazenamento de filamento é iniciado manualmente através do menu STORAGE ou automaticamente ao final do processo de secagem. Neste modo, o dispositivo mantém a umidade relativa alvo na câmara, permitindo começar a impressão a qualquer momento sem secagem prévia do filamento. No menu `STORAGE`, defina a umidade relativa alvo `HUMIDITY` e a temperatura máxima permitida `TEMPERATURE`. Inicie o modo de armazenamento selecionando START. Pare o modo de armazenamento pressionando o encoder por tempo prolongado.

No modo de armazenamento, o display muda periodicamente entre a leitura da balança e os parâmetros de armazenamento:

* A linha superior do menu mostra a temperatura alvo e o tempo restante  
* `AIR` - temperatura do ar na câmara  
* `HEATER` - temperatura do elemento aquecedor  
* `HUMIDITY` - umidade relativa na câmara

### 

### Configuração dos parâmetros que afetam a secagem

As configurações estão no menu `SETTINGS`. É possível alterar as configurações do fluxo de ar, a diferença entre a temperatura da câmara e a temperatura do elemento aquecedor, o obturador e o PID. Após alterar os parâmetros, é necessário salvá-los acessando `SETTINGS -> SAVE`

#### Configuração do ventilador

`AIRFLOW`  
Define a porcentagem de funcionamento do ventilador após atingir a temperatura de trabalho para reduzir o ruído do dispositivo. O valor recomendado é não menor que 70\.

#### Configuração da diferença de temperatura

`DELTA`  
Define o valor alvo da diferença entre a temperatura do elemento aquecedor e a temperatura da câmara do secador. O valor recomendado é 20\. Uma diferença muito grande pode resultar em aquecimento desigual e, por exemplo, fusão do filamento. Uma diferença muito pequena resultará em tempo de aquecimento prolongado.

#### Configurações do servo do obturador

`SETTINGS -> SERVO` 
O secador é equipado com um obturador de ventilação controlado por servo, permitindo que o ar úmido saia da câmara do secador. Durante a operação, o secador abre periodicamente o obturador para ventilação e o fecha novamente.

* `CLOSED` - tempo em minutos quando o obturador está fechado  
* `OPEN` - tempo em minutos quando o obturador está aberto  
* `CORNER` - ângulo em que o obturador se abre (valor recomendado - 85\)  
* `TEST` - abrir/fechar o obturador para teste

#### Configurações do regulador PID

`PID`  
Configurações do PID (regulador proporcional-integral-derivativo). Esta seção é destinada a usuários avançados e geralmente não requer ajustes

* `kP`  
  coeficiente de ganho da componente proporcional do regulador  
* `kI`  
  coeficiente de ganho da componente integral do regulador  
* `kD`  
  coeficiente de ganho da componente derivativa do regulador  
* `AUTOPID`  
  ajuste automático do PID

### Configuração da balança

#### Calibração da balança

A configuração da balança é feita para cada carretilha individualmente. Para calibrar a balança, prepare um peso exatamente de 1kg que será colocado de forma estável na plataforma da carretilha.

Acesse o menu `SCALE -> SPOOLX -> SET`. Remova tudo da plataforma da balança quando indicado no display. Em seguida, coloque o peso de 1kg preparado na plataforma quando a mensagem correspondente for exibida no display.

#### Tara da balança

Se você utiliza filamento em carretilhas idênticas, pode ser conveniente configurar a "tara" - a massa da carretilha vazia, para que a balança mostre apenas a massa do filamento restante. Esta operação não substitui a calibração da balança.

Coloque a carretilha vazia na plataforma, aguarde até que seu peso seja exibido e configure-o no menu correspondente `SCALE -> SPOOLX -> TARE`
 

### Problemas e soluções

#### O secador exibe uma mensagem de erro e/ou bip ao ser ligado

Pressione o botão giratório uma vez para ir ao menu principal.   
Se o dispositivo continuar a apitar - reinicie-o ligando e desligando. Se o problema se repetir, tente memorizar o número do erro e informar ao desenvolvedor.

#### O secador sai do modo de secagem sozinho

Este problema pode ser causado por desaceleração excessiva do ventilador no modo de secagem e carga correspondente na fonte de alimentação. Defina o valor recomendado no menu `SETTINGS -> AIRFLOW` para 75 ou mais. Se o problema persistir, defina este valor para 100\.

#### O ventilador faz sons anormais no modo de secagem

Este problema pode ser causado por desaceleração excessiva do ventilador no modo de secagem. Defina o valor recomendado no menu `SETTINGS -> AIRFLOW` para 75 ou mais

#### A balança mostra valores inadequados

Verifique se a plataforma da balança está bem presa ao assoalho do secador e, se não estiver, aperte com uma chave de fenda de estrela. Calibre a balança de acordo com as instruções

#### As configurações não são aplicadas, embora eu as tenha alterado

Após alterar qualquer configuração no menu SETTINGS, é necessário salvá-las no menu `SETTINGS -> SAVE`

#### O display do secador não apaga imediatamente após desligar a alimentação

Este comportamento é normal


### Apêndice 1: parâmetros de secagem recomendados

|  | Secagem |  | Armazenamento |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Temperatura, ℃ | Tempo, h | Temperatura, ℃ | Umidade |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 60-70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80-110 | 12 | 65 | 10 |
| PA-CF, Ultran | 90-110 2-4h, depois 80 4h | 6 | 70 | 10 |
 
!!! warning "Isenção de responsabilidade"
    Este apêndice fornece recomendações gerais. Antes de secar, sempre verifique os parâmetros indicados pelo fabricante do filamento e avalie o resultado pela qualidade dos itens impressos.

    Lembre-se: alguns fabricantes desonestos podem reduzir as temperaturas recomendadas na expectativa de facilidade de uso, em vez de qualidade ideal.


### Apêndice 2: estrutura do menu

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
