### Finalidade do dispositivo e informações gerais

O dispositivo é destinado à secagem e armazenamento de filamento para impressão 3D. O dispositivo é fornecido como um kit ou montado. O dispositivo montado já está configurado e não requer ajustes específicos.

### Controle do dispositivo

#### Ativação e desativação do dispositivo

A ativação e desativação são feitas por meio de um botão de alternância no painel frontal. Durante a ativação, a versão atual do firmware do dispositivo é exibida; o ventilador pode ser ligado se a temperatura dentro da câmara exceder 40℃. Este comportamento é normal. Desligar o secador não requer ações prévias.

#### Menu

Todo o controle do dispositivo é feito através do menu. A navegação pelo menu é feita usando um botão rotativo (codificador).

* Rotação no sentido horário - aumentar o valor ou selecionar o próximo item do menu
* Rotação no sentido anti-horário - diminuir o valor ou selecionar o item anterior do menu
* Pressão breve - ativar o item selecionado do menu
* Pressão prolongada - voltar ao menu anterior ou parar o modo de operação atual (secagem ou armazenamento)

### Modo de secagem de filamento

Carregue a quantidade necessária de carretilhas com filamento no secador e, opcionalmente, alimente o filamento nos tubos PTFE, se pretende imprimir do secador. No menu DRYING, defina a temperatura (`TEMPERATURE`) e o tempo em minutos (`TIME`). Inicie o modo de secagem selecionando `START`. Se necessário, pare a secagem pressionando o codificador por mais tempo.

No modo de secagem, o display alterna periodicamente entre as leituras das balanças e os parâmetros de secagem:

* A linha superior do menu indica a temperatura alvo e o tempo restante
* `AIR` - temperatura do ar na câmara
* `HEATER` - temperatura do elemento aquecedor
* `HUMIDITY` - umidade relativa na câmara

O algoritmo de operação do dispositivo no modo de secagem:

1. Aquecimento até a temperatura alvo com ventilador funcionando a 100%
2. Manutenção da temperatura com ventilador em velocidade reduzida durante o tempo especificado
3. Transição para o modo de armazenamento de acordo com as configurações

### Modo de armazenamento de filamento

O modo de armazenamento de filamento é iniciado manualmente através do menu STORAGE ou automaticamente ao término do processo de secagem. Neste modo, o dispositivo mantém a umidade relativa alvo na câmara, permitindo começar a imprimir a qualquer momento sem secagem prévia do filamento. No menu `STORAGE`, defina a umidade relativa alvo `HUMIDITY` e a temperatura máxima permitida `TEMPERATURE`. Inicie o modo de armazenamento selecionando START. Pare o modo de armazenamento pressionando o codificador por mais tempo.

No modo de armazenamento, o display alterna periodicamente entre as leituras das balanças e os parâmetros de armazenamento:

* A linha superior do menu indica a temperatura alvo e o tempo restante
* `AIR` - temperatura do ar na câmara
* `HEATER` - temperatura do elemento aquecedor
* `HUMIDITY` - umidade relativa na câmara

### 

### Configuração de parâmetros que afetam a secagem

As configurações estão no menu `SETTINGS`. É possível alterar as configurações de fluxo de ar, a diferença entre a temperatura da câmara e a temperatura do elemento aquecedor, a abertura da válvula e o PID. Após alterar os parâmetros, é necessário salvá-los chamando `SETTINGS -> SAVE`

#### Configuração do ventilador

`AIRFLOW`  
Define a porcentagem de operação do ventilador após atingir a temperatura de funcionamento para reduzir o ruído do dispositivo. O valor recomendado é de no mínimo 70.

#### Configuração da diferença de temperatura

`DELTA`  
Define o valor alvo da diferença entre a temperatura do elemento aquecedor e a temperatura na câmara do secador. O valor recomendado é 20. Uma diferença muito grande pode levar ao aquecimento irregular e, por exemplo, ao derretimento do filamento. Uma diferença muito pequena pode levar a um tempo de aquecimento prolongado.

#### Configurações da válvula servo

`SETTINGS -> SERVO` 
O secador é equipado com uma válvula de ventilação com servo, que permite que o ar úmido saia da câmara do secador. Durante a operação, o secador periodicamente abre e fecha a válvula de ventilação.

* `CLOSED` - tempo em minutos quando a válvula está fechada
* `OPEN` - tempo em minutos quando a válvula está aberta
* `CORNER` - ângulo em que a válvula se abre (valor recomendado - 85)
* `TEST` - abrir/fechar a válvula para teste

#### Configurações do regulador PID

`PID`  
Configurações do PID (regulador proporcional-integral-derivativo). Esta seção é destinada a usuários avançados e geralmente não requer ajuste

* `kP`  
  coeficiente de ganho da componente proporcional do regulador
* `kI`  
  coeficiente de ganho da componente integral do regulador
* `kD`  
  coeficiente de ganho da componente derivativa do regulador
* `AUTOPID`  
  ajuste automático do PID

### Configuração das balanças

#### Calibração das balanças

As balanças são configuradas individualmente para cada carretilha. Para configurar as balanças, prepare um peso de exatamente 1 kg que fique estável na plataforma da carretilha.

Vá ao menu `SCALE -> SPOOLX -> SET`. Remova tudo da plataforma das balanças quando a mensagem aparecer no display. Em seguida, coloque o peso preparado de 1 kg na plataforma quando a mensagem apropriada aparecer no display.

#### Tara das balanças

Se você usar filamento em carretilhas idênticas, pode ser conveniente configurar uma "tara" - a massa da carretilha vazia, para que a balança mostre a massa líquida do filamento restante. Esta operação não substitui a calibração das balanças.

Coloque uma carretilha vazia na plataforma, aguarde a exibição do seu peso e defina-o no menu correspondente `SCALE -> SPOOLX -> TARE`
 

### Problemas e suas soluções

#### Ao ligar, o secador exibe uma mensagem de erro e/ou emite um bip

Pressione o botão rotativo uma vez para ir ao menu principal.   
Se o dispositivo continuar a emitir bips - reinicie-o ligando e desligando. Em casos repetidos, tente lembrar o número do erro e informar ao desenvolvedor.

#### O secador sai do modo de secagem espontaneamente

Este problema pode ser causado por uma redução excessiva do ventilador no modo de secagem e pela carga correspondente na fonte de alimentação. Defina o valor recomendado no menu `SETTINGS -> AIRFLOW` para 75 ou superior. Se o problema persistir, defina esse valor para 100.

#### No modo de secagem, o ventilador emite sons anormais

Este problema pode ser causado por uma redução excessiva do ventilador no modo de secagem. Defina o valor recomendado no menu `SETTINGS -> AIRFLOW` para 75 ou superior

#### A balança mostra valores inadequados

Verifique o aperto da plataforma da balança ao piso do secador e, se estiver solto, aperte com uma chave de fenda de cruzeta. Calibre a balança de acordo com as instruções

#### As configurações não são aplicadas, embora eu as tenha alterado

Após alterar qualquer configuração no menu SETTINGS, é necessário salvá-las no menu `SETTINGS -> SAVE`

#### O display do secador não apaga imediatamente após desligar a alimentação

Este comportamento é normal


### Apêndice 1: parâmetros recomendados de secagem

|  | Secagem |  | Armazenamento |  |
| :---- | ----- | ----- | ----- | ----- |
|  | Temperatura, ℃ | Tempo, h | Temperatura, ℃ | Umidade |
| PLA | 55 | 8 | 50 | 15 |
| PETG | 65 | 8 | 60 | 15 |
| TPU | 60-70 | 8 | 60 | 15 |
| ABS/ASA | 85 | 8 | 70 | 15 |
| PA | 80-110 | 12 | 65 | 10 |
| PA-CF, Ultran | 90-110 2-4h, depois 80 4h | 6 | 70 | 10 |
 
!!! warning "Aviso legal"
    Este apêndice fornece recomendações gerais. Antes de secar, certifique-se de consultar os parâmetros especificados pelo fabricante do filamento e avalie o resultado pela qualidade dos produtos impressos.

    Lembre-se: alguns fabricantes inescrupulosos podem subestimar as temperaturas recomendadas visando facilidade de uso, não qualidade ideal.


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
