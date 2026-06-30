# Sobre o Projeto

![PCB](../img/IMG_3080.png)

Solução universal para secagem de filamento. Pode ser construída com componentes acessíveis e baratos, como um antigo aquecedor de bloco de calor, um ventilador desnecessário e um dissipador de calor. A carcaça pode ser uma caixa de ferragens e qualquer isolante térmico. A lista de componentes utilizados é limitada apenas pela imaginação.

Capacidades:
- secar em temperaturas de até 110°C
- secar em temperatura selecionada por tempo determinado
- regular a potência do aquecedor e manter com precisão a temperatura do ar necessária na câmara
- ventilar a câmara em intervalos especificados
- exibir temperatura do ar, elemento aquecedor e umidade
- após conclusão da secagem, mudar para modo de armazenamento e armazenar com leve aquecimento indefinidamente
- armazenar em temperatura selecionada
- ao atingir a umidade especificada, economizar energia, ligando/desligando a secagem pelo parâmetro HUMIDITY_HYSTERESIS
- possuir predefinições para plásticos frequentemente utilizados
- ajustar automaticamente o controlador PID
- pesar o carretel e exibir o plástico restante na tela
- gerar um sinal alto no pino para conexão com a entrada do sensor de filamento da placa da impressora

Características do hardware:
- controle de praticamente qualquer elemento aquecedor de 24V/220V
- controle do ventilador de sopro da câmara por PWM
- versão para operação com tensão de 24V (também pode ser 12V) até a versão 4.3
- versão para operação com tensão de 220V
- uso de módulos de balança chineses acessíveis para obter resultados de peso bastante confiáveis do carretel

Designs da placa estão localizados em [jlcpcb.com](https://oshwlab.com/svet_team/idryer)

Características do firmware:
- arquivo de firmware contém todas as variações possíveis, a configuração do firmware é editada no arquivo configuration.h
- firmware para bootloader Arduino / não é a coisa mais segura apesar de muitas verificações e proteções
  - a partir de v0.5.* suporte foi descontinuado, a funcionalidade não foi testada, todas as referências no firmware são atavismo
- firmware baseado no núcleo MiniCore
  - funciona com temporizador de watchdog / em caso de falha do software, reiniciará o firmware
  - idiomas russo e inglês
  - controle PID
  - modos de secagem e armazenamento
  - suporte para seleção de frequências PWM do ventilador
  - teste de frequências ótimas do ventilador
  - suporte para até 4 módulos de balança no ADC HX711
  - consulta de peso 10 e 80 vezes por segundo
  - exibição do peso dos carretéis na tela
  - entrada do peso da tara no menu
  - alarme de falta de filamento visual, sonoro e na saída do sensor de filamento
  - [suporte para servo](https://t.me/iDryer/361) para ventilação regular da câmara
  - nomes personalizados de predefinições
  - menu estendido e funções
  - segurança do firmware

!!! warning annotate "IMPORTANTE!"

    :arrow_forward:</i> Durante a operação, é necessário lembrar das regras de uso de equipamentos elétricos domésticos e aquecedores elétricos.

    :arrow_forward:</i> Os algoritmos utilizados não podem garantir 100% de segurança do dispositivo.

[Firmware](https://github.com/pavluchenkor/iDryerController)

[Grupo no Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribuindo :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
