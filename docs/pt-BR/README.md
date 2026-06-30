# Sobre o Projeto

![PCB](../img/IMG_3080.png)

Solução universal para secagem de filamento. Pode ser construída a partir de componentes acessíveis e baratos, como um antigo aquecedor de bloco, um ventilador desnecessário e um dissipador, e usar uma caixa de material de construção como gabinete e qualquer isolante. A lista de peças usadas é limitada apenas pela imaginação.

Recursos:
- secar a temperatura de até 110°C
- secar em temperatura selecionada durante tempo especificado
- regular a potência do aquecedor e manter com precisão a temperatura do ar necessária no secador
- ventilar a câmara em intervalos especificados
- exibir temperatura do ar, elemento aquecedor, umidade
- após a conclusão da secagem, mudar para modo de armazenamento e armazenar com pequeno aquecimento indefinidamente
- armazenar em temperatura selecionada
- ao atingir a umidade especificada, economizar energia elétrica, ligar/desligar secagem pelo parâmetro HUMIDITY_HYSTERESIS
- possui pressets para plásticos frequentemente usados
- ajustar PID automaticamente
- pesar o carretel e exibir o plástico restante na tela
- gerar sinal alto no pino para conexão com entrada de sensor de filamento da placa da impressora

Características do hardware:
- controle de quase qualquer elemento aquecedor 24V/220V
- controle de ventilador de sopro da câmara via PWM
- versão para funcionar com tensão de 24V (também 12V) até versão 4.3
- versão para funcionar com tensão de 220V
- uso de módulos de balança chineses baratos para obter resultados bastante confiáveis do peso do carretel

Design de placa localizado em [jlcpcb.com](https://oshwlab.com/svet_team/idryer)


Características do firmware:
- arquivo de firmware contém todas as variantes possíveis, configuração do firmware editada no arquivo configuration.h
- firmware para bootloader Arduino / não é o mais seguro apesar de muitas verificações e proteções
  - a partir de v0.5.* suporte descontinuado, funcionalidade não testada, todas as referências no firmware são resquícios
- firmware para núcleo MiniCore
  - funciona watchdog timer / em caso de falha de software, recarrega o firmware
  - idiomas russo e inglês
  - Regulação PID
  - modos de secagem e armazenamento
  - suporte para seleção de frequências PWM do ventilador
  - teste de frequências ótimas de ventilador
  - suporte para até 4 módulos de balança no ADC HX711
  - sondagem de balança 10 e 80 vezes por segundo
  - exibição do peso dos carretéis na tela
  - entrada de peso de tara no menu
  - sinalização de término do filamento visual, sonora, na saída do sensor de filamento
  - [suporte para servomotor](https://t.me/iDryer/361) para ventilação regular da câmara
  - nomes customizados de pressets
  - menu estendido e funções
  - segurança do firmware

!!! warning annotate "IMPORTANTE!"

    :arrow_forward:</i> Durante a operação, é necessário lembrar as regras de operação de equipamentos elétricos domésticos e dispositivos de aquecimento elétrico.

    :arrow_forward:</i> Os algoritmos usados não podem garantir 100% de segurança do dispositivo.


[Firmware](https://github.com/pavluchenkor/iDryerController)

[Grupo no Telegram :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribuindo :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
