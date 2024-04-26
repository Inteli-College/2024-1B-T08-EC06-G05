---
title: Definição de requisitos
sidebar_position: 1
---

Nesta seção serão abordados os requisitos funcionais e não funcionais criados. Esses requisitos são fundamentais para definir as funcionalidades e guiar a execução do projeto. Os requisitos de um sistema são essenciais para definir suas funcionalidades e guiar a execução do projeto. Os requisitos funcionais delineiam as ações fundamentais para o funcionamento do produto, enquanto os requisitos não funcionais complementam essas funcionalidades, incluindo aspectos como segurança e desempenho. Eles formam a base que orienta o desenvolvimento, garantindo uma compreensão clara do que será realizado e como as etapas do projeto serão conduzidas. Sendo assim, foram desenvolvidos os seguintes requisitos:

**1. Tabela de Requisitos Funcionais (RF) - Turtlebot para verificar limpeza:**

| Código do Requisito | Título               | Detalhes                | Método de Teste        |
|----------------------|----------------------|-------------------------|------------------------|
| RF-01                | Locomoção do robô | Por meio de botões presentes na aplicação WEB criada, o usuário deve ser capaz de teleoperar o robô, conseguindo mover o Turtlebot. | Por meio da aplicação WEB, o robô se move. Caso ele não se mova em alguma direção, o teste falhou. |
| RF-02                | Verificação da limpeza | Por meio de botões presentes na aplicação WEB criada, o usuário deve ser capaz de ordenar o robô a apontar a câmera para um tubo específico para ser feita a análise da limpeza. | Por meio da interface, o robô ativa a câmera e identifica se o tubo está sujo ou não. Caso a câmera não ative ou a câmera não reconheça se está limpo ou não, o teste falhou.  |
| RF-03                | Análise da limpeza | Por meio da inteligência artificial (IA) criada, o robô deve analisar a imagem transmitida pela câmera e retornar para o usuário (pela plataforma WEB) se aquele tubo está sujo ou não. | Por meio da interface, o robô ativa a câmera e identifica se o tubo está sujo ou não. Caso a câmera não ative ou a câmera não reconheça se está limpo ou não, o teste falhou.  |
| RF-04                | Registro de logs | O sistema deve ser capaz de guardar a informação de quais tubos estão sujos em um banco de dados PostgreSQL. | Caso as informações não sejam guardadas corretamente ou não sejam mandadas corretamente pela API, o teste falhou. |
| RF-05                | Transmissão de imagem | Por causa do robô ser teleoperado, o sistema deve usar a câmera acoplada ao Turtlebot para enviar imagens para o servidor, que serão exibidas na aplicação WEB criada. | Caso as informações não sejam guardadas corretamente ou não sejam mandadas corretamente pela API, o teste falhou. |
| RF-06                | Exposição de logs | O sistema deve ser capaz de coletar as informações do banco de dados e expor elas por meio de uma API. Por pedido da empresa parceira Atvos, não será criada uma interface para ver as informações expostas da API. | Caso as informações não sejam guardadas corretamente ou não sejam mandadas corretamente pela API, o teste falhou. |

**Fonte:** Elaborado pela equipe SugarZ3ro

**2. Tabela de Requisitos Não Funcionais (RNF) - Turtlebot para verificar limpeza:**

| Código do Requisito | Título               | Detalhes                | 
|----------------------|----------------------|-------------------------|
| RNF-01               | Tempo de funcionamento | A bateria do robô deve ser duradoura e, considerando uma bateria nova de lítio-ion ou polímero de lítio, deve durar pelo menos 9 horas interruptas e ser carregada em menos de 8 horas, a fim de minimizar a quantidade de baterias que devem ser levadas junto ao robô e não ter que trocá-la constantemente. | 
| RNF-02               | Tempo de resposta | O tempo de resposta da locomoção do robô e da câmera pela aplicação WEB deve ser menor que 200ms e os dois tempos de resposta devem ser similares, com no máximo 50ms de difença. Assim, há grande chande que a experiência do usuário se mantenha em um nível satisfatório e não note diferenças na posição verdadeira do Turtlebot e aquela vista pela imagem. |
| RNF-03               | Velocidade de locomoção | A aceleração e velocidade máxima do Turtlebot devem ser constantes durante o seu funcionamento, sendo a velocidade máxima marcada em 15m/s, assegurando assim a experiência do usuário, mantendo um padrão de uso. |
| RNF-04               | Acurácia da inteligência artificial | A inteligência artificial que analisa as imagens da câmera deve apresentar uma acurácia de no mínimo 75%, para assim evitar que haja trabalho sem necessidade de limpar algo que já esteja limpo e que deixe de limpar algo que está sujo, diminuindo a eficiência da produção |
| RNF-05               | Transito de informações | O sistema deve ser capaz de suportar um grande volume de dados sendo transmitidos simultaneamente, pois, em paralelo, o sistema estará lidando com informações de movimento, vídeo da câmera, respostas da IA e registros. Considerando o uso do ROS para comunicação entre o Turtlebot e o backend em Flask, espera-se uma carga significativa de dados. |
| RNF-05               | Armazenamento de informações | O sistema deve ser capaz de suportar um grande volume de dados guardados no banco de dados, já que são 20.000 tubos por reboiler, com aproximadamente 9 reboilers por usina. Assim sendo, o sistema deve ser capaz de suportar no mínimo 100.000 linhas de dados que guardam quais tubos estão sujos. |
| RNF-06               | Usabilidade | A interface deve ser de fácil uso e intuitiva, com uma navegabilidade satisfatória. Como método de medição, foi escolhido o  Teste de Usabilidade do Sistema (SUS), tendo um objetivo de pelo menos 70 pontos para garantir os requisitos previamente citados. |

**Fonte:** Elaborado pela equipe SugarZ3ro

Estas tabelas apresentam os requisitos funcionais (RF) e não funcionais (RNF) relacionados ao uso do robô (Turtlebot) na usina da Atvos, bem como a interface que o controla. Nota-se que para teleoperar o Turtlebot, o teleoperador deve estar conectado à mesma internet que o robô.