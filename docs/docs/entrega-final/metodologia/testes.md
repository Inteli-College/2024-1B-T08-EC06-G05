---
title: Testes realizados
sidebar_position: 6
---

# Relatório de Testes de Usabilidade e Validação de Requisitos Não Funcionais

## Introdução

&emsp;&emsp;Este relatório documenta os testes de usabilidade e a validação dos Requisitos Não Funcionais (RNFs) do sistema de controle do Turtlebot 3 Hamburger. Os testes de usabilidade foram realizados para avaliar a experiência do usuário, enquanto os testes de RNFs verificaram aspectos técnicos críticos como tempo de resposta, usabilidade e velocidade de locomoção.

&emsp;&emsp;Outros testes foram realizados previamente e forneceram feedbacks importantes, que nos levaram a criar pop-ups e novas funcionalidades visando melhorar a usabilidade. Após a realização desses testes prévios, foi decidido conduzir mais testes para verificar se as novas funcionalidades estavam de acordo com as expectativas dos usuários e os requisitos do projeto.

## 1. Testes de Usabilidade

### Objetivo
&emsp;&emsp;Avaliar a facilidade de uso da aplicação WEB pelos usuários.

### Metodologia
&emsp;&emsp;Os testes de usabilidade foram conduzidos seguindo um roteiro estruturado. Cada etapa do processo foi analisada e as observações foram registradas. Os testes foram realizados no laboratório da faculdade Inteli, em meio a diversos membros desenvolvedores do projeto, barulhos externos e grande movimentação no ambiente. Os usuários foram brevemente contextualizados sobre os objetivos do projeto, de forma a não influenciar suas ações e pensamentos durante o teste.

### Roteiro do Teste de Usabilidade

#### 1. Tela Inicial
- **Tarefa:** O participante deve observar a tela inicial e identificar as opções disponíveis (Iniciar e Informações).
- **Objetivo:** O objetivo é entender qual vai ser a primeira ação do usuario ao se deparar com a primeira tela da solução.
- **Pergunta:** O que você faria nesta tela? Para onde você clicaria primeiro?

#### 2. Tela de Informações
- **Tarefa:** Se o participante clicar em Informações, peça para que leia e compreenda as informações fornecidas sobre a tela de controle.
- **Objetivo:** O objetivo é entender a facilidade de entendimento do usuário sobre a aplicação por meio dessa tela.
- **Pergunta:** As informações fornecidas são claras e compreensíveis?

#### 3. Iniciar Operação
- **Tarefa:** O participante deve clicar em Iniciar, digitar um nome de reboiler e clicar em Continuar.
- **Objetivo:** Verificar o fluxo de início de operação do robô.
- **Pergunta:** Houve alguma dificuldade em entender ou completar esta etapa?

#### 4. Tela de Controle
- **Tarefa:** O participante deve explorar a tela de controle, identificando os elementos (visão da câmera, setas de controle, botão de análise, botão de parada de emergência, botão de retorno ao menu).
- **Objetivo:** O objetivo é verificar o qual intuitiva a tela é, dados que os usuários podem não ter muitas habilidades com tecnologia.
- **Pergunta:** Descreva o que você vê nesta tela e o que você acha que cada elemento faz.

#### 5. Movimentação do Robô
- **Tarefa:** Utilize as setas de controle para movimentar o robô para frente, direita, esquerda e trás.
- **Objetivo:** Verificar a qualidade das funcionalidades da tela e se o tempo re resposta é aceitavel pelo usuário. 
- **Pergunta:** Os controles são responsivos e intuitivos?

#### 6. Análise de Imagem
- **Tarefa:** Clique no botão de Análise para tirar uma foto e enviar para a IA.
- **Objetivo:** A análise computacionar funciona de forma eficiente para o usuário.
- **Pergunta:** O processo de análise foi claro? A resposta da IA foi compreensível?

#### 7. Parada de Emergência
- **Tarefa:** Utilize o botão de Parada de Emergência.
- **Objetivo:** Verificar se o botão funciona de forma eficiente.
- **Pergunta:** A função de parada de emergência está bem posicionada e é fácil de acessar?

#### 8. Retorno ao Menu Inicial
- **Tarefa:** Clique no botão de Retorno ao Menu Inicial.
- **Objetivo:** Verificar fluxo de utilização da solução.
- **Pergunta:** Foi fácil retornar ao menu inicial? Você encontraria facilmente essa opção em caso de necessidade?

### Resultados dos Testes de Usabilidade

&emsp;&emsp;Para a realização dos testes de usabilidade, foram convidadas quatro pessoas de faixas etárias diferentes e com contextos de estudo e trabalho diversos. É importante notar que todos têm boas habilidades tecnológicas e contato constante com tecnologias semelhantes às utilizadas no projeto desenvolvido pela SugarZ3ro. Os convidados para a realização dos testes foram: Gabriel Coletto, André Leal, Ana Luiza e André Hutzler.

[Link para planilha de testes](https://docs.google.com/spreadsheets/d/1-YfRXwlAJLJ60PEgnCPHU68fz5dvZ41Y/edit?usp=sharing&ouid=105392408613944484717&rtpof=true&sd=true)

### Observações e Resultados 

#### Navegação
- **Tela Inicial:** As instruções na Tela Inicial são claras, e os botões são intuitivos, permitindo aos usuários iniciar ou acessar informações com facilidade. No entanto, a interação com o pop-up da tela inicial gerou uma leve preocupação entre os usuários sobre o que deveria ser feito, indicando que pequenos ajustes na familiarização com a interface podem ser benéficos.

- **Tela de Informações:**  Houve algumas dificuldades na compreensão da Tela de Informações, principalmente na funcionalidade de scroll. Isso indica a necessidade de melhorias na visibilidade ou no design dessa interação. Apesar dessas dificuldades, os usuários reconheceram que a Tela de Informações fornece explicações detalhadas e úteis sobre a tela de controle.

#### Funções
- **Controle do Robô:** O controle do robô foi considerado responsivo e fácil de usar. Todos os usuários demonstraram facilidade na movimentação do robô, incluindo a mudança de direção e a parada, sem apresentar dificuldades significativas. Isso confirma que o sistema de controle por setas está bem implementado e intuitivo.

- **Análise de Imagem:** A funcionalidade do botão de análise de imagem não foi bem compreendida pelos usuários devido à sua resposta ser apresentada apenas no banco de dados e backend. Apesar do bom funcionamento técnico, essa funcionalidade não atende plenamente aos requisitos de uma boa experiência do usuário. Melhorias na representação visual e feedback imediato são necessárias para tornar esta função mais intuitiva.

- **Parada de Emergência:** O botão de parada de emergência foi testado com sucesso em várias situações de movimentação, demonstrando ser acessível e funcional. No entanto, a necessidade de recarregar a página para retomar a movimentação não favorece um fluxo de utilização eficiente. Aperfeiçoamentos que eliminem a necessidade de recarregar a página são recomendados para melhorar a experiência do usuário.

**Conclusão:** Enquanto várias áreas da interface demonstram alta funcionalidade e usabilidade, há espaço para melhorias, especialmente na Tela de Informações e na funcionalidade de análise de imagem. Focar em aprimorar essas áreas ajudará a criar uma experiência de usuário mais fluida e intuitiva.

---

## 2. Testes de Validação de Requisitos Não Funcionais (RNFs)

### RNF1: Tempo de Resposta

#### Objetivo
Verificar o tempo de resposta da locomoção do robô e da câmera pela aplicação WEB.

#### Metodologia
Foram realizados 10 testes para medir o tempo de resposta dos comandos de locomoção e da câmera. Utilizou-se um cronômetro digital para medir o intervalo entre o envio do comando e a resposta do sistema.

#### Resultados

##### Locomoção

| Execução | Tempo de Resposta (s) |
|----------|-----------------------|
| 1        | 0.45                  |
| 2        | 0.48                  |
| 3        | 0.50                  |
| 4        | 0.47                  |
| 5        | 0.46                  |
| 6        | 0.49                  |
| 7        | 0.51                  |
| 8        | 0.44                  |
| 9        | 0.46                  |
| 10       | 0.47                  |

**Média de Tempo de Resposta da Locomoção:** 0.473 s

##### Câmera

| Execução | Tempo de Resposta (s) |
|----------|-----------------------|
| 1        | 0.72                  |
| 2        | 0.95                  |
| 3        | 1.31                  |
| 4        | 0.73                  |
| 5        | 1.4                   |
| 6        | 0.96                  |
| 7        | 0.90                  |
| 8        | 1.31                  |
| 9        | 0.53                  |
| 10       | 0.74                  |

**Média de Tempo de Resposta da Câmera:** A média do tempo de resposta é aproximadamente 0,955 segundos.

#### Análise
Os resultados indicam que o tempo médio de resposta para os comandos de locomoção e câmera está fora dos limites aceitáveis ( < 500ms), sugerindo que o sistema não atende a responsividade adequada para operação em tempo real.

---

### RNF2: Usabilidade

#### Objetivo
Avaliar a facilidade de uso da aplicação WEB pelos usuários.

#### Metodologia
Foi realizada uma autoavaliação da interface de usuário da aplicação WEB, considerando a clareza das instruções, a navegação entre telas e a usabilidade das funções principais.

#### Observações e Resultados

| Aspecto         | Comentários                                |
|-----------------|--------------------------------------------|
| Navegação       | As instruções na Tela Inicial são claras e os botões são intuitivos. A Tela de Informações fornece uma explicação detalhada e útil sobre a tela de controle. |
| Funções         | O controle do robô através das setas é responsivo e fácil de usar. A função de análise de imagem é direta, mas não fornece feedback claro. O botão de parada de emergência é facilmente acessível e funcional. |

#### Conclusão
A usabilidade da aplicação WEB é funcional, com navegação intuitiva, mas precisa de melhoras para melhor usabilidade. 

---

### RNF3: Velocidade de Locomoção

#### Objetivo
Verificar a aceleração e a velocidade máxima constante do Turtlebot durante seu funcionamento.

#### Metodologia
Foram realizados 5 testes para medir a aceleração e a velocidade máxima do Turtlebot. Utilizou-se um cronômetro digital e uma fita métrica para medir os tempos e distâncias.

#### Resultados

##### Aceleração

| Execução | Tempo para Velocidade Máxima (s) |
|----------|----------------------------------|
| 1        | 2.5                              |
| 2        | 2.4                              |
| 3        | 2.6                              |
| 4        | 2.5                              |
| 5        | 2.4                              |

**Média de Tempo para Velocidade Máxima:** 2.48 s

##### Velocidade Máxima

| Execução | Distância Percorrida (m) | Tempo (s) | Velocidade (m/s) |
|----------|---------------------------|-----------|------------------|
| 1        | 5                         | 22D        | 0.23             |
| 2        | 5                         | 21        | 0.24             |
| 3        | 5                         | 22        | 0.23             |
| 4        | 5                         | 21        | 0.24             |
| 5        | 5                         | 22        | 0.23             |

**Média de Velocidade Máxima:** 0.234 m/s

#### Análise
Os resultados mostram que o Turtlebot atinge a velocidade máxima em um tempo médio de 2.48 segundos e mantém uma velocidade constante média de 0.234 m/s, o que está dentro das especificações esperadas.

---

## Conclusão

Os testes realizados validaram os Requisitos Não Funcionais do sistema de controle do Turtlebot 3 Hamburger. O tempo de resposta, usabilidade e velocidade de locomoção atenderam aos critérios estabelecidos, indicando que o sistema é eficaz e adequado para a operação planejada. Por outro lado o tempo de resposta da câmera não atendeu o requisito e dese sobre correções em desenvolvimentos futuros pensando na melhora da usabilidade.  

---