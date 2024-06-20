---
title: Testes realizados
sidebar_position: 6
---

# Relatório de Testes de Usabilidade e Validação de Requisitos Não Funcionais

## Introdução

Este relatório documenta os testes de usabilidade e a validação dos Requisitos Não Funcionais (RNFs) do sistema de controle do Turtlebot 3 Hamburger. Os testes de usabilidade foram realizados para avaliar a experiência do usuário, enquanto os testes de RNFs verificaram aspectos técnicos críticos como tempo de resposta, usabilidade e velocidade de locomoção.

## 1. Testes de Usabilidade

### Objetivo
Avaliar a facilidade de uso da aplicação WEB pelos usuários.

### Metodologia
Os testes de usabilidade foram conduzidos seguindo um roteiro estruturado. Cada etapa do processo foi analisada e observações foram registradas.

### Roteiro do Teste de Usabilidade

#### 1. Tela Inicial
- **Tarefa:** O participante deve observar a tela inicial e identificar as opções disponíveis (Iniciar e Informações).
- **Pergunta:** O que você faria nesta tela? Para onde você clicaria primeiro?

#### 2. Tela de Informações
- **Tarefa:** Se o participante clicar em Informações, peça para que leia e compreenda as informações fornecidas sobre a tela de controle.
- **Pergunta:** As informações fornecidas são claras e compreensíveis?

#### 3. Iniciar Operação
- **Tarefa:** O participante deve clicar em Iniciar, digitar um nome de reboiler e clicar em Continuar.
- **Pergunta:** Houve alguma dificuldade em entender ou completar esta etapa?

#### 4. Tela de Controle
- **Tarefa:** O participante deve explorar a tela de controle, identificando os elementos (visão da câmera, setas de controle, botão de análise, botão de parada de emergência, botão de retorno ao menu).
- **Pergunta:** Descreva o que você vê nesta tela e o que você acha que cada elemento faz.

#### 5. Movimentação do Robô
- **Tarefa:** Utilize as setas de controle para movimentar o robô para frente, direita, esquerda e trás.
- **Pergunta:** Os controles são responsivos e intuitivos?

#### 6. Análise de Imagem
- **Tarefa:** Clique no botão de Análise para tirar uma foto e enviar para a IA.
- **Pergunta:** O processo de análise foi claro? A resposta da IA foi compreensível?

#### 7. Parada de Emergência
- **Tarefa:** Utilize o botão de Parada de Emergência.
- **Pergunta:** A função de parada de emergência está bem posicionada e é fácil de acessar?

#### 8. Retorno ao Menu Inicial
- **Tarefa:** Clique no botão de Retorno ao Menu Inicial.
- **Pergunta:** Foi fácil retornar ao menu inicial? Você encontraria facilmente essa opção em caso de necessidade?

### Resultados dos Testes de Usabilidade

[Link para panilha de testes]()

### Observações e Resultados 

#### Navegação
- **Tela Inicial:** As instruções na Tela Inicial são claras, com botões intuitivos para iniciar ou acessar informações.
- **Tela de Informações:** A Tela de Informações fornece explicações detalhadas e úteis sobre a tela de controle.

#### Funções
- **Controle do Robô:** O controle do robô através das setas é responsivo e fácil de usar.
- **Análise de Imagem:** A função de análise de imagem é direta e fornece feedback claro sobre a condição do tubo.
- **Parada de Emergência:** O botão de parada de emergência é facilmente acessível e funcional.

**Conclusão:**
A aplicação WEB é intuitiva e fácil de usar, com navegação clara e funções bem implementadas. Não foram encontradas dificuldades significativas durante a autoavaliação.

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
| 1        | 0.32                  |
| 2        | 0.35                  |
| 3        | 0.31                  |
| 4        | 0.33                  |
| 5        | 0.34                  |
| 6        | 0.36                  |
| 7        | 0.30                  |
| 8        | 0.31                  |
| 9        | 0.33                  |
| 10       | 0.34                  |

**Média de Tempo de Resposta da Câmera:** 0.33 s

#### Análise
Os resultados indicam que o tempo médio de resposta para os comandos de locomoção e câmera está dentro dos limites aceitáveis (<500ms), sugerindo que o sistema é responsivo e adequado para operação em tempo real.

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
| Funções         | O controle do robô através das setas é responsivo e fácil de usar. A função de análise de imagem é direta e fornece feedback claro. O botão de parada de emergência é facilmente acessível e funcional. |

#### Conclusão
A usabilidade da aplicação WEB é satisfatória, com navegação intuitiva e funções facilmente utilizáveis. 

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
| 1        | 5                         | 22        | 0.23             |
| 2        | 5                         | 21        | 0.24             |
| 3        | 5                         | 22        | 0.23             |
| 4        | 5                         | 21        | 0.24             |
| 5        | 5                         | 22        | 0.23             |

**Média de Velocidade Máxima:** 0.234 m/s

#### Análise
Os resultados mostram que o Turtlebot atinge a velocidade máxima em um tempo médio de 2.48 segundos e mantém uma velocidade constante média de 0.234 m/s, o que está dentro das especificações esperadas.

---

## Conclusão

Os testes realizados validaram os Requisitos Não Funcionais do sistema de controle do Turtlebot 3 Hamburger. O tempo de resposta, usabilidade e velocidade de locomoção atenderam aos critérios estabelecidos, indicando que o sistema é eficaz e adequado para a operação planejada.

---