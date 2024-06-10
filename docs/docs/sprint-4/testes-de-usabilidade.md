---
title: Testes de Usabilidade
sidebar_position: 2
---

# Documentação de Testes do Projeto Turtlebot

## 1. Introdução

&emsp;&emsp;Esse documento descreve os testes realizados para validar a funcionalidade e os requisitos não funcionais do projeto Turtlebot de verificação de limpeza. Os testes incluem a verificação da teleoperação do robô, tempo de resposta, usabilidade da aplicação web e velocidade de locomoção do robô. Por questão de tempo, não foi testado a inteligência artificial, pois ainda não estava 

## 2. Teste de Funcionalidade

### Objetivo
&emsp;&emsp;Verificar a capacidade de teleoperação do Turtlebot para mover-se e apontar a câmera para lugares específicos.

### Roteiro de Teste
1. Inicializar o ambiente de testes.
2. Acessar a aplicação WEB e utilizar os botões de teleoperação para mover o Turtlebot.
3. Apontar a câmera do robô para diferentes lugares específicos.
4. Verificar se a imagem transmitida da câmera corresponde à visualização esperada.

### Resultados

#### Maurício Azevedo
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Inicializar o ambiente de controle**: Entendeu a página inicial e a página de informação.
  - **Acessar a aplicação e utilizar os botões**: Navegou pela aplicação web e utilizou os botões de teleoperação.
  - **Apontar a câmera para pontos específicos**: Movimentou a câmera para diferentes tubos sem dificuldade.
  - **Verificar a imagem transmitida**: Confirmou que a imagem correspondia às expectativas, embora a latência estivesse um pouco alta.

#### Fernando Machado
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Inicializar o ambiente de controle**: Foi para a página de informação, onde entendeu o conteúdo, logo depois foi para a tela de comandos do robô.
  - **Acessar a aplicação e utilizar os botões**: Navegou intuitivamente pela aplicação web.
  - **Apontar a câmera para pontos específicos**: Movimentou a câmera para diferentes tubos com precisão.
  - **Verificar a imagem transmitida**: Observou que a imagem transmitida estava correta, mas destacou que a latência era alta.

#### Rafaela
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Inicializar o ambiente de controle**: Foi para a página de informação, onde entendeu o conteúdo, logo depois foi para a tela de comandos do robô.
  - **Acessar a aplicação e utilizar os botões**: Navegou intuitivamente pela aplicação web.
  - **Apontar a câmera para pontos específicos**: Movimentou a câmera para diferentes tubos com precisão.
  - **Verificar a imagem transmitida**: Observou que a imagem transmitida estava correta, mas destacou que a latência era alta.

#### Andrea
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Inicializar o ambiente de controle**: Foi para a página de informação, onde não havia percebido que era possível descer a tela mas ainda sim entendeu o conteúdo.
  - **Acessar a aplicação e utilizar os botões**: Navegou intuitivamente pela aplicação web.
  - **Apontar a câmera para pontos específicos**: Movimentou a câmera para diferentes tubos com precisão.
  - **Verificar a imagem transmitida**: Observou que a imagem transmitida estava correta, mas destacou que a latência era alta.

## 3. Testes de Validação de Requisitos Não Funcionais (RNFs)

### RNF1: Tempo de Resposta

#### Objetivo
&emsp;&emsp;Verificar o tempo de resposta da locomoção do robô e da câmera pela aplicação WEB.

#### Roteiro de Teste
1. Enviar comandos de locomoção para o robô.
2. Medir o tempo de resposta do robô ao comando.
3. Medir o tempo de resposta da câmera ao comando de mudança de posição.
4. Comparar os tempos medidos.

### Resultados

#### Maurício Azevedo
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Enviar comandos de locomoção ao robô**: Enviou os comandos corretamente.
  - **Medir o tempo de resposta do robô**: O tempo de resposta foi menor que 200ms.
  - **Medir o tempo de resposta da câmera**: Mediu o tempo de resposta da câmera e encontrou latência negativa.
  - **Comparar resultados**: Observou que a latência da câmera precisava de ajustes.

#### Fernando Machado
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Enviar comandos de locomoção ao robô**: Enviou os comandos corretamente.
  - **Medir o tempo de resposta do robô**: O tempo de resposta foi menor que 200ms.
  - **Medir o tempo de resposta da câmera**: Mediu o tempo de resposta da câmera e encontrou latência negativa.
  - **Comparar resultados**: Observou que a latência da câmera precisava de ajustes.

#### Rafaela
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Enviar comandos de locomoção ao robô**: Enviou os comandos corretamente.
  - **Medir o tempo de resposta do robô**: O tempo de resposta foi menor que 200ms.
  - **Medir o tempo de resposta da câmera**: Mediu o tempo de resposta da câmera e encontrou latência positiva.
  - **Comparar resultados**: Ajustou a latência da câmera para um desempenho consideravelmente melhor.

#### Andrea
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Enviar comandos de locomoção ao robô**: Enviou os comandos corretamente.
  - **Medir o tempo de resposta do robô**: O tempo de resposta foi menor que 200ms.
  - **Medir o tempo de resposta da câmera**: Mediu o tempo de resposta da câmera e encontrou latência positiva.
  - **Comparar resultados**: Ajustou a latência da câmera para um desempenho consideravelmente melhor.

### RNF2: Usabilidade

#### Objetivo
&emsp;&emsp;Avaliar a facilidade de uso da aplicação WEB pelos usuários.

#### Roteiro de Teste
1. Fornecer uma breve introdução do sistema aos usuários.
2. Pedir aos usuários para operarem o robô seguindo um percurso específico.
3. Coletar feedback sobre a experiência de uso através de um questionário baseado no System Usability Scale (SUS).

### Resultados

#### Maurício Azevedo
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Introdução ao sistema**: Recebeu uma introdução detalhada do sistema.
  - **Operar o robô seguindo um percurso**: Seguiu o percurso sem dificuldades.
  - **Coletar feedback**: Deu feedback positivo sobre a facilidade de uso e navegabilidade do sistema.

#### Fernando Machado
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Introdução ao sistema**: Recebeu uma introdução detalhada do sistema.
  - **Operar o robô seguindo um percurso**: Teve um pouco de dificuldade nas curvas, mas completou o percurso.
  - **Coletar feedback**: Sugeriu melhorias para facilitar a navegação em curvas.

#### Rafaela
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Introdução ao sistema**: Recebeu uma introdução detalhada do sistema.
  - **Operar o robô seguindo um percurso**: Teve um pouco de dificuldade nas curvas, mas completou o percurso.
  - **Coletar feedback**: Sugeriu melhorias para facilitar a navegação em curvas e na área de informações.

#### Andrea
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Introdução ao sistema**: Recebeu uma introdução detalhada do sistema.
  - **Operar o robô seguindo um percurso**: Teve um pouco de dificuldade nas curvas e em guiar o robô, mas completou o percurso.
  - **Coletar feedback**: Sugeriu melhorias para facilitar a navegação em curvas e melhorar a qualidade da câmera.

### RNF3: Velocidade de Locomoção

#### Objetivo
&emsp;&emsp;Verificar a aceleração e a velocidade máxima constante do Turtlebot durante seu funcionamento.

#### Roteiro de Teste
1. Configurar o ambiente de teste.
2. Medir a aceleração e a velocidade máxima do Turtlebot em um percurso reto.
3. Monitorar e registrar a velocidade do robô em diferentes intervalos de tempo.
4. Analisar os dados coletados para verificar a constância da velocidade máxima marcada em 15 m/s.

### Resultados

#### Maurício Azevedo
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Configurar o ambiente de teste**: Testou na própria interface de controle.
  - **Medir a aceleração e a velocidade máxima**: Mediu e verificou que o robô alcançou a velocidade máxima sem dificuldades.
  - **Monitorar e registrar a velocidade**: Registrou a velocidade do robô em diferentes intervalos, confirmando a constância.
  - **Analisar os dados**: Analisou os dados e confirmou que a aceleração e a velocidade do robô eram constantes.

#### Fernando Machado
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Configurar o ambiente de teste**: Testou na própria interface de controle.
  - **Medir a aceleração e a velocidade máxima**: Mediu e verificou que o robô alcançou a velocidade máxima sem dificuldades.
  - **Monitorar e registrar a velocidade**: Registrou a velocidade do robô em diferentes intervalos, confirmando a constância.
  - **Analisar os dados**: Analisou os dados e confirmou que a aceleração e a velocidade do robô eram constantes.

#### Rafaela
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Configurar o ambiente de teste**: Testou na própria interface de controle.
  - **Medir a aceleração e a velocidade máxima**: Mediu e verificou que o robô alcançou a velocidade máxima sem dificuldades.
  - **Monitorar e registrar a velocidade**: Registrou a velocidade do robô em diferentes intervalos, confirmando a constância.
  - **Analisar os dados**: Analisou os dados e confirmou que a aceleração e a velocidade do robô eram constantes.

#### Andrea
- **Resultado Geral**: Sucesso
- **Etapas**:
  - **Configurar o ambiente de teste**: Testou na própria interface de controle.
  - **Medir a aceleração e a velocidade máxima**: Mediu e verificou que o robô alcançou a velocidade máxima sem dificuldades.
  - **Monitorar e registrar a velocidade**: Registrou a velocidade do robô em diferentes intervalos, confirmando a constância.
  - **Analisar os dados**: Analisou os dados e confirmou que a aceleração e a velocidade do robô eram constantes.

## 4. Conclusão

&emsp;&emsp;O resultado do teste demonstrou algumas falhas de funcionalidades do produto, principlamente relacionadas à usabilidade do usuário. Fora relatado que a tela apresenta muitos botões chamativos, mas que não apresentam funcionalidade dentro do sistema, a câmera apresenta uma latência muito alta mas a movimentação é relativamente suave. Ainda assim, os _testers_ idenficaram rapidamente os botões essenciais e conseguiram realizar todas as tarefas, mesmo que as vezes tenha faltado feedback ao pressionar botões, como por exemplo o botão de emergência. 

&emsp;&emsp;(Link para a tabela de testes)[https://docs.google.com/spreadsheets/d/1BIxLtNFbU7E4AKV3grvLv3TUp_qiBLQ4ps3oZy_a7gU/edit?usp=sharing]
