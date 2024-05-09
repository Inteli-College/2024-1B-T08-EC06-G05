---
title: Movimentação do robô
sidebar_position: 1
---

# Movimentação do robô

&emsp;&emsp;Durante a segunda sprint do projeto, a equipe SugarZ3ro focou no desenvolvimento de uma forma de movimentação do robô de maneira remota. Para isso, conforme descrito na [proposta inicial de arquitetura](../../sprint-1/arquitetura/arquitetura.md) do projeto, a equipe utilizou ROS 2 (Robot Operating System) através do sistema operacional Linux Ubuntu, que foi instalado tanto na Raspberry contida no Turtlebot quanto no computador utilizado para controlar a movimentação do robô.

## Estrutura de diretórios e arquivos

&emsp;&emsp;Para tornar prático o desenvolvimento e a execução da movimentação remota do robô, a equipe optou por utilizar organização de diretórios padrão em projetos envolvendos ROS 2. Desse modo, a estrutura de pastas do projeto conta com um workspace em ```~/app/workspace```, diretório no qual há um pacote ROS ```SugarZ3ro_pkg``` criado pela equipe SugarZ3ro.

&emsp;&emsp;Tal pacote apresenta uma estrutura comum a pacotes ROS padrões, a qual é minuciosamente descrita na [documentação oficial do ROS](https://docs.ros.org/en/dashing/Tutorials/Creating-Your-First-ROS2-Package.html). Consequentemente, o script com o código para a movimentação remota do robô se encontra em ```./SugarZ3r0_pkg/movimentation.py``` e pode ser executada por meio de um comando pré-definido graças ao arquivo ```setup.py``` contido no diretório principal do pacote. 

## Execução

&emsp;&emsp;Para executar o controle da movimentação remota do robô, confira a [respectiva seção](../instalacao.md).

## CLI (Command Line Interface)

&emsp;&emsp;Após seguir o passo a passo de execução, o usuário terá acesso a uma CLI (Command Line Interface) que o permitirá movimentar o robô usando as teclas ```W```, ```A```, ```D```, ```S``` e ```Q```. Cada tecla executa uma ação diferente ao ser pressionada, conforme descição na tabela 1, a seguir.

<p style={{textAlign: 'center'}}>Tabela 1 - Relação entre teclas e ações executadas</p>

| **Tecla** | **Ação**                                                   |
|-----------|------------------------------------------------------------|
| W         | Aumenta velocidade linear do robô                          |
| A         | Aumenta velocidade angular do robô em sentido anti-horário |
| D         | Aumenta velocidade angular do robô em sentido horário      |
| S         | Iguala a velocidade (linear e/ou angular) do robô a 0      |
| Q         | Interrompe a execução do código de movimentação do robô    |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe SugarZ3ro</p>

&emsp;&emsp;Por aumento de velocidade linear, entende-se que o robô segue em linha reta. Já pelo aumento da velocidade angular em sentido anti-horário e sentido horário, entende-se que o robô vira, respectivamente, para a esquerda e para a direita em relação ao seu próprio eixo. Por fim, por igualar a velocidade do robô a 0, entende-se que o robô fica imóvel. 

## Sistema de segurança


