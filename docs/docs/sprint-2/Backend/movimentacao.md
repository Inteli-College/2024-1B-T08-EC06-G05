---
title: Movimentação do robô
sidebar_position: 1
---

# Movimentação do robô

&emsp;&emsp;Durante a segunda sprint do projeto, a equipe SugarZ3ro focou no desenvolvimento de uma forma de movimentação do robô de maneira remota. Para isso, conforme descrito na [proposta inicial de arquitetura](../../sprint-1/arquitetura/arquitetura.md) do projeto, a equipe utilizou ROS 2 (Robot Operating System) através do sistema operacional Linux Ubuntu, que foi instalado tanto na Raspberry contida no Turtlebot quanto no computador utilizado para controlar a movimentação do robô.

## Estrutura de diretórios e arquivos

&emsp;&emsp;Para tornar prático o desenvolvimento e a execução da movimentação remota do robô, a equipe optou por utilizar organização de diretórios padrão em projetos envolvendos ROS 2. Desse modo, a estrutura de pastas do projeto conta com um workspace em ```~/app/workspace```, diretório no qual há um pacote ROS ```SugarZ3ro_pkg``` criado pela equipe SugarZ3ro localizado em ```~/app/workspace/src/SugarZ3ro_pkg```.

&emsp;&emsp;Tal pacote apresenta uma estrutura comum a pacotes ROS padrões, a qual é minuciosamente descrita na [documentação oficial do ROS](https://docs.ros.org/en/dashing/Tutorials/Creating-Your-First-ROS2-Package.html). Consequentemente, o script com o código para a movimentação remota do robô se encontra em ```~/app/workspace/src/SugarZ3ro_pkg/SugarZ3r0_pkg/movimentation.py``` e pode ser executada por meio de um comando pré-definido graças ao arquivo ```setup.py``` contido no diretório principal do pacote. 

## Execução

### Pré-requisitos

- ROS2 instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e  

Para executar o script em questão, deve-se seguir os seguintes passos:

1. No sistema operacional da Raspberry contida no Turtlebot 3 a ser controlado,

&emsp;&emsp;

introducao

localizacao (estrutura de pasta)

como executar

como usar / funcionamento do código

video de demonstração