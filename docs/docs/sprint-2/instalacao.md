---
title: Instalação e execução
sidebar_position: 3
---

# Instalação e execução

## Movimentação do robô

### Pré-requisitos

- ROS2 instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para teleoperá-lo

- [Pacote ROS do Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master) instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para teleoperá-lo

- Turtlebot 3 e computador usado para teleoperá-lo conectados na mesma rede wi-fi

### Comunicação via rede local

Passo a passo

1. No sistema operacional da Raspberry contida no Turtlebot 3 a ser controlado, abra uma janela de terminal e digite os seguintes comandos para ativar a  

Para executar o script em questão, deve-se seguir os seguintes passos:


### Comunicação via SSH



## Como instalar o TinyDB

Para realizar a instalação do TinyDB, é sugerido o uso de um ambiente virtual pelos seguintes comandos:

`Verifique se você está na raiz do projeto`

- python3 -m venv venv

Este comando vai criar um ambiente virtual na pasta `venv`

É necessario ativar o ambiente virtual com o seguinte comando:

- source venv/bin/activate

Apos a ativação do venv, deve-se intalar o TinyDB com o comando `pip install tinydb`