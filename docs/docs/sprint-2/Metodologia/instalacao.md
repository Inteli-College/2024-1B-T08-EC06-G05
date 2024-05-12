---
title: Instalação e execução
sidebar_position: 3
---

# Instalação e execução

Nesta seção, você pode conferir as instruções para instalar e executar cada uma das partes do sistema da solução desenvolvidas até a sprint 2.

## Movimentação do robô

### Pré-requisitos

- ROS2 instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- [Pacote ROS do Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master) instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- Raspberry do Turtlebot 3 e computador usado para operá-lo remotamente conectados na mesma rede wi-fi

- Git instalado no computador usado para operar o robô remotamente

### Comunicação via rede local

#### Passo a passo

1. No sistema operacional da Raspberry contida no Turtlebot 3 a ser controlado, abra uma janela de terminal e digite os seguintes comandos para limitar a comunicação via ROS a um domínio com ID 5 dentro da rede:

`echo 'export ROS_DOMAIN_ID=5' >> ~/.bashrc`

`source ~/.bashrc`

2. Na mesma janela de terminal, digite o seguinte comando para iniciar a comunicação entre a Raspberry e o microcontrolador do robô, bem como torná-lo apto a receber comandos de movimentação remotamente:

`ros2 launch turtlebot3_bringup robot.launch.py`

3. No sistema operacional do computador que será utilizado para controlar o robô de maneira remota, abra uma janela de terminal no diretório de sua preferência e clone o repositório através do seguinte comando:

`git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G05.git`

4. Na mesma janela de terminal, digite os seguintes comandos para iniciar o build do workspace:

`cd 2024-1B-T08-EC06-G05/src/workspace`

`colcon build`

5. Na mesma janela de terminal, digite o seguinte comando para habilitar o uso do pacote criado pela equipe SugarZ3ro:

`source install/local_setup.bash`

6. Na mesma janela de terminal, digite os seguintes comandos para limitar a comunicação via ROS a um domínio com ID 5 dentro da rede:

`echo 'export ROS_DOMAIN_ID=5' >> ~/.bashrc`

`source ~/.bashrc`

7. Por fim, na mesma janela de terminal, digite o seguinte comando para executar o script responsável por inicializar a CLI para controle de movimentação do robô:

`ros2 run SugarZ3ro_pkg start_moving`

### Comunicação via SSH

#### Passo a passo

1. Siga o [passo 1](./instalacao.md#comunicação-via-rede-local) do tutorial para comunicação via rede local.

2. No sistema operacional da Raspberry do Turtlebot 3, abra uma janela de terminal e clone e dê build no repositório do projeto no diretório de sua preferência através dos comandos:

`git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G05.git`

`cd 2024-1B-T08-EC06-G05/src/workspace`

`colcon build`

3. Na mesma janela de terminal, digite os seguintes comandos para instalar o pacote responsável por iniciar um servidor SSH e executá-lo.

`sudo apt install openssh-server`

`sudo systemctl enable ssh`

`sudo ufw allow ssh`

`sudo systemctl start ssh`

4. Na mesma janela de terminal, digite o seguinte comando para iniciar a comunicação entre a Raspberry e o microcontrolador do robô, bem como torná-lo apto a receber comandos de movimentação remotamente:

`ros2 launch turtlebot3_bringup robot.launch.py`

5. No sistema operacional do computador que será utilizado para controlar o robô de maneira remota, abra uma janela de terminal e digite o seguinte comando:

`ssh user@server`

:::tip

No comando acima, `user` é o nome de usuário do sistema operacional da Raspberry do Turtlebot 3 e `server` é o ip desta. Caso você esteja usando o Turtlebot 3 do grupo 5, o comando a ser digitado nessa etapa será `ssh sugarz3ro@10.128.0.8` .

:::

6. Digite a senha de usuário que será solicitada pelo terminal.

7. Por fim, adentre o diretório no qual você clonou o repositório do projeto (no passo 2) e digite os seguintes comandos para executar a CLI de movimentação do robô:

`cd src/workspace`

`source install/local_setup.bash`

`ros2 run SugarZ3ro_pkg start_moving`

---

## Como instalar o TinyDB

Para realizar a instalação do TinyDB, é sugerido o uso de um ambiente virtual pelos seguintes comandos:

`Verifique se você está na raiz do projeto`

- python3 -m venv venv

Este comando vai criar um ambiente virtual na pasta `venv`

É necessario ativar o ambiente virtual com o seguinte comando:

- source venv/bin/activate

Após a ativação do venv, deve-se intalar o TinyDB com o comando `pip install tinydb`