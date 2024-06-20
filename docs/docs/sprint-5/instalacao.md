---
title: Instalação e execução
sidebar_position: 2
---

# Instalação e execução

Nesta seção, você pode conferir as instruções para instalar e executar cada uma das partes do sistema da solução desenvolvidas até a sprint 5.

## Movimentação do robô via aplicação WEB

### Pré-requisitos

- ROS2 instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- [Pacote ROS do Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master) instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- Raspberry do Turtlebot 3 e computador usado para operá-lo remotamente conectados na mesma rede wi-fi

- Git instalado no computador usado para operar o robô remotamente

- Pacote ROSBridge do Turtlebot 3 instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- Node.js atualizado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3

### Comunicação via SSH

#### Passo a passo

1. No sistema operacional da Raspberry contida no Turtlebot 3 a ser controlado, abra uma janela de terminal e digite os seguintes comandos para encontrar o IP do Turtlebot 3 dado pela rede:

```bash
    ip addr
```

Copie o número registrado em Wlan que vem antes da "/" (exemplo: 10.128.0.30). Esse será o IP que será utilizado para comunicação do robô para a aplicação WEB.

2. Na mesma janela de terminal, digite os seguintes comandos para instalar o pacote responsável por iniciar um servidor SSH e executá-lo.


```bash
    sudo apt install openssh-server
    sudo systemctl enable ssh
    sudo ufw allow ssh
    sudo systemctl start ssh
```

3. No sistema operacional do computador que será utilizado para controlar o robô de maneira remota, abra uma janela de terminal e digite o seguinte comando:

```bash
    ssh user@server
```

No comando acima, `user` é o nome de usuário do sistema operacional da Raspberry do Turtlebot 3 e `server` é o ip copiado do passo 1. Caso você esteja usando o Turtlebot 3 do grupo 5, o comando a ser digitado nessa etapa será `ssh sugarz3ro@IpCopiado` .

4. Digite a senha de usuário que será solicitada pelo terminal.

5. Com o ssh conectado, digite os seguintes comandos para limitar a comunicação via ROS a um domínio com ID 5 dentro da rede:

```bash
    echo 'export ROS_DOMAIN_ID=5' >> ~/.bashrc
    source ~/.bashrc
```

6. Com o ssh conectado, clone o repositório do projeto no diretório de sua preferência através dos comandos:

```bash
    git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G05.git
```

7. Na mesma janela de terminal, altere o IP em dois scripts: `rosbridge_movement.jsx` e `camera.jsx` Com os comandos abaixo:

```bash
    cd 2024-1B-T08-EC06-G05/src/frontend/src/components/camera
    nvim camera.jsx
    cd ..
    cd rosbridge_movement/
    nvim rosbridge_movement/
```

Para mudar o IP, após realizar o comando nvim nos dois casos terá uma parte do script como mostrado abaixo. Basta mudar a sequência de números antes do caractere de dois pontos no url para o IP copiado no passo 1.

```bash
    ros.current = new ROSLIB.Ros({
      url: 'ws://10.128.0.30:9090'
    });
```

8. Na mesma janela do terminal, digite o seguinte comando para iniciar a comunicação entre a Raspberry e o microcontrolador do robô, bem como torná-lo apto a receber comandos de movimentação remotamente:

```bash
    ros2 launch turtlebot3_bringup robot.launch.py
```

9. Em outra janela de terminal, ainda conectado no ssh, digite o seguinte comando para iniciar o ROSBridge para estabelecer a comunicação entre o robô e a aplicação WEB:

```bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

10. Em outra janela de terminal, ainda conectado no ssh, digite os seguintes comandos partindo da raiz do projeto para transmitir as informações da câmera:

```bash
    cd src/workspace/src/SugarZ3ro_pkg/SugarZ3ro_pkg/
    python3 sender.py
```

11. Em outra janela de terminal, desconectado do ssh, digite os seguintes comandos a partir da raiz do projeto para ir até a pasta do frontend e iniciar a aplicação WEB, que já contará com a movimentação do robô e o sistema de segurança integrados:

```bash
    cd 2024-1B-T08-EC06-G05/src/frontend
    npm run dev
```

12. Em outra janela de terminal, desconectado do ssh, digite os seguintes comandos a partir da raiz do projeto para ir até a pasta do backend e iniciar a API da solução:

```bash
    cd 2024-1B-T08-EC06-G05/src/backend
    flask --app app run --debug
```

## Movimentação do robô via CLI (Caso de emergência)

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

1. Siga o [passo 1](#passo-a-passo) do tutorial para comunicação via rede local.

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

É necessário ativar o ambiente virtual com o seguinte comando:

- source venv/bin/activate

Após a ativação do venv, deve-se instalar o TinyDB com o comando `pip install tinydb`

## Como instalar o YOLOv8

Para realizar a instalação do YOLOv8, é sugerido o uso de um ambiente virtual pelos comandos apresentados na seção de Como instalar o TinyDB. 

Após garantir que o ambiente virtual está ativado,, deve-se instalar o Ultralytics com o seguinte comando:

`pip install ultralytics`

## Como treinar o modelo usando YOLOv8

Para realizar o treinamento do modelo, é necessário que os parametros sejam ajustados de acordo com as necessidades do treinamento no arquivo `yolo.py`. Após os ajustes, execute o arquivo até que o treinamento seja finalizado, e então as métricas serão apresentadas no terminal.

## Como utilizar o modelo treinado usando YOLOv8

Para testar a eficiencia do modelo treinado usando YOLOv8, pode ser utilizado dois arquivos de codigo: `yoloVideo.py`(para teste com imagens capturadas por uma webcam em tempo real) e `yoloImagem.py`(para imagens salvas em uma base de dados)

No arquivo yoloVideo.py, deve ser alterado o modelo que será usando no processo de teste de acordo com o desejado pelo desenvolvedor na seguinte linha:
`model = YOLO("./home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train4/weights/best.pt")`

No arquivo `yoloImagem.py`, a imagem à ser analisada é definida na seguinte linha:

`image_path = "../data-base/imgs/img2.png"`

Caso seja necessário analisar outra imagens, o "path" deve ser atualizado e a imagem deve ser adicionada na pasta `/data-base/imgs`



