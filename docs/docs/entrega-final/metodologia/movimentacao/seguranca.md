---
title: Documentação do Sistema de Segurança
sidebar_position: 1
---

# Documentação do Sistema de Segurança - Versão Atualizada

## Introdução

&emsp;&emsp;Na sprint 2, foram apresentados dois sistemas de segurança pelo grupo SugarZ3ro atrelados ao controle do Turtlebot, através dos quais o usuário teria mais precisão na movimentação e a opção de acionar uma parada de emergência ao pressionar a tecla `Q`. Durante a sprint 3, o sistema de segurança passou por significativas melhorias para aprimorar a detecção de obstáculos e a prevenção de colisões, utilizando dados de um LiDAR (Laser Imaging Detection and Ranging). Na sprint 5, o sistema de segurança usando um LiDAR foi aprimorado para que a movimentação do robô continuasse possível apenas nas direções onde objetos não foram identificados, o que torna a usabilidade do sistema de segurança de maior qualidade e facilita a utilização pelo usuário.

## Sistema de Segurança

&emsp;&emsp;O sistema de segurança integrado à interface foi aprimorado no mesmo diretório do frontend, no arquivo que controla a movimentação do Turtlebot 3. O arquivo `rosbridge_movement.jsx`, localizados em `~src/frontend/src/components/rosbridge_movement/rosbridge_movement.jsx`, agora a movimentação do robô é interrompida na direção em que o obstáculo foi identificado e, ao mesmo tempo, permite-se que a movimentação do robô continue nas direções onde nenhum obstáculo foi identificado.

1.  **Inicialização do tópico de verificação (dados LiDAR)**
    ```javascript
    // Initialize the scan topic (LiDAR data)
    const lidarTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
    });

    lidarTopic.subscribe((message) => {
      checkForObstacles(message);
    });

    return () => {
      ros.current.close();
    };
    ```
    - **Descrição**: Este trecho de código cria um subscription para o tópico `scan`, que recebe dados do sistema LiDAR.
    - **Função**: Permite que o nó receba continuamente dados de distância do sistema LiDAR, necessários para detectar obstáculos.

2. **Callback do LiDAR (LaserScan)** 
    ```javascript
    const checkForObstacles = (data) => {
    const ranges = data.ranges;
    const minDistance = 0.3; // Define a distância mínima segura

    // Filtrar leituras inválidas
    const validRanges = ranges.filter(range => range > 0 && range < Infinity);

    if (validRanges.length === 0) {
        setLidarData('none');
        return; // Sem leituras válidas, sair da função
    }

    const minRange = Math.min(...validRanges);

    if (minRange <= minDistance) {
        const minIndex = ranges.indexOf(minRange);
        const numberOfIndices = ranges.length;

        const valorA = Math.floor(numberOfIndices / 4);
        const valorB = valorA * 3;

        if (valorA < minIndex && minIndex < valorB) {
        if (lidarData !== 'back') {
            console.log('Obstáculo detectado atrás');
            handleStop()
            setCollision(true)
            setLidarData('back');
            broadcastObstacle('back');
        }
        } else {
        if (lidarData !== 'front') {
            console.log('Obstáculo detectado à frente');
            handleStop()
            setCollision(true)
            setLidarData('front');
            broadcastObstacle('front');
        }
        }
    } else {
        console.log('Nenhum obstáculo detectado');
        setLidarData('none');
        setCollision(false);
        broadcastObstacle('none');
    }
    };

    ```
    - Descrição: Esta função é chamada sempre que uma nova mensagem é publicada no tópico `scan`.
    - Função: Verifica se há obstáculos usando os dados do LiDAR. Se múltiplas leituras válidas indicarem a presença de um obstáculo a uma distância menor que 0.3 metros, o estado collision é atualizado para true.

3. **Função de Movimento**
    ```javascript
        const move = (linear, angular) => {
            if ((lidarData === 'front' && linear > 0) || (lidarData === 'back' && linear < 0)) {
                console.log('Collision detected! Stopping movement.');
                handleStop();
                return; // Não enviar comandos de movimento na direção do obstáculo
            }

            console.log(`Moving: linear=${linear}, angular=${angular}`);
            const twist = new ROSLIB.Message({
                linear: { x: linear, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angular }
            });
            cmdVel.current.publish(twist);
            };
    ```
    - Descrição: Esta função publica uma mensagem no tópico /cmd_vel para controlar o movimento do TurtleBot.
    - Função: Se uma colisão é detectada, impede o movimento para frente ou para trás, conforme a direção do obstáculo. Caso contrário, permite o movimento com as velocidades linear e angular especificadas.

4. **Manipuladores de Movimento**
    ```javascript
    const handleForward = () => move(0.2, 0);
    const handleLeft = () => move(0, 0.5);
    const handleRight = () => move(0, -0.5);
    const handleBackward = () => move(-0.2, 0);
    const handleStop = () => move(0, 0);
    const handleTurnoff = () => {
    handleStop();
    ros.current.close();
    };

    ```
    - Descrição: Funções específicas para controlar os movimentos do TurtleBot (avançar, virar à esquerda, virar à direita, mover para trás, parar e desligar).
    - Função: Permitem controlar o TurtleBot através de comandos específicos.

5. **Parâmetros do Sistema de Segurança**
    `minDistance`:
        - Descrição: A distância mínima segura antes de tomar ação corretiva.
        - Valor: 0.3 metros (30 cm).

    `collision`:
        - Descrição: Estado que indica se uma colisão foi detectada.
        - Valores possíveis: True ou False.

## Conclusão

&emsp;&emsp;O componente TurtleBotController é crucial para garantir que o TurtleBot opere de maneira segura e eficiente. Ele utiliza dados do LiDAR para detectar obstáculos e ajusta o movimento do robô conforme necessário para evitar colisões. Este sistema é implementado de forma contínua e responsiva, assegurando que o TurtleBot possa navegar de maneira autônoma sem riscos de danos.