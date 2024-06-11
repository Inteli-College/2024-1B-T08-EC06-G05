// src/components/TurtleBotController.js
import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const TurtleBotController = ({ children }) => {
  const ros = useRef(null);
  const cmdVel = useRef(null);
  const [lidarData, setLidarData] = useState('none');
  const [collision, setCollision] = useState(false);

  useEffect(() => {
    // Connect to the ROS bridge server
    ros.current = new ROSLIB.Ros({
      url: 'ws://10.128.0.50:9090' // TROCAR POR 'ws://localhost:9090' PARA TESTES LOCAIS
    });

    ros.current.on('connection', () => {
      console.log('Movement: Connected to websocket server.');
    });

    ros.current.on('error', (error) => {
      console.log('Movement: Error connecting to websocket server: ', error);
    });

    ros.current.on('close', () => {
      console.log('Movement: Connection to websocket server closed.');
    });

    // Initialize the cmd_vel topic
    cmdVel.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

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
  }, []);

  const checkForObstacles = (data) => {
    const ranges = data.ranges;
    const minDistance = 0.2; // Define a distância mínima segura

    // Filtrar leituras inválidas
    const validRanges = ranges.filter(range => range > 0 && range < Infinity);

    if (validRanges.length === 0) {
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
          setLidarData('back');
          broadcastObstacle('back');
        }
      } else {
        if (lidarData !== 'front') {
          console.log('Obstáculo detectado à frente');
          setLidarData('front');
          broadcastObstacle('front');
        }
      }
    } else {
      if (lidarData !== 'none') {
        console.log('Nenhum obstáculo detectado');
        setLidarData('none');
        broadcastObstacle('none');
      }
    }
  };

  const broadcastObstacle = (position) => {
    const message = JSON.stringify({ obstacle: position });
    // Implementar a função de broadcast para enviar a mensagem para onde for necessário
    console.log('Broadcast:', message);
  };

  // Function to handle movements
  const move = (linear, angular) => {
    if (collision && linear > 0) {
      console.log('Collision detected! Stopping movement.');
      handleStop();
      linear = 0; // Stop forward movement if collision is detected
    }

    console.log(`Moving: linear=${linear}, angular=${angular}`);
    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
    cmdVel.current.publish(twist);
  };

  // Movement handlers
  const handleForward = () => move(0.2, 0);
  const handleLeft = () => move(0, 0.5);
  const handleRight = () => move(0, -0.5);
  const handleBackward = () => move(-0.2, 0);
  const handleStop = () => move(0, 0);
  const handleTurnoff = () => {
    handleStop();
    ros.current.close();
  };

  const movementhandlers = {
    forward: handleForward,
    left: handleLeft,
    right: handleRight,
    backward: handleBackward,
    stop: handleStop,
    turnoff: handleTurnoff
  };

  return (
    <>
      {typeof children === 'function'
        ? children({ movementhandlers, collision })
        : React.Children.map(children, (child) =>
            React.isValidElement(child)
              ? React.cloneElement(child, { movementhandlers, collision })
              : child
          )}
    </>
  );
};

export default TurtleBotController;
