// src/components/TurtleBotController.js
import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const TurtleBotController = ({ children }) => {
  const ros = useRef(null);
  const cmdVel = useRef(null);
  const lidarData = useRef(null);
  const [collision, setCollision] = useState(false);

  useEffect(() => {
    // Connect to the ROS bridge server
    ros.current = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
      // TROCAR POR 'ws://localhost:9090' PARA TESTES LOCAIS
      // TROCAR POR 'ws://10.128.0.50:9090' PARA TESTES COM O ROBÔ
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
    lidarData.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
    });

    lidarData.current.subscribe((message) => {
      checkForObstacles(message);
    });

    return () => {
      ros.current.close();
    };
  }, []);

  const checkForObstacles = (message) => {
    const ranges = message.ranges;
    const minDistance = 0.3; // Increase this value if necessary

    let isObstacleDetected = false;
    let validReadings = 0;

    for (let i = 0; i < ranges.length; i++) {
      if (ranges[i] < minDistance && ranges[i] > 0) { // Ignore invalid readings (e.g., 0 values)
        validReadings++;
        if (validReadings > 5) { // Only consider it an obstacle if multiple valid readings are detected
          isObstacleDetected = true;
          console.log("OBSTÁCULO DETECTADO!");
          break;
        }
      }
    }

    setCollision(isObstacleDetected);
  };

  // Function to handle movements
  const move = (linear, angular) => {
    if (collision && linear > 0) {
      console.log('Collision detected! Stopping movement.');
      handleStop()
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
