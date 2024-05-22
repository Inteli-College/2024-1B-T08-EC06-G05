// src/components/TurtleBotController.js
import React, { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const TurtleBotController = ({ children }) => {
  const ros = useRef(null);
  const cmdVel = useRef(null);

  useEffect(() => {
    // Connect to the ROS bridge server
    ros.current = new ROSLIB.Ros({
      url: 'ws://10.128.0.30:9090'
    });

    ros.current.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    ros.current.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.current.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    // Initialize the cmd_vel topic
    cmdVel.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    return () => {
      ros.current.close();
    };
  }, []);

  // Function to handle movements
  const move = (linear, angular) => {
    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
    cmdVel.current.publish(twist);
  };

  // Movement handlers
  const handleForward = () => move(0.10, 0);
  const handleLeft = () => move(0, 1.0);
  const handleRight = () => move(0, -1.0);
  const handleStop = () => move(0, 0);
  const handleTurnoff = () => {
    handleStop();
    ros.current.close();
  };

  const movementHandlers = {
    forward: handleForward,
    left: handleLeft,
    right: handleRight,
    stop: handleStop,
    turnoff: handleTurnoff
  };

  return React.Children.map(children, child => {
    return React.cloneElement(child, { movementHandlers });
  });
};

export default TurtleBotController;
