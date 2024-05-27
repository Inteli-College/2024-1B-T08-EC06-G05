
import React, { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import EventEmitter2 from 'eventemitter2';

const VideoStream = () => {
  const videoRef = useRef(null);

  useEffect(() => {
    // Conectar ao servidor ROS
    const ros = new ROSLIB.Ros({
      url: 'ws://10.128.0.30:9090'
    });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
    }); 

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    // Assinar ao tópico de vídeonsor_msgs/Compres
    const videoTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    // Função para lidar com os quadros de vídeo recebidos
    videoTopic.subscribe((message) => {
      if (videoRef.current) {
        videoRef.current.src = 'data:image/jpeg;base64,' + message.data;
      }
    });

    // Limpar a assinatura quando o componente for desmontado
    return () => {
      videoTopic.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <div>
      <h1>Real-time Video Stream from ROS2 Topic</h1>
      <img id="videoStream" ref={videoRef} alt="Video Stream" style={{ width: '640px', height: '480px' }} />
    </div>
  );
};

export default VideoStream;
