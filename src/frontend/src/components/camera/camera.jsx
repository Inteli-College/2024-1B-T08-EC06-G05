import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const VideoStream = () => {
  const videoRef = useRef(null);
  const [latency, setLatency] = useState(0);
  const [sentTime, setSentTime] = useState(null);
  const ros = useRef(null); // Use ref to store ros instance

  useEffect(() => {
    // Create and connect to the ROS server if not already connected
    if (!ros.current) {
      ros.current = new ROSLIB.Ros({
        url: 'ws://10.128.0.50:9090' // TROCAR POR 'ws://localhost:9090' PARA TESTES LOCAIS
      });

      ros.current.on('connection', () => {
        console.log('Camera: Connected to websocket server.');
      });

      ros.current.on('error', (error) => {
        console.log('Camera: Error connecting to websocket server: ', error);
      });

      ros.current.on('close', () => {
        console.log('Connection to websocket server closed.');
      });
    }

    // Inscrever-se no tópico de frames de vídeo
    const videoTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    // Inscrever-se no tópico de latência
    const latencyTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/latency',
      messageType: 'std_msgs/String'
    });

    latencyTopic.subscribe((message) => {
      const receivedSentTime = new Date(message.data);
      setSentTime(receivedSentTime);
    });

    videoTopic.subscribe((message) => {
      if (videoRef.current) {
        videoRef.current.src = 'data:image/jpeg;base64,' + message.data;
      }

      if (sentTime) {
        const currentTime = new Date();
        const calculatedLatency = currentTime - sentTime; // latência em milissegundos
        setLatency(calculatedLatency);
        console.log('Current Time:', currentTime);
        console.log('Sent Time:', sentTime);
        console.log('Calculated Latency:', calculatedLatency);

        // Resetar sentTime para null para evitar o uso do mesmo sentTime para múltiplas imagens
        setSentTime(null);
      }
    });

    // Limpar as inscrições quando o componente desmontar
    return () => {
      videoTopic.unsubscribe();
      latencyTopic.unsubscribe();
    };
  }, [sentTime]);

  return (
    <div>
      <img
        id="videoStream"
        ref={videoRef}
        alt="Video Stream"
        style={{ width: '1280px', height: '720px', position: 'fixed', zIndex: -50 }}
      />
      <div className="flex">
        <div className="h-5vh p-1 w-33 bg-opacity-70 bg-orange-400 flex items-center font-bold text-black text-xl font-sans absolute right-0">
          Latency: {latency} ms
        </div>
      </div>
    </div>
  );
};

export default VideoStream;
