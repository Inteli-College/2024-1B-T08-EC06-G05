import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const VideoStream = ({ aiButtonState }) => {
  const videoRef = useRef(null);
  const [latency, setLatency] = useState(0);
  const [sentTime, setSentTime] = useState(null);
  const ros = useRef(null); // Use ref to store ros instance
  const [currentFrame, setCurrentFrame] = useState(null);
  const [hasSentImageString, setHasSentImageString] = useState(false);

  useEffect(() => {
    // Create and connect to the ROS server if not already connected
    if (!ros.current) {
      ros.current = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // TROCAR POR 'ws://localhost:9090' PARA TESTES LOCAIS
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

    
    // Subscribe to the video frames topic
    const videoTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    // Subscribe to the latency topic
    const latencyTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/latency',
      messageType: 'std_msgs/String'
    });

    latencyTopic.subscribe((message) => {
      const sentTime = new Date(message.data);
      setSentTime(sentTime);
    });

    videoTopic.subscribe((message) => {
      if (videoRef.current) {
        videoRef.current.src = 'data:image/jpeg;base64,' + message.data;
        setCurrentFrame(message.data); // Update currentFrame using useState
      }
      
      if (sentTime) {
        const currentTime = new Date();
        const latency = currentTime - sentTime; // latency in milliseconds
        setLatency(latency);
        // Reset sentTime to null to avoid using the same sentTime for multiple images
        setSentTime(null);
      }
    });

    // Clean up the subscriptions when the component unmounts
    return () => {
      videoTopic.unsubscribe();
      latencyTopic.unsubscribe();
    };
  }, [sentTime]);

  useEffect(() => {
    if (aiButtonState && currentFrame && !hasSentImageString) {
      console.log('botao apertado'); // Log the currentFrame

      fetch('http://127.0.0.1:5000/post_img_string', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ currentFrame: currentFrame }),
      })

      setHasSentImageString(true); // Mark that the frame has been logged

    } else if (!aiButtonState) {

      setHasSentImageString(false); // Reset the flag when the button is not pressed

    }
  }, [aiButtonState, currentFrame, hasSentImageString]);
  

  return (
    <div>
      <img
        id="videoStream"
        ref={videoRef}
        alt="Video Stream"
        style={{ width: '1280px', height: '720px', position: 'fixed', zIndex: -50, marginTop: 0 }}
      />
      <div className="flex mt-4">
        <div className="h-5vh p-1 w-33 bg-opacity-70 bg-orange-400 flex items-center font-bold text-black text-xl font-sans absolute right-0">
          Latency: {latency} ms
        </div>
      </div>
    </div>
  );
};

export default VideoStream;
