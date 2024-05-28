import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const VideoStream = () => {
  const videoRef = useRef(null);
  const [latency, setLatency] = useState(0);
  const [sentTime, setSentTime] = useState(null);

  useEffect(() => {
    // Connect to the ROS server
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

    // Subscribe to the video frames topic
    const videoTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    // Subscribe to the latency topic
    const latencyTopic = new ROSLIB.Topic({
      ros: ros,
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
      }
      
      if (sentTime) {
        const currentTime = new Date();
        const latency = currentTime - sentTime; // latency in milliseconds
        setLatency(latency);
        console.log('Current Time:', currentTime);
        console.log('Sent Time:', sentTime);

        // Reset sentTime to null to avoid using the same sentTime for multiple images
        setSentTime(null);
      }
    });

    // Clean up the subscriptions when the component unmounts
    return () => {
      videoTopic.unsubscribe();
      latencyTopic.unsubscribe();
      ros.close();
    };
  }, [sentTime]);

  return (
    <div>
      <h1>Real-time Video Stream from ROS2 Topic</h1>
      <img
        id="videoStream"
        ref={videoRef}
        alt="Video Stream"
        style={{ width: '640px', height: '480px' }}
      />
      <div className="flex mt-4">
        <div className="w-0 h-0 border-t-[3vh] border-t-customBlue border-r-[3vh] border-r-customBlue border-l-[3vh] border-l-transparent border-b-[3vh] border-b-transparent"></div>
        <div className="h-5vh p-1 w-32 bg-customBlue flex items-center justify-end font-bold text-black text-xl font-sans">
          Latency: {latency} ms
        </div>
      </div>
    </div>
  );
};

export default VideoStream;