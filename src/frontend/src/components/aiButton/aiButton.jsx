import React, { useRef, useState, useEffect } from "react";
import axios from "axios";
import { BsStars } from "react-icons/bs";

const AiButton = () => {
  const videoRef = useRef(null);
  const [status, setStatus] = useState('');

  useEffect(() => {
    if (videoRef.current) {
      videoRef.current.addEventListener('loadeddata', () => {
        console.log('Video loaded');
      });
    }
  }, []);

  const handleSaveFrame = async (event) => {
    event.preventDefault();

    if (videoRef.current) {
      const canvas = document.createElement('canvas');
      canvas.width = videoRef.current.videoWidth;
      canvas.height = videoRef.current.videoHeight;
      const context = canvas.getContext('2d');

      if (!context) {
        console.error('Error: Canvas context is null');
        return;
      }

      context.drawImage(videoRef.current, 0, 0, canvas.width, canvas.height);
      console.log('Canvas drawn from video');

      canvas.toBlob(async (blob) => {
        if (blob) {
          console.log('Blob created from canvas');
          const formData = new FormData();
          formData.append('image', blob, 'frame.png');

          try {
            console.log('Sending image to server...');
            const response = await axios.post('http://localhost:5000/process_image', formData, {
              headers: {
                'Content-Type': 'multipart/form-data'
              }
            });
            const status = response.data.status;
            setStatus(status);
            console.log(`Status: ${status}`);
          } catch (error) {
            console.error('Error processing image', error);
          }
        } else {
          console.error('Error: Canvas.toBlob did not return a blob');
        }
      }, 'image/png');
    }
  };

  return (
    <div>
      
      <form onSubmit={handleSaveFrame}>
        <button
          type="submit"
          className="bg-white bg-opacity-70 active:bg-opacity-100 rounded-xl border border-gray-700 flex items-center justify-center p-6"
        >
          <BsStars size={70} />
        </button>
      </form>
      {status && <p>Status: {status}</p>}
    </div>
  );
};

export default AiButton;
