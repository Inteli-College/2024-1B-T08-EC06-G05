import React, { useState, useEffect } from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/rosbridge_movement/rosbridge_movement';
import MoveRight from '../components/movimentacao/direita';
import MoveLeft from '../components/movimentacao/esquerda';
import MoveForward from '../components/movimentacao/frente';
import MoveBackward from '../components/movimentacao/tras';
import WarningButton from '../components/warningButton/warning';
import Modal from '../components/modal/modal';  // Adjust the path as needed
import ObstaclePopUp from '../components/obstaclePopUp/obstaclePopUp';
import { Alert } from "@material-tailwind/react";
import { BackendAlert } from '../components/alert/backend_alert';  // Adjust the path as needed

export function AlertDefault() {
  return (
    <Alert className="bg-white text-black p-6 text-xl w-full max-w-lg mx-auto mt-4 shadow-lg rounded-lg">
      Botão de emergência ativado!! Para movimentar o robô novamente, recarregue a página!
    </Alert>
  );
}

function TeleopScreen() {
  const [showAlert, setShowAlert] = useState(false);
  const [showBackendAlert, setShowBackendAlert] = useState(false);
  const [backendAlertMessage, setBackendAlertMessage] = useState('');

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch('/api/alert'); // Adjust the endpoint URL as needed
        const data = await response.json();

        if (data.status) {
          setBackendAlertMessage('The operation was successful!');
        } else {
          setBackendAlertMessage('The operation failed!');
        }

        setShowBackendAlert(true);
        setTimeout(() => {
          setShowBackendAlert(false);
        }, 3000);
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    };

    fetchData();
  }, []); // Empty dependency array means this runs once when the component mounts

  const handleAlert = () => {
    setShowAlert(true);
  };

  return (
    <TurtleBotController>
      {({ movementhandlers, lidarData, collision }) => (
        <>
          {collision && (
            <div className="alert absolute top-1 items-center justify-center">
              <ObstaclePopUp />
            </div>
          )}
          <div className='relative' style={{ width: "1280px", height: "720px" }}>
            <VideoStream />
            <div className='absolute top-16 left-32'>
              <Modal movementhandlers={movementhandlers} handleAlert={handleAlert} />
            </div>
            <div className='absolute bottom-64 right-64'>
              <WarningButton movementhandlers={movementhandlers} handleAlert={handleAlert} />
            </div>
            <div className="flex items-center justify-center h-full">
              {showAlert && <AlertDefault />}
              {showBackendAlert && <BackendAlert message={backendAlertMessage} />}
            </div>
            <div className='absolute bottom-32 right-32'>
            <AiButton />
            </div>
            <div className='absolute bottom-32 left-28'>
              <div className='absolute bottom-16 left-14'><MoveForward movementhandlers={movementhandlers} lidarData={lidarData} /></div>
              <div className='flex'>
              <div className='absolute bottom-1'><MoveLeft movementhandlers={movementhandlers} /></div>
              <div className='absolute bottom-1 left-28'><MoveRight movementhandlers={movementhandlers} /></div>
              </div>
              <div className='absolute left-14'><MoveBackward movementhandlers={movementhandlers} lidarData={lidarData} /></div>
            </div>
          </div>
        </>
      )}
    </TurtleBotController>
  );
}

export default TeleopScreen;
