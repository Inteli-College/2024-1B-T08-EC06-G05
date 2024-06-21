import React, { useState, useEffect } from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/rosbridge_movement/rosbridge_movement';
import MoveRight from '../components/movimentacao/direita';
import MoveLeft from '../components/movimentacao/esquerda';
import MoveForward from '../components/movimentacao/frente';
import MoveBackward from '../components/movimentacao/tras';
import TurnoffButton from '../components/turnoffbutton/turnoff';
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
  const [aiButtonState, setAiButtonState] = useState(false);
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
      {({ movementhandlers, collision }) => (
        <>
          {collision && (
            <div className="fixed inset-0 flex items-center justify-center z-50">
              <ObstaclePopUp />
            </div>
          )}
          <div className='relative w-full h-auto max-w-screen-xl mx-auto'>
            <VideoStream />
            <div className='fixed top-4 left-4 lg:top-16 lg:left-32'>
              <TurnoffButton movementhandlers={movementhandlers} />
            </div>
            <div className='fixed bottom-16 mr-28 mb-28 right-16 lg:bottom-36 lg:right-36 lg:mr-32 lg:mb-28'>
              <WarningButton movementhandlers={movementhandlers} />
            </div>
            <div className='fixed bottom-16 right-16 lg:bottom-32 lg:right-32'>
              <AiButton />
            </div>
            <div className='fixed bottom-32 left-60 transform -translate-x-1/2'>
              <div className='flex flex-col items-center space-y-2'>
                <MoveForward movementhandlers={movementhandlers} collision={collision} />
                <div className='flex space-x-2'>
                  <MoveLeft movementhandlers={movementhandlers} collision={collision} />
                  <MoveBackward movementhandlers={movementhandlers} collision={collision} />
                  <MoveRight movementhandlers={movementhandlers} collision={collision} />
                </div>
              </div>
            </div>
          </div>
        </>
      )}
    </TurtleBotController>
  );
}
export default TeleopScreen;