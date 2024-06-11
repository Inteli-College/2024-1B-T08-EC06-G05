import React, { useState } from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/rosbridge_movement/rosbridge_movement';
import MoveRight from '../components/movimentacao/direita';
import MoveLeft from '../components/movimentacao/esquerda';
import MoveForward from '../components/movimentacao/frente';
import MoveBackward from '../components/movimentacao/tras';
import TurnoffButton from '../components/turnoffbutton/turnoff';
import WarningButton from '../components/warningButton/warning';
import ObstaclePopUp from '../components/obstaclePopUp/obstaclePopUp';
import { Alert } from "@material-tailwind/react";

export function AlertDefault() {
  return (
    <Alert className="bg-white text-black p-6 text-xl w-full max-w-lg mx-auto mt-4 shadow-lg rounded-lg">
      Botão de emergência ativado!! Para movimentar o robô novamente, recarregue a página!
    </Alert>
  );
}

function TeleopScreen() {
  const [showAlert, setShowAlert] = useState(false);

  const handleAlert = () => {
    setShowAlert(true);
  };

  return (
    <TurtleBotController>
      {({ movementhandlers, collision }) => (
        <>
          {collision && (
            <div className="alert absolute top-1 items-center justify-center">
              <ObstaclePopUp />
            </div>
          )}
          <div className='relative' style={{ width: "1280px", height: "720px" }}>
            <VideoStream />
            <div className='absolute top-16 left-32'>
              <TurnoffButton movementhandlers={movementhandlers} />
            </div>
            <div className='absolute bottom-64 right-64'>
              <WarningButton movementhandlers={movementhandlers} handleAlert={handleAlert} />
            </div>
            <div className="flex items-center justify-center h-full">
              {showAlert && <AlertDefault />}
            </div>
            <div className='absolute bottom-32 right-32'>
              <AiButton />
            </div>
            <div className='absolute bottom-32 left-28'>
              <div className='absolute bottom-16 left-14'><MoveForward movementhandlers={movementhandlers} collision={collision} /></div>
              <div className='flex'>
                <div className='absolute bottom-1'><MoveLeft movementhandlers={movementhandlers} collision={collision} /></div>
                <div className='absolute bottom-1 left-28'><MoveRight movementhandlers={movementhandlers} collision={collision} /></div>
              </div>
              <div className='absolute left-14'><MoveBackward movementhandlers={movementhandlers} collision={collision} /></div>
            </div>
          </div>
        </>
      )}
    </TurtleBotController>
  );
}

export default TeleopScreen;
