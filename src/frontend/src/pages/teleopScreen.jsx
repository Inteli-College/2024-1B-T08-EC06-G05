// src/components/TeleopScreen.js
import React from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/rosbridge_movement/rosbridge_movement';
import MoveRight from '../components/movimentacao/direita';
import MoveLeft from '../components/movimentacao/esquerda';
import MoveForward from '../components/movimentacao/frente';
import MoveBackward from '../components/movimentacao/tras';
import TurnoffButton from '../components/turnoffbutton/turnoff';
import WarningButton from '../components/warningButton/warning'

function TeleopScreen() {
  return (
    <TurtleBotController>
      {({ movementhandlers, collision }) => (
        <>
          {collision && <div className="alert">OBSTÁCULO DETECTADO!</div>}
          <div className='relative' style={{width: "1280px", height: "720px"}}>
          <VideoStream />
          <div className='absolute top-16 left-32'>
          <TurnoffButton movementhandlers={movementhandlers} />
          </div>
          <div className='absolute bottom-64 right-64'>
              <WarningButton />
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
