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

function TeleopScreen() {
  return (
    <TurtleBotController>
      {({ movementhandlers, collision }) => (
        <>
          {collision && <div className="alert">OBSTÁCULO DETECTADO!</div>}
          <VideoStream />
          <AiButton />
          <TurnoffButton movementhandlers={movementhandlers} />
          <div className='mt-96 ml-10'>
            <div className='ml-9'>
          <MoveForward movementhandlers={movementhandlers} collision={collision} />
          </div>
          <MoveLeft movementhandlers={movementhandlers} collision={collision} />
          <MoveBackward movementhandlers={movementhandlers} collision={collision} />
          <MoveRight movementhandlers={movementhandlers} collision={collision} />
          </div>
        </>
      )}
    </TurtleBotController>
  );
}

export default TeleopScreen;
