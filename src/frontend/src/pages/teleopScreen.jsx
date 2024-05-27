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
          <h1>Teleoperação</h1>
          {collision && <div className="alert">OBSTÁCULO DETECTADO!</div>}
          <AiButton />
          <VideoStream />
          <MoveForward movementhandlers={movementhandlers} collision={collision} />
          <MoveLeft movementhandlers={movementhandlers} collision={collision} />
          <MoveBackward movementhandlers={movementhandlers} collision={collision} />
          <MoveRight movementhandlers={movementhandlers} collision={collision} />
          <TurnoffButton movementhandlers={movementhandlers} />
        </>
      )}
    </TurtleBotController>
  );
}

export default TeleopScreen;
