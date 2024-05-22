import React from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/rosbridge_movement/rosbridge_movement';
import MoveRight from '../components/movimentacao/direita';
import MoveLeft from '../components/movimentacao/esquerda';
import MoveForward from '../components/movimentacao/frente';
import StopButton from '../components/movimentacao/stop';
import TurnoffButton from '../components/turnoffbutton/turnoff';

function TeleopScreen() {
  return (
    <>
      <h1>Teleoperação</h1>
      <AiButton />
      <VideoStream />
      <TurtleBotController>
        <div className="flex flex-col items-center">
          <h2 className="text-xl mb-4">TurtleBot Controller</h2>
          <div className="grid grid-cols-3 gap-4">
            <div></div>
            <MoveForward />
            <div></div>
            <MoveLeft />
            <StopButton />
            <MoveRight />
            <div></div>
            <TurnoffButton />
            <div></div>
          </div>
        </div>
      </TurtleBotController>
    </>
  );
}

export default TeleopScreen;
