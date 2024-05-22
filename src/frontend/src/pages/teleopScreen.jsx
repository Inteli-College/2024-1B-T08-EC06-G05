import React from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/TurtleBotController';
import MoveRight from '../components/MoveRight';
import MoveLeft from '../components/MoveLeft';
import MoveForward from '../components/MoveForward';
import StopButton from '../components/StopButton';
import TurnoffButton from '../components/TurnoffButton';

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
            <MoveForward />
            <MoveLeft />
            <StopButton />
            <MoveRight />
            <TurnoffButton />
          </div>
        </div>
      </TurtleBotController>
    </>
  );
}

export default TeleopScreen;
