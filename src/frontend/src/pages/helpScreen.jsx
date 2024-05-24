import React from 'react';
import PowerIcon from '../assets/turnoff.svg';
import BtVoltarCinza from '../components/voltarButton/voltar';
import AIicon from '../assets/ligar-ia.svg';
import MoveIcon from '../assets/controle-carrinho.svg';
import BtParada from '../assets/bt-parada.svg';

function HelpScreen() {
  return (
    <div className='flex items-center bg-black flex-col align-middle h-screen'>
      <div className='overflow-y-scroll h-2/4 mt-20'>
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4">
          <img src={PowerIcon} alt="Power button" />
          <p className="text-center my-4">
            Botão responsável por desligar o robô. Utilize-o caso queira desligar e retornar à página inicial.
          </p>
        </div>
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 mt-32">
          <img src={AIicon} alt="AI button" />
          <p className="text-center my-4">
            Botão responsável por ligar a IA de reconhecimento de sujeira. Ligue-o quando desejar analisar uma imagem.
          </p>
        </div>
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 mt-32">
          <img src={MoveIcon} alt="Move button" />
          <p className="text-center my-4">
            Botão responsável pela movimentação do robô. Utilize-o para movê-lo para a posição desejada.
          </p>
        </div>
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 mt-32">
          <img src={BtParada} alt="Emergency stop button" />
          <p className="text-center my-4">
            Botão de parada de emergência. Utilizá-lo fará com que o robô pare imediatamente e não receba mais comandos. Utilize-o em casos em que seja necessário parar o robô rápido.
          </p>
        </div>
      </div>
      <div className='mt-10'>
        <BtVoltarCinza />
      </div>
    </div>
  );
}

export default HelpScreen;

