import React from 'react';
import PowerIcon from '../assets/turnoff.svg';
import BtVoltarCinza from '../components/voltarButton/voltar';
import AIicon from '../assets/ligar-ia.svg';
import MoveIcon from '../assets/controle-carrinho.svg';


const HelpScreen = () => {
  return (
    <div className='flex items-center bg-black flex-col align-middle h-screen'>
      <div>
      <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 overflow-auto mt-32">
          <img src={PowerIcon} alt="Power Icon" />
          <p className="text-center my-4">
            Botão responsável por desligar o robô. Utilize-o caso queira desligar e retornar à página inicial.
          </p>
        </div>
      </div>
      {/* <div className="w-[715px] h-[300px] overflow-y-scroll mt-20">
        
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 overflow-auto">
          <img src={PowerIcon} alt="Power Icon" />
          <p className="text-center my-4">
            Botão responsável por desligar o robô. Utilize-o caso queira desligar e retornar à página inicial.
          </p>
        </div>
       
      </div> */}
      <div className='mt-10'>
        <BtVoltarCinza />
      </div>
    </div>
  );
};

export default HelpScreen;
