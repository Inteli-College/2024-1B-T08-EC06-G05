import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import TurnOffButton from '../turnoffbutton/turnoff';

const Modal = () => {
    const [showModal, setShowModal] = useState(false);
  
    const navigate = useNavigate()

    const Desligar = () => {
        navigate("/")
    }
    return (
      <>
        <TurnOffButton onClick={() => setShowModal(true)} />
        {showModal ? (
        <div className="fixed inset-0 flex items-center justify-center z-50">
          <div className="absolute bg-white w-96 p-6 rounded-xl shadow-lg">
            <div className="flex flex-col text-center">
              <h2>Tem certeza que deseja desligar o robô? </h2>
              <div className="flex flex-row justify-center gap-4 mt-5">
                <button className="bg-gray-300 text-black font-bold px-4 py-2 rounded" onClick={() => setShowModal(false)}>Cancelar</button>
                <button className="bg-orange-400 text-white font-bold px-4 py-2 rounded" onClick={Desligar}>Desligar</button>
              </div>
            </div>
          </div>
        </div>
      ) : null}
      </>
    );
  };
  
  export default Modal;