import React from 'react';
import { useNavigate } from 'react-router-dom';

const ReboilerInput = () => {
  const navigate = useNavigate();

  function reboilerInputAction(event) {
    event.preventDefault();
    const formData = new FormData(event.target);
    var reboilerID = formData.get("reboilerInputNumber");

    fetch('http://127.0.0.1:5000/post_reboiler_id', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ reboilerID: reboilerID }),
    })
      .then(response => response.json())
      .then(data => {
        console.log('Success:', data);
        navigate('/teleopScreen');
      })
      .catch((error) => {
        console.error('Error:', error);
      });
  }

  return (
    <div className="fixed inset-0 flex items-center justify-center z-50">
      <div className="absolute bg-white w-96 p-6 rounded-xl shadow-lg">
        <div className="flex flex-col text-center">
          <h2>Qual reboiler você está verificando?</h2>
          <form onSubmit={reboilerInputAction} className="flex flex-col space-y-4 justify-center pl-10 pr-10">
            <input type='text' name='reboilerInputNumber' className="p-2 border rounded" />
            <div className="flex justify-center space-x-16">
              <button type='submit' className="w-24 h-16 md:w-20 md:h-20 bg-orange-500 text-white rounded hover:bg-orange-600">Pronto</button>
              <button type='button' className="w-24 h-16 md:w-20 md:h-20 bg-gray-500 text-white rounded hover:bg-gray-600">Cancelar</button>
            </div>
          </form>
        </div>
      </div>
    </div>
  );
};

export default ReboilerInput;
