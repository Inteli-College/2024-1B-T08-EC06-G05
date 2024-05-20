import React, { useState, useEffect } from "react";

function InfoPing() {
    const [ping, setPing] = useState(0);

    useEffect(() => {
        // Simulando a atualização do valor de ping
        const interval = setInterval(() => {
            const newPing = Math.floor(Math.random() * 1000); // Gerando um valor aleatório para o ping
            setPing(newPing);
        }, 1000);

        return () => clearInterval(interval); // Limpeza do intervalo ao desmontar o componente
    }, []);

    return (
        <div className="flex">
            
            <div className="w-0 h-0 
  border-t-[3vh] border-t-customBlue
  border-r-[3vh] border-r-customBlue
  border-l-[3vh] border-l-transparent
  border-b-[3vh] border-b-transparent">
            </div>

            <div className="h-5vh p-1 w-32 bg-customBlue flex items-center justify-end font-bold text-white text-xl font-sans">
                Ping: {ping}ms
            </div>

        </div>
    );
}

export default InfoPing;
