import React, { useState, useEffect } from "react";
import './infoPing.css';

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
        <div className="alignmentClass">
            <div className="trianglePart"></div>
            <div className="rectanglePart">
                Ping: {ping}ms
            </div>
        </div>
    );
}

export default InfoPing;
