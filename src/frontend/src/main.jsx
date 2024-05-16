import React from 'react'
import ReactDOM from 'react-dom/client'
import './index.css'
import MainMenu from './pages/mainMenu/mainMenu.jsx';
import TeleopScreen from './pages/teleopScreen/teleopScreen.jsx';
import { createBrowserRouter, RouterProvider } from 'react-router-dom';
import HelpScreen from './pages/helpScreen/helpScreen.jsx';
import BtPopUpCinza from './components/buttonsPopUp/popUpCinza.jsx'
import BtPopUpLaranja from './components/buttonsPopUp/popUpLaranja.jsx'
import MovTras from './components/movimentacao/tras.jsx'
import MovDireita from './components/movimentacao/direita.jsx'
import MovFrente from './components/movimentacao/frente.jsx'
import MovEsquerda from './components/movimentacao/esquerda.jsx'

// Criação do roteador com as rotas definidas
const router = createBrowserRouter([
  {
    path: '/',
    element: <MainMenu/>
  },
  {
    path: '/teleopScreen',
    element: <TeleopScreen/>
  },
  {
    path: '/helpScreen',
    element: <HelpScreen />
  }
]);

// Renderização do aplicativo com o provedor de roteador
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
    <BtPopUpCinza />
    <BtPopUpLaranja />
    <MovTras />
    <MovDireita />
    <MovFrente />
    <MovEsquerda />
  </React.StrictMode>
);
