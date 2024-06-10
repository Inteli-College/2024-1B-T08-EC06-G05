import React from 'react'
import ReactDOM from 'react-dom/client'
import './index.css'
import MainMenu from './pages/mainMenu.jsx';
import TeleopScreen from './pages/teleopScreen.jsx';
import { createBrowserRouter, RouterProvider } from 'react-router-dom';
import HelpScreen from './pages/helpScreen.jsx';
import EmergencyPopUp from './components/emergencyPopUp/emergencyPopUp.jsx';

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
  },
  {
    path: '/emergencyPopUp',
    element: <EmergencyPopUp />
  }
]);

// Renderização do aplicativo com o provedor de roteador
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>
);
