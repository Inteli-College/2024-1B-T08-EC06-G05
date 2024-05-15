import React from 'react'
import ReactDOM from 'react-dom/client'
import './index.css'
import MainMenu from './pages/mainMenu/mainMenu.jsx';
import TeleopScreen from './pages/teleopScreen/teleopScreen.jsx';
import { createBrowserRouter, RouterProvider } from 'react-router-dom';
import HelpScreen from './pages/helpScreen/helpScreen.jsx';

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
  </React.StrictMode>
);