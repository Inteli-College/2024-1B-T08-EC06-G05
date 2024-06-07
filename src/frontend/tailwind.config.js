/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/App.jsx",
    "./src/main.jsx",
    "./src/components/buttonsPopUp/popUpCinza.jsx",
    "./src/components/buttonsPopUp/popUpLaranja.jsx",
    "./src/components/movimentacao/frente.jsx",
    "./src/components/movimentacao/tras.jsx",
    "./src/components/movimentacao/direita.jsx",
    "./src/components/movimentacao/esquerda.jsx",
    "./src/components/aiButton/aiButton.jsx",
    "./src/components/infoPing/infoPing.jsx",
    "./src/pages/helpScreen.jsx",
    "./src/pages/mainMenu.jsx",
    "./src/pages/teleopScreen.jsx",
    "./src/components/voltarButton/voltar.jsx",
    "./src/components/turnoffButton/turnoff.jsx",
    "./src/components/warningButton/warning.jsx",
    "./src/components/OverlayTurnOff/modal.jsx",
    "./src/components/VideoStream/videoStream.jsx",
    "./src/components/camera/camera.jsx",
    "./src/components/reboilerInput/reboilerInput.jsx",
  ],
  theme: {
    extend: {
      spacing: {
        '-2': '-0.5rem',
      },
      colors: {
        customBlue: '#49748C',
      }
    },
  },
  plugins: [],
}