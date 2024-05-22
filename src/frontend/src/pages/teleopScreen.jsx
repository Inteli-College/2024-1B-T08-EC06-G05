import InfoPing from "../components/infoPing/infoPing"
import AiButton from "../components/aiButton/aiButton"
import MovFrente from "../components/movimentacao/frente"
import MovDireita from "../components/movimentacao/direita"
import MovEsquerda from "../components/movimentacao/esquerda"
import MovTras from "../components/movimentacao/tras"
import WarningButton from "../components/warningButton/warning"
import TurnOffButton from "../components/turnoffbutton/turnoff"
import BtVoltarCinza from "../components/voltarButton/voltar"

function TeleopScreen() {
  return (
    <>
      <h1>Teleoperação</h1>
      <InfoPing />
      <AiButton />
      <WarningButton />
      <TurnOffButton />
      <BtVoltarCinza />
    </>
  )
}

export default TeleopScreen