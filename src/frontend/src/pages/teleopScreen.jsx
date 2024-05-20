import InfoPing from "../components/infoPing/infoPing"
import AiButton from "../components/aiButton/aiButton"
import MovFrente from "../components/movimentacao/frente"
import MovDireita from "../components/movimentacao/direita"
import MovEsquerda from "../components/movimentacao/esquerda"
import MovTras from "../components/movimentacao/tras"

function TeleopScreen() {
  return (
    <>
      <h1>Teleoperação</h1>
      <InfoPing />
      <AiButton />
      <MovDireita />
      <WarningButton />
      <TurnOffButton />
    </>
  )
}

export default TeleopScreen