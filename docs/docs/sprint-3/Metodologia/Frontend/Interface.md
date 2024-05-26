# Transformação do Wireframe em Interface

## Introdução

Durante a terceira sprint do projeto, a equipe SugarZ3ro concentrou seus esforços no desenvolvimento da interface gráfica. Conforme descrito na proposta inicial de arquitetura do projeto, a equipe utilizou React para construir a interface do usuário, permitindo uma interação intuitiva e eficiente com o sistema. A estilização foi realizada utilizando Tailwind CSS, que oferece uma abordagem utilitária para a aplicação de estilos, facilitando a manutenção e escalabilidade do projeto.

## Tecnologias Utilizadas

### React
- React é uma biblioteca JavaScript para construir interfaces de usuário de forma declarativa. React facilita a criação de componentes interativos e reutilizáveis, permitindo desenvolver aplicações complexas de forma mais eficiente. É amplamente usada para desenvolver aplicações web e móveis.

### JSX
- JSX é uma extensão de sintaxe para JavaScript usada junto com React. Permite escrever a estrutura do HTML dentro do código JavaScript, tornando o código mais legível e expressivo. JSX transforma elementos de HTML em funções de JavaScript.

### Tailwind CSS
- Tailwind CSS é um framework de CSS que utiliza uma abordagem utilitária para estilos. Ao invés de escrever CSS tradicional, os desenvolvedores podem aplicar classes utilitárias diretamente em elementos HTML, o que simplifica a manutenção de estilos e acelera o processo de desenvolvimento.

## Desenvolvimento

### Tela de Início

#### Implementação

```js
jsx

const MainMenu = () => {
  return (
    <div className="bg-black h-screen flex flex-col items-center justify-center">
      <div className="mb-8">
        <img src={SugarZ3roLogo} alt="Logo SugarZ3ro" />
      </div>
      <div className='flex flex-col items-center space-y-4 md:flex-row md:space-x-10 md:mt-28'>
        <button className="bg-white text-black py-3 px-10 rounded-full font-bold hover:bg-gray-400 mt-2 md:mt-0">Start</button>
        <button className="bg-transparent text-white border-none hover:text-slate-400 mt-4 md:mt-0" style={{ borderRadius: '100%', borderWidth: '2px'}}>
            <FontAwesomeIcon icon={faCircleInfo} style={{ fontSize: '2.5rem'}}/>
        </button>
      </div>
    </div>
  );
};

export default MainMenu;
```


*Estilos Tailwind:*
- bg-black, text-black, text-white: Define as cores de fundo e de texto.
- h-screen: Altura total da tela.
- flex, flex-col: Layout flexível com direção de coluna.
- items-center, justify-center: Centralização dos elementos.
- rounded-full, hover:bg-gray-400: Estilos para arredondamento e interação de hover.
- py-3, px-10: Padding vertical e horizontal.

### Tela de Informações


#### Implementação

```js
jsx
const HelpScreen = () => {
  return (
    <div className='flex items-center bg-black flex-col align-middle h-screen'>
      <div>
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 overflow-auto mt-32">
          <img src={PowerIcon} alt="Power Icon" />
          <p className="text-center my-4">
            Botão responsável por desligar o robô. Utilize-o caso queira desligar e retornar à página inicial.
          </p>
        </div>
      </div>
      {/* <div className="w-[715px] h-[300px] overflow-y-scroll mt-20">
        
        <div className="bg-black text-white w-full flex flex-col items-center justify-center border-2 border-white rounded p-4 overflow-auto">
          <img src={PowerIcon} alt="Power Icon" />
          <p className="text-center my-4">
            Botão responsável por desligar o robô. Utilize-o caso queira desligar e retornar à página inicial.
          </p>
        </div>
       
      </div> */}
      <div className='mt-10'>
        <BtVoltarCinza />
      </div>
    </div>
  );
};
```


*Estilos Tailwind:*
- bg-black, text-white: Define as cores de fundo e de texto.
- flex, flex-col: Layout flexível com direção de coluna.
- items-center, justify-center: Centralização dos elementos na vertical e horizontal.
- border-2, border-white: Aplica uma borda de 2px na cor branca.
- rounded: Arredondamento das bordas do container.
- p-4: Padding interno de 1rem.
- overflow-auto: Permite a rolagem automática dentro do elemento se o conteúdo exceder o tamanho do container.
- mt-32: Margem superior de 8rem.

### Tela de Controle


#### Implementação

```js
jsx
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
```



*Descrição dos Componentes:*
- InfoPing: Mostra informações sobre o status da conexão ou outros indicadores de rede.
- AiButton: Permite ao usuário ativar funcionalidades de inteligência artificial para análise de imagem ou dados em tempo real.
- WarningButton: Serve como um botão de emergência, que pode ser usado para alertar sobre condições críticas ou para parar o robô imediatamente.
- TurnOffButton: Este botão é responsável por desligar o robô, encerrando as operações e retornando à tela inicial.
- BtVoltarCinza: Um botão para voltar à tela anterior ou principal.

## Conclusão

O processo de transformação de wireframes para componentes interativos foi guiado pelo uso de React e Tailwind CSS, permitindo um desenvolvimento ágil e modular. Esta abordagem assegurou que cada componente pudesse ser desenvolvido, testado e estilizado em uma pagina de forma independente.