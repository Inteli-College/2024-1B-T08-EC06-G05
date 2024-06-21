---
title: Integração da movimentação
sidebar_position: 2
---

## Introdução 

&emsp;&emsp;Nessa seção será abordado como foi feita a integração do frontend com a movimentação do Turtlebot3 Burger. Nota-se que o frontend foi construído em react. Para mais informações de como o frontend foi criado, [clique aqui](./Interface.md).

## Mudanças do código da movimentação

&emsp;&emsp;Nessa seção será abordado as mudanças do código de movimentação feitas na Sprint 3. 

### Migração do Script em Python para Script em React

&emsp;&emsp;Visto que o frontend foi construído em react, houve a necessidade de criar um script de movimentação para um componente react, o componente `rosbridge_movement`. Esse script é responsável por fazer a comunicação do robô pelo frontend por meio do ROSBridge (uma ferramenta que permite a comunicação entre sistemas ROS (Robot Operating System) e aplicações externas via WebSockets, facilitando a integração de robôs com web serviços e outras tecnologias), assim enviando as informações publicadas no tópico `/cmd_vel`, tópico ROS responsável pela movimentação do turtlebot3 burger. Note que também foi desenvolvido um script de movimentação em Python, utilizando CLI para controlar o robô. Embora esse código não seja mais utilizado atualmente, ele continua presente no projeto para ser empregado como recurso de contingência, caso ocorra algum problema com a aplicação web.


## Rosbridge_movement
&emsp;&emsp;Como dito anteriormente, esse componente foi criado para controlar a comunicação da movimentação do frontend para o robô. Assim, toda vez que a página é carregada, é estabelecida a comunicação com o turtlebot e os comandos já podem ser usados. Note que essa comunicação está sendo feita por rede. Sendo assim, o script usa o IP do robô dado pela rede para se comunicar com o robô. Portanto, no script abaixo (retirado do componente rosbridge_movement, na constante TurtleBotController) deve ser alterada a URL para o IP do robô na rede que está presente. Para mais informações sobre como achar o IP do robô, [clique aqui](https://inteli-college.github.io/2024-1B-T08-EC06-G05/entrega-final/instalacao). Esse componente está encarregado de coletar todas as informações vindas dos botões e processar elas, assim mandando para o robô pelo tópico `/cmd_vel`. Para isso funcionar, esse componente leva para os componentes filhos o elemento `movementhandlers` que identifica a mensagem que os botões enviam e transforma ela no formato ideal de envio para o robô e publica essa informação no tópico, assim fazendo o robô se movimentar. Note que esse componente não é visível, é usado somente para coletar as informações dos botões e enviar elas ao turtlebot3.
```bash
    ros.current = new ROSLIB.Ros({
      url: 'ws://10.128.0.30:9090'
    });
```
## Botões
&emsp;&emsp;Todos os botões foram construídos com o intuito de entender se o usuário está mantendo pressionado o botão ou não, e criado para funcionar tanto para celular, quanto para computador. O projeto é mobile-first, por isso foi implementado para funcionar em celular, e só foi implementado para funcionar por computador por questão de comodidade dos testes no período de desenvolvimento. Note que as funções que os botões enviam são mandadas para o componente rosbridge_movement, que processa as funções e re-estrutura a informação para mandar para o turtlebot3. Esse sistema funciona por meio do conceito de filhos do react, sendo o componente rosbridge_movement um componente pai dos demais botões.
### Botões direcionais
&emsp;&emsp;Os botões direcionais são os componentes visíveis criados para controle do robô, sendo eles setas com direção norte, sul, leste e oeste. Todos os botões foram feitos com base em duas funções que identificam se o usuário apertou e soltou o componente, para assim o robô só andar enquanto o usuário estiver pressionando a tecla e parar quando o usuário soltar.
**Botão MoveRight**
&emsp;&emsp;O componente `direita.jsx` contem uma função que aceita movementhandlers como prop. Esta prop é um objeto que deve conter funções para manipular o movimento para a direita (right) e para parar o movimento (stop).
```javascript
const handleMouseDown = () => {
  console.log('Right button pressed');
  if (movementhandlers && movementhandlers.right) {
    movementhandlers.right();
  }
};
```
&emsp;&emsp;Esta função é chamada quando o botão é pressionado (click ou touch). Ela verifica se o movementhandlers está definido e se contém a função right. Se essas condições forem verdadeiras, a função right é chamada.
```javascript
const handleMouseUp = () => {
  if (movementhandlers && movementhandlers.stop) {
    movementhandlers.stop();
  }
};
```
&emsp;&emsp;Esta função é chamada quando o botão é liberado (mouse up ou touch end). Ela verifica se o movementhandlers está definido e se contém a função stop. Se essas condições forem verdadeiras, a função stop é chamada.
O mesmo foi aplicado para os outros componentes que controlam a direção da movimentação do robo. Essa logica foi aplicada para os seguintes componentes: `esquerda.jsx`, `frente.jsx`, `tras,jsx`.

<p style={{textAlign: 'center'}}>Figura 5 - Botões da tela de teleoperação</p>

<div style={{textAlign: 'center'}}>
    ![Tela de informações](../../../../static/img/sprint-5/mark_teleop.png)
</div>

### Botão de emergência
&emsp;&emsp;O botão de emergência é um componente visível criado para controle do robô, sendo ele um símbolo de exclamação. Esse botão tem funcionalidade de parar totalmente toda a comunicação com o robô, a fim dele parar totalmente. Caso queira re-estabelecer a conexão, é necessário recarregar a página. Esse botão foi construído para no momento que for pressionado ele chame uma função que corte a conexão com o robô. O componente respectivo a esse borão é encontrado no componente no arquivo `warning.jsx`.
O componente `WarningButton` é uma função que aceita `movementhandlers` e `handleAlert` como props. `movementhandlers` é um objeto que deve conter a função `turnoff`, e `handleAlert` é uma função que lida com alertas.
```javascript
const WarningButton = ({ movementhandlers, handleAlert }) => {
  const handleClick = () => {
    console.log('Turnoff button pressed');
    handleAlert();
    if (movementhandlers && movementhandlers.turnoff) {
      movementhandlers.turnoff();
    }
  };
  return (
    <button
      onClick={handleClick}
      className="bg-red-500 flex items-center justify-center p-4 rounded-full text-white active:bg-red-600 focus:outline-none">
      <img src={WarningIcon} alt="Alert" className="h-8 w-8" />
    </button>
  );
};
export default WarningButton;
```
`handleClick` é uma função é chamada quando o botão é clicado. Ela executa as seguintes ações:
- Loga uma mensagem no console indicando que o botão de desligamento foi pressionado.
- Chama a função handleAlert.
- Verifica se movementhandlers está definido e se contém a função turnoff. Se essas condições forem verdadeiras, a função turnoff é chamada.

## Conclusão
&emsp;&emsp;O código de movimentação foi desenvolvido completamente em componentes react. Há um componente principal, o rosbridge_movement que processa as informações que os botões enviam e re-estrutura elas para enviar ao robô. Note que essa comunicação é feita pelo IP do robô dado pela rede, então é importante que o usuário e o robô estejam na mesma rede. É importante notar que a movimentação é diretamente impactada pelo sistema de segurança, explicado em [Documentação do Sistema de Segurança](./seguranca.md).

