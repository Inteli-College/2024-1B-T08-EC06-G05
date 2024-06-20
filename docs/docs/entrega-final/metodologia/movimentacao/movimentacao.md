---
title: Integração da movimentação
sidebar_position: 2
---

## Introdução 

&emsp;&emsp;Nessa seção será abordado como foi feita a integração do frontend com a movimentação do Turtlebot3 Burger. Nota-se que o frontend foi construído em react. Para mais informações de como o frontend foi criado, [clique aqui](../Interface.md).

## Mudanças do código da movimentação

&emsp;&emsp;Nessa seção será abordado as mudanças do código de movimentação feitas na Sprint 3. 

### Migração do Script Python para um Script em React

&emsp;&emsp;Visto que o frontend foi construído em react, houve a necessidade de alterar o script de movimentação para um componente react. Assim foi criado o componente rosbridge_movement. Esse script é responsável por fazer a comunicação do robô pelo frontend por meio do ROSBridge (uma ferramenta que permite a comunicação entre sistemas ROS (Robot Operating System) e aplicações externas via WebSockets, facilitando a integração de robôs com web serviços e outras tecnologias), assim enviando as informações publicadas no tópico `/cmd_vel`, tópico ROS responsável pela movimentação do turtlebot3 burger. Note que o script de movimentação feito em Python continua presente no projeto, com o intuito de ser utilizado em última instância caso algum problema tenha ocorrido com a aplicação WEB.

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

### Botão de emergência

&emsp;&emsp;O botão de emergência é um componente visível criado para controle do robô, sendo ele um símbolo de exclamação. Esse botão tem funcionalidade de parar totalmente toda a comunicação com o robô, a fim dele parar totalmente. Caso queira re-estabelecer a conexão, é necessário recarregar a página. Esse botão foi construído para no momento que for pressionado ele chame uma função que corte a conexão com o robô. 

## Conclusão

&emsp;&emsp;O código de movimentação foi migrado completamente para componentes em react. Há um componente principal, o rosbridge_movement que processa as informações que os botões enviam e re-estrutura elas para enviar ao robô. Note que essa comunicação é feita pelo IP do robô dado pela rede, então é importante que o usuário e o robô estejam na mesma rede.
