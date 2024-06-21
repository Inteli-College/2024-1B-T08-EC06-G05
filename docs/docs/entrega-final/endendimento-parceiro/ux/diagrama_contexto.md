---
title: Diagrama de contexto
sidebar_position: 4
---

# Diagrama de contexto

&emsp;&emsp;A criação de um diagrama de contexto é essencial no design de UX, pois visualiza o sistema e as trocas de informações entre ele e outras entidades, como usuários ou sistemas. Esse diagrama é crucial para planejar o fluxo de informações de um produto ou sistema, permitindo que os desenvolvedores estruturem uma arquitetura de informação que atenda aos objetivos do projeto e às necessidades dos usuários. No projeto da equipe SugarZ3ro, a elaboração de um diagrama de contexto foi uma escolha estratégica para pré-definir uma arquitetura inicial adequada.

<p style={{textAlign: 'center'}}>Figura 1 - Diagrama de contexto</p>

![Diagrama de contexto](../../../../static/img/sprint-1/diagrama_de_contexto.png)

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe SugarZ3ro</p>

&emsp;&emsp;Conforme descrito visualmente na figura 1, a aplicação web funciona como intermediária para todas as informações trocadas entre entidades na solução. Através dela, o operador pode receber a imagem da câmera acoplada ao robô e, com base nessa imagem, enviar instruções de movimentação ao robô. Essas instruções chegam ao robô que, por sua vez, envia continuamente a imagem atualizada da câmera para a aplicação web.

&emsp;&emsp;A partir da imagem capturada pela câmera, a aplicação web é capaz de gerar dados sobre a obstrução dos tubos dos reboilers checados pelo robô. Esses dados, por sua vez, serão transferidos por meio de uma API para uma dashboard própria dos sistemas da Atvos, na qual os usuários terão acesso visual sobre quais tubos de quais reboilers estão obstruídos.

&emsp;&emsp;Diante do exposto, percebe-se que o fluxo de informação está estruturado de forma que a aplicação web recebe e envia todos os dados envolvidos no controle do robô. Desse modo, é coerente e adequado que a persona Liz a utilize, uma vez que, como pessoa responsável por teleoperar o robô durante seu funcionamento, ela necessita visualizar a imagem atual captada pela câmera acoplada ao robô e movimentá-lo por meio de uma interface visual. Em relação à persona Carlos, o contato com esse fluxo de informação é pouco e indireto: devido ao seu baixo letramento digital, seu acesso à informação de quais tubos de quais reboilers estão obstruídos se daria por meio de um repasse de informação por um(a) funcionário(a) como Liz, que acessa o dashboard interno dos sistemas da Atvos.