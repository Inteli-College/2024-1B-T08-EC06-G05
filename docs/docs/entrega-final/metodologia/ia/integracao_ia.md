---
title: Integração da IA
sidebar-position: 3
---

# Integração da Inteligência Artificial

&emsp;&emsp;Conforme descrito na proposta de solução, o sistema desenvolvido pela equipe SugarZ3ro conta com uma análise de imagens feita por IA (Inteligência Artificial). De maneira mais específica, o robô utilizado pela equipe recebeu uma câmera acoplada, a qual será responsável por captar imagens em tempo real dos canos dos reboilers contidos nas usinas da Atvos. A IA, como dito na [seção referente](./inteligencia_artificial.md), terá a função de analisar as imagens dos canos captadas e determinar, através de uma rede neural pré-treinada com YOLO, se o cano da imagem está sujo ou não.

&emsp;&emsp;Para que essa funcionalidade se adequasse à necessidade da empresa parceira e ao contexto dos usuários finais do protótipo, este modelo de IA desenvolvido na sprint 4 foi integrado à interface de usuário na sprint 5. Desse modo, o usuário pode ativar a detecção de impurezas a qualquer momento apertando o respectivo botão na tela de teloperação.

## 