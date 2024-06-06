---
title: Implementação da Visão computacional
sidebar-position: 1
---

# Introdução

&emsp;&emsp;Nessa seção será descrito a forma de como a Inteligência Artificial foi criada desde a decisão das tecnologias utilizadas, escolha do dataset até a criação e implementação da IA, juntamente a comparação dos modelos criados e qual modelo foi escolhido para ser utilizado no projeto. Note que o parceiro não disponibilizou dados dos canos para serem utilizados, assim houve a necessidade de encontrar um dataset análogo ao problema que foi apresentado.

## Tecnologias Utilizadas

&emsp;&emsp;Nessa seção será descrito a tecnologia que foi escolhida para criar os modelos de inteligência artificial e o porquê dessa escolha.

&emsp;&emsp;Esse projeto foi desenvolvido para identificar impurezas presentes em canos de um reboiler. Como falado anteriormente, o parceiro não disponibilizou uma base de dados para ser usada, logo houve a necessidade de escolher um modelo de treinamento de IA que fosse geral e identificasse multiplos objetos e formas diferentes. 

&emsp;&emsp;Assim, foi escolhido utilizar o YOLOv8 , que é uma versão avançada do algoritmo YOLO para detecção em tempo real de objetos em imagens e vídeos, assim servindo para o projeto que utiliza imagens para detecção de impurezas. Apesar de existir versões mais atualizadas da YOLO (v9 e v10), foi escolhida a YOLOv8 por sua facilidade e conformidade de uso, pois YOLOv10 foi lançado enquanto esse projeto estava sendo desenvolvido, sendo uma tecnologia muito nova e com falta de casos de uso. Além disso, o modo de utilização do YOLOv9 implica complicações no modo de instalar, já que é necessário clonar um repositório e lidar com várias dependências, especialmente considerando que estamos utilizando computadores externos para processar nosso modelo de IA e o TurtleBot3. Portanto, optamos pelo YOLOv8, que é mais fácil de instalar e atende bem às necessidades do nosso projeto.

&emsp;&emsp;Ademais, foi feito um processo de fine-tuning em cima do modelo do YOLOv8 para identificar impurezas dentro de tubos de uma forma melhorada. Fine-tuning é um processo de usar um modelo pronto e re-treinar o modelo com um novo dataset para fazer com que a IA identifique melhor algo em específico, que estará presente no novo dataset escolhido.

## Dataset

&emsp;&emsp;Nessa seção será abordado a escolha do dataset escolhido para realizar o fine-tuning e o porquê dessa escolha.

&emsp;&emsp;Como tinha sido definido que seria utilizado o modelo YOLO para construção da Inteligência Artificial, foi utilizado [esse site](https://public.roboflow.com) para achar um dataset anóloga ao problema do projeto. Esse site foi escolhido por causa de apresentar datasets de conteúdos diversos como open source e a disponibilidade para download no modo de arquivo que o YOLOv8 utiliza. 

&emsp;&emsp;O dataset escolhido foi [esse dataset](https://universe.roboflow.com/purdue-university-niruh/precision-ag-subterranean/browse?queryText=&pageSize=50&startingIndex=0&browseQuery=true), pois apresenta impurezas e obstáculos em canos que estão claros e escuros, além de conter imagens sem impurezas para mostrar ao modelo o que seria um cano limpo. Está sendo usado todas as 631 imagens presentes no dataset, dividas em 549 imagens de treinamento, 50 imagens de validação e 32 imagens de teste. Note que todas as imagens presentes tem dimensão de 640x640.

## Criação e comparação dos modelos de IA

&emsp;&emsp;Nessa seção será abordado as inteligências artificiais criadas, além de demonstrar os detalhes do modo de treinamento e seus resultados, baseados no modelo [MarioLembreDePerguntarAIsa]

## Modelo escolhido

&emsp;&emsp;Nessã seção será abordado o modelo de inteligência artificial escolhido para ser usado no projeto, além dos motivos que levaram a essa escolha.