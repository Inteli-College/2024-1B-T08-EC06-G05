---
title: Documentação da API
sidebar_position: 1
---

# Documentação da API do projeto

## Introdução

&emsp;&emsp;API (Application Programming Interface) refere-se a um conjunto de funções e comandos padronizados que permitem a comunicação e integração entre diferentes sistemas de maneira facilitada através da programação. Em paráfrase, com APIs, é possível receber e enviar dados entre sistemas sem a necessidade de criar um meio de comunicação entre estes a partir do zero.

&emsp;&emsp;Como forma de atender aos interesses da Atvos, empresa parceira do projeto, a equipe SugarZ3ro desenvolveu uma API integrada à aplicação web existente na solução para que seja possível manipular com mais facilidade os dados referentes ao nível de sujeira dos canos de reboilers. Desse modo, a empresa poderá, de maneira prática, obter tais dados e exibí-los em forma de gráficos em sua plataforma interna própria para tal.

&emsp;&emsp;A API do projeto foi construída em Python com base no micro-framework Flask. Ela faz parte do backend da aplicação web do sistema e interage com o frontend desta, com o banco de dados e com a inteligência artificial utilizada para analisar os canos dos reboilers.

## Documentação das rotas

&emsp;&emsp;Cada uma das rotas que compõem a API foi documentada com uso do Postman, software que facilita o processo de criação, testagem e documentação de APIs. A documentação da API pode ser acessada [clicando aqui](https://documenter.getpostman.com/view/27301833/2sA3XLEPvZ).

## Testagem das rotas

&emsp;&emsp;Cada uma das rotas que compõem a API também foi testada com uso do Postman. Foram realizados dois tipos de testes: manuais e automáticos.

### Testes manuais

&emsp;&emsp;Os testes manuais consistiram na criação manual de requisições para cada uma das rotas existentes na API. Cada uma das requisições foi criada e executada junto à execução local do servidor em Flask com uma taxa de 100% de sucesso (nenhuma das requisições falhou).

&emsp;&emsp;A seguir, há uma sequência de figuras com capturas de tela da testagem das rotas, mostrando qual rota foi testada, o corpo e parâmetros/argumentos passados para as requisições e o que foi retornado por elas, conforme descrito na documentação das rotas.