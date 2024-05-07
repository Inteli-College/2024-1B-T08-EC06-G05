---
title: Banco de Dados
sidebar_position: 1
---

# Banco de Dados em TinyDB

&emsp;&emsp;Nesta seção, é explicado como o banco de dados TinyDB foi desenvolvido e como planejamos utilizar essa base de dados nas próximas sprints de desenvolvimento. É importante mencionar que o TinyDB é uma base de dados NoSQL, não relacional, e que utiliza arquivos JSON para armazenar informações. Esse foi o banco de dados escolhido pela simplicidade e fácil configuração, além de ser um projeto de prova de conceito, ou seja, um projeto de pequeno porte que não requer grandes armazenamentos de dados. No momento, planejamos utilizar apenas o arquivo `pipes.json`, mas pode surgir a necessidade de criar novos arquivos JSON para armazenar os dados devido à natureza não relacional do TinyDB. O arquivo `pipes.json` pode ser encontrado na pasta src/data-base.

## Arquivos

&emsp;&emsp;Dado o fato de que o TinyDB não trabalha com o sistema de tabelas, pode ser necessário criar novos arquivos JSON para armazenar informações. Atualmente, nosso projeto conta apenas com um arquivo chamado pipes.json, que armazena os dados dos canos dos reboilers. Este arquivo contém as seguintes informações:

- **id**: identificador do cano
- **status**: boleano para identificar se está limpo ou sujo (basedo na futura analise de visão computacional)
- **id-boiler**: identificador do reboiler
- **dirty-grade**: nota de sujeira
- **datetime**: data e horario que as informações foram coletadas

O formato dos dados de um cano é o seguinte:

```
{
    "id": <int>,
    "status": <boolean>,
    "id-boiler": <int>,
    "dirty-grade": <int>,
    "datetime": <str>
}
```

&emsp;&emsp;O arquivo pipes.json armazena dados sobre o cano, incluindo seu identificador próprio, o status (limpo ou sujo de acordo com a análise computacional), o identificador do reboiler onde o cano está, a nota definida pelo sistema de análise computacional em relação à sujeira do cano e a data e horário em que essas informações foram enviadas para o banco de dados.

&emsp;&emsp;Esta é a primeira versão da base de dados do projeto e pode sofrer alterações conforme novas necessidades. Já realizamos testes para verificar se os dados estavam sendo corretamente enviados para o arquivo JSON, utilizando dados para "dirty-grade" gerados aleatoriamente e definindo um máximo de 30% de sujeira para que o cano seja considerado limpo.







