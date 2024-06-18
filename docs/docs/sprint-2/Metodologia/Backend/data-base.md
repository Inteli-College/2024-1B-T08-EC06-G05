---
title: Banco de Dados
sidebar_position: 2
---

# Banco de Dados em TinyDB

&emsp;&emsp;Nesta seção, é explicado como o banco de dados TinyDB foi desenvolvido e é planejado utilizar essa base de dados nas próximas sprints de desenvolvimento. É importante mencionar que o TinyDB é uma base de dados NoSQL, não relacional, e que utiliza arquivos JSON para armazenar informações. A escolha de um banco de dados não relacional se deve a fácil configuração, além de ser um projeto de prova de conceito, ou seja, um projeto de pequeno porte que não requer grandes armazenamentos de dados. No momento, é planejado utilizar apenas o arquivo `pipes.json`, mas pode surgir a necessidade de criar novos arquivos JSON para armazenar os dados devido à natureza não relacional do TinyDB. O arquivo `pipes.json` pode ser encontrado na pasta src/data-base.

## Arquivos

&emsp;&emsp;Dado o fato de que o TinyDB não trabalha com o sistema de tabelas, pode ser necessário criar novos arquivos JSON para armazenar informações. Atualmente, nosso projeto conta apenas com um arquivo chamado pipes.json, que armazena os dados dos canos dos reboilers. Este arquivo contém as seguintes informações:

- **id**: identificador do cano
- **status**: boleano para identificar se está limpo (True) ou sujo (False), basedo na análise de visão computacional.
- **id-boiler**: identificador do reboiler
- **dirty-grade**: porcentagem de sujeira
- **datetime**: data e horário que as informações foram coletadas

O formato dos dados de um cano é o seguinte:

```json
{
    "id": <int>,
    "status": <boolean>,
    "id-boiler": <int>,
    "dirty-grade": <int>,
    "datetime": <str>
}
```

&emsp;&emsp;A escolha das informações contidas nesta base de dados foi pensada com base nas necessidades atuais. Como o projeto foi inicialmente projetado para apenas guardar se os canos estão sujos ou não e uma identificação deles, foi decidido que somente haveria informações básicas de identificação dos canos para monitoramento, quais já foram analisados, o resultado da análise, o identificador do boiler e a data e horário em que essa informação foi salva no banco de dados. No momento, não é preciso de outras informações sobre os boilers nem informações sobre o usuário do sistema, seguindo os pedidos e observações do cliente em relação às necessidades do sistema.

&emsp;&emsp;Esta é a primeira versão da base de dados do projeto e pode sofrer alterações conforme novas necessidades. Foram realizados testes para verificar se os dados estavam sendo corretamente enviados para o arquivo JSON, utilizando dados para "dirty-grade" gerados aleatoriamente e definindo um máximo de 30% de sujeira para que o cano seja considerado limpo. A utilização do método `random` foi exclusiva para o primeiro teste, o que significa que ele não será utilizado na versão final do banco de dados. Quando tivermos dados gerados pela análise computacional, eles serão inseridos no lugar dos atuais valores gerados pelo `random`. O percentual de 30% foi escolhido apenas para teste no momento atual. No futuro, decidiremos junto ao parceiro o que ele considera um cano sujo ou limpo. A versão do banco de dados atual está adaptada para testes e não é a versão final do banco de dados principal.







