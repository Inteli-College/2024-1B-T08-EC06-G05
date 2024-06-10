---
title: Banco de Dados
sidebar_position: 1
---

# Banco de Dados em TinyDB

&emsp;&emsp;Nesta seção, é explicado como o banco de dados da solução foi desenvolvido e como é planejado utilizá-lo no desenvolvimento do projeto. É importante mencionar que o banco de dados é TinyDB, que se trata de um tipo de banco de dados NoSQL, não relacional, e que utiliza arquivos JSON para armazenar informações. Esse tipo de banco de dados foi escolhido pela simplicidade e fácil configuração que ele apresenta, o que o torna ideal para uma prova de conceito, ou seja, um projeto de pequeno porte que não requer grandes armazenamentos de dados.

&emsp;&emsp;No momento, planeja-se utilizar apenas o arquivo `pipes.json` para armazenamento dos dados, mas pode surgir a necessidade de criar novos arquivos JSON para armazenar os dados devido à natureza não relacional do TinyDB. O arquivo `pipes.json` pode ser encontrado na pasta ~/src/data-base.

## Arquivos

&emsp;&emsp;Dado o fato de que o TinyDB não trabalha com o sistema de tabelas, pode ser necessário criar novos arquivos JSON para armazenar informações. Atualmente, nosso projeto conta apenas com um arquivo chamado pipes.json, que armazena os dados dos canos dos reboilers. As informações contidas nesse arquivo estão escritas a seguir, na Tabela 1:

<p style={{textAlign: 'center'}}>Tabela 1 - Relação entre chave e valor das informações contidas no banco de dados</p>

| Chave       | Valor                                                                                           |
|-------------|-------------------------------------------------------------------------------------------------|
| id          | `int`- representa o número de identificação do cano analisado                                   |
| id_boiler   | `int` - representa o número de identificação do reboiler analisado                              |
| dirty_grade | `int` - representa a porcentagem de sujidade do cano analisado (com base na análise de visão computacional)                                  |
| status      | `bool` - verdadeiro caso o cano analisado esteja sujo e falso caso o cano analisado esteja limpo |
| datetime    | `string` - ano, mês, dia, horas, minutos e segundos do momento em que o registro foi feito       |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe SugarZ3ro</p>

O formato no qual tais dados são armazenados é o seguinte:

```json
{
    "id": <int>,
    "id-boiler": <int>,
    "dirty-grade": <int>,
    "status": <boolean>,
    "datetime": <str>
}
```

&emsp;&emsp;A escolha das informações contidas nesta base de dados foi pensada com base nas necessidades atuais. Como o projeto foi inicialmente projetado para apenas guardar se os canos estão sujos ou não e um códio de identificação deles, foi decidido que somente haveria informações básicas de identificação dos canos para monitoramento.

&emsp;&emsp;No momento, não é preciso de outras informações sobre os reboilers nem informações sobre o usuário do sistema, seguindo os pedidos e observações do cliente em relação às necessidades do sistema.

&emsp;&emsp;Esta é a primeira versão da base de dados do projeto e pode sofrer alterações conforme novas necessidades. Foram realizados testes para verificar se os dados estavam sendo corretamente enviados para o arquivo JSON, utilizando dados para "dirty-grade" gerados aleatoriamente e definindo um máximo de 30% de sujeira para que o cano seja considerado limpo. A utilização do método `random` foi exclusiva para o primeiro teste, o que significa que ele não será utilizado na versão final do banco de dados. Além disso, conforme descrito na [seção referente à API da solução](./api.md), também foram realizados testes de criação, leitura e deleção de dados do banco de dados através da API.

&emsp;&emsp; O percentual de 30% foi escolhido apenas para teste no momento atual. No futuro, a porcentagem máxima de sujidade de um cano para considerá-lo limpo será decidida pelo parceiro. A versão do banco de dados atual está adaptada para testes e não é necessariamente a versão final do banco de dados principal.







