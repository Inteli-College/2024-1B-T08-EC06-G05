---
title: Métodos de teste
sidebar_position: 2
---

&emsp;&emsp;Nessa seção será abordado os testes mapeados para verificar as funcionalidades da solução. Testes desempenham um papel crucial no desenvolvimento de produtos e na avaliação de serviços, servindo como ferramentas essenciais para garantir a qualidade, usabilidade e eficácia. Eles fornecem insights valiosos sobre o desempenho de um sistema em condições controladas, possibilitando a identificação e correção de falhas antes do lançamento ao público. Além disso, testes bem planejados contribuem significativamente para a confiabilidade do projeto.

&emsp;&emsp;Existem diversas formas de testar um projeto. Neste caso, optou-se pelo Teste de Usabilidade do Sistema (SUS) devido à sua capacidade de fornecer avaliações rápidas e confiáveis sobre a usabilidade. O SUS é composto por um questionário conciso que pode ser adaptado a diversos tipos de produtos, o que o torna especialmente versátil. Sua popularidade também se deve ao fato de ser uma metodologia bem validada, facilitando a interpretação dos resultados e a implementação de melhorias no design de interfaces. Com isso em mente, foram formulados os testes a seguir:

<p style={{textAlign: 'center'}}>Tabela 1 - Testes usando o modelo SUS</p>

 Código do Teste | Título      | Detalhes   | Requisitos Testados |
|-----------------|-------------|------------|---------------------|
| T-01           | Mover o robô | A pessoa que testará deve entrar na aplicação WEB e conseguir mover o robô, sem sentir que suas ações estejam demorando para ser feita pelo robô e sentir que é intuitivo e simples de usar. | RF-01, RNF-02 , RNF-03, RNF-07, RNF-05      |
| T-02           | Câmera para locomoção | A pessoa que testaŕa deve entrar na aplicação WEB e conseguir se locomover e ver onde o robô está indo pela cẫmera, sem sentir que está demorando para atualizar a imagem ou que a imagem esteja dessincronizada com  a realidade | RF-05, RNF-02, RNF-05, RNF-06, RNF-07          |
| T-03           | Câmera para identificação da limpeza | A pessoa que testará deve entrar na aplicação WEB e conseguir apontar a câmera para um tubo e analisar se o tubo está sujo ou não, com essa informação sendo guardada no banco de dados | RF-02, RF-03, RNF-04, RNF-05, RNF-06, RNF-07         |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe SugarZ3ro</p>

&emsp;&emsp;Note que, para o teste do SUS fornecer dados reais e valiosos, o teste deve ser conduzido com a menor influência externa possível, assim conseguindo verificar de verdade se a pessoa que está testando consegue navegar e utilizar a solução. Sendo assim, os testes individuais não seram expostos para quem testará, mas cada teste será avaliado separadamente.

&emsp;&emsp;Porém, do jeito que o teste SUS é conduzido, é dificil mensurar coisas como tempo de resposta e de funcionamento. Visando suprir essa lacuna, criamos outros testes que serão realizados dentro da equipe para verificar os pontos citados, que podem ser visualisados na tabela abaixo:

<p style={{textAlign: 'center'}}>Tabela 2 - Testes de validação do tempo de funcionamento e resposta</p>

| Código do Teste | Título      | Detalhes   | Requisitos Testados |
|-----------------|-------------|------------|---------------------|
| T-04           | Duração da bateria do robô | Por meio de uso contínuo, deve ser verificado se a bateria dura pelo menos 9 horas e consegue ser carregada em menos de 8 horas. Note que, para esse teste ser efetivo, o robô deve estar em constante uso durante o teste. Somente assim será verificado a durabilidade real da bateria | RF-01, RNF-01, RNF-05       |
| T-05           | Tempo de resposta | Deve ser conduzido um teste por no mínimo 30 minutos para verificar o tempo de resposta (máximo de 200ms) e se a velocidade se mantém constante | RF-01, RF-05, RNF-02, RNF-03, RNF-05          |
| T-06           | Armazenamento de informações | O banco de dados deve aguentar no mínimo 100.000 linhas. Assim, alocaremos linhas no banco de dados adicionando de 10.000 em 10.000 até chegar em 110.000. Se o banco de dados aguentar esse volume de dados e não ficar muito lento (A API deve retornar os dados em até 1,5s), o teste resultará em sucesso. | RF-06, RF-06, RNF-08         |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe SugarZ3ro</p>
