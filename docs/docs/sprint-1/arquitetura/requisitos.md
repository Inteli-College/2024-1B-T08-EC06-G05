---
title: Definição de requisitos
sidebar_position: 1
---

Nessa seção será aborado os requisitos funcionais e não funcionais criados. Esses requisitos são fundamentais para definir as funcionalidades e guiar a execução do projeto. Os requisitos funcionais definem as features fundamentais para o funcionamento do proejto, enquanto os requisitos não funcionais definem aspectos qualidativos como por exemplo tempo de resposta. A confecção desses gera uma maior compreensão do projeto e como será desenvolvido, servindo como um guia de objetivos no projeto. Sendo assim, foi desenvolvido os seguintes requisitos:

Os requisitos de um sistema são essenciais para definir suas funcionalidades e guiar a execução do projeto. Os requisitos funcionais delineiam as ações fundamentais para o funcionamento do produto, enquanto os requisitos não funcionais complementam essas funcionalidades, incluindo aspectos como segurança e desempenho. Eles formam a base que orienta o desenvolvimento, garantindo uma compreensão clara do que será realizado e como as etapas do projeto serão conduzidas.

**1. Tabela de Requisitos Funcionais (RF) - Turtlebot para verificar limpeza:**

| Código do Requisito | Título               | Detalhes                | Método de Teste        |
|----------------------|----------------------|-------------------------|------------------------|
| RF-01                | Locomoção do robô. | Pela interface criada, o robô deve ser capaz de mover. | Por meio da interface, o robô se move. Caso ele não se mova em alguma direção, o teste falhou. |
| RF-02                | Verificação da limpeza. | Pela interface criada, o usuário deve ser capaz de orndenar o robô, por meio da câmera, identificar se um tubo está sujo ou não. | Por meio da interface, o robô ativa a câmera e identifica se o tubo está sujo ou não. Caso a câmera não ative ou a câmera não reconheça se está limpo ou não, o teste falhou.  |
| RF-03                | Registro de logs. | Deve haver APIs para mandar as informações de logs de funcionamento do robô, como quantos e quais tubos estão sujos. | Caso as informações não sejam guardadas corretamente ou não serem mandadas corretamente pela API, o teste falhou. |

**Fonte:** Elaborado pela equipe SugarZ3ro

**2. Tabela de Requisitos Não Funcionais (RNF) - Turtlebot para verificar limpeza:**

| Código do Requisito | Título               | Detalhes                | Método de Teste        |
|----------------------|----------------------|-------------------------|------------------------|
| RNF-01               | Tempo de funcionamento. | A bateria do robô deve ser duradoura e funcionar por um expediente inteiro (8h), para minimizar a quantidade de baterias que devem ser levadas junto ao robô e não ter que trocar ela constantemente. | Considerando o desgaste de bateria, uma bateria nova de lítio-ion ou polímero de lítio deve durar pelo menos 9 horas interruptas e ser carregada em menos de 8 horas. Caso alguma dessas exigências não seja suprida, o teste falhou.  |
| RNF-02               | Tempo de resposta. | O tempo de resposta da locomoção do robô deve ser o menor possível para que a experiência do usuário mantenha em um nível satisfatório. | O tempo de resposta do comando de movimento do robô (pela interface) deve ser de menos de 200ms. Caso seja maior que 200ms, o teste falhou. |
| RNF-03               | Velocidade de locomoção. | O robô deve manter uma aceleração e velocidade máxima constante definida pelo grupo SugarZ3ro, assim assegurando a experiência do usuário, já que mantem um padrão. | A aceleração e velocidade máxima do turtlebot deve ser constante durante o funcionamento dele, sendo a velocidade máxima marcada em 15m/s. O teste deve ser feito por 30 minutos (mínimo), com arrancadas e paradas para medir se a aceleração e velocidade máxima são constantes o suficiente para o usuário que testar não perceber mudanças.  |
| RNF-04               | Acurácia da inteligência artificial. | A inteligência artifical que analisa as imágens da câmera deve apresentar uma acurácia alta, para assim evitar que haja trabalho sem necessidade de limpar algo que já esteja limpo e que deixe de limpar algo que está sujo, diminuindo a eficiência da produção. | A inteligência artificial deve captar a imagem da câmera e analisar se está limpo ou sujo, com uma acurácia de no mínimo 80%. Caso tenha uma acurária menor que o valor de 80%, o teste falhou. |
| RNF-05               | Armazenamento de informações. | O sistema deve ser capaz de aguentar um grande volume de informações, já que o projeto lidará com um grande volume de informação. | O banco de dados deve aguentar pelo menos 100.000 linhas e o sistema não pode ficar mais lento que 1.5s para retornar as informações que o usuário procurar. |
| RNF-06               | Usabilidade. | A interface deve ser de fácil uso e intuitiva. | Por meio do teste SUS o nosso sistema deve ter uma nota de no mínimo 70 para garantir que o sistema tenha uma navegabilidade satisfatória. |

**Fonte:** Elaborado pela equipe SugarZ3ro

Essas tabelas apresentam os requisitos funcionais (RF) e não funcionais (RNF) relacionados ao uso do robô (turtlebot) na usina da Atvos, bem como a interface que o contola.contola.