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

&emsp;&emsp;Como tinha sido definido que seria utilizado o modelo YOLO para construção da Inteligência Artificial, foi utilizado [esse site](https://public.roboflow.com) para achar um dataset anóloga ao problema do projeto. Esse site foi escolhido por apresentar datasets de conteúdos diversos open source e a disponibilidade para download no modo de arquivo que o YOLOv8 utiliza. 

&emsp;&emsp;O dataset escolhido foi [esse](https://universe.roboflow.com/purdue-university-niruh/precision-ag-subterranean/browse?queryText=&pageSize=50&startingIndex=0&browseQuery=true), pois apresenta impurezas e obstáculos em canos que estão claros e escuros, além de conter imagens sem impurezas para mostrar ao modelo o que seria um cano limpo. Está sendo usado todas as 631 imagens presentes no dataset, dividas em 549 imagens de treinamento, 50 imagens de validação e 32 imagens de teste. Note que todas as imagens presentes tem dimensão de 640x640.

## Criação e comparação dos modelos de IA

&emsp;&emsp;Nessa seção, será abordada a comparação dos diferentes modelos de Inteligência Artificial desenvolvidos, destacando os detalhes do modo de treinamento e seus resultados. Esta comparação é fundamental para escolher o modelo mais adequado para nosso projeto, que tem como objetivo a identificação de sujeira dentro de canos. A seguir, será inserida uma subseção que explicará em detalhes o que cada métrica representa, para que qualquer pessoa, mesmo sem background técnico, consiga entender a sua importância na avaliação do desempenho dos modelos.


### Métricas do Modelo
&emsp;&emsp;As métricas são indicadores utilizados para avaliar a performance do modelo de IA. A seguir, apresentamos as principais métricas: **Precisão**, **Recall**, **mAP50**, **mAP50-95**, e **Fitness**, explicando o que cada uma representa e sua importância para nosso projeto.

#### Precisão
&emsp;&emsp;A precisão indica a porcentagem de verdadeiros positivos (detecções corretas) em relação ao total de detecções feitas pelo modelo. Uma alta precisão é importante porque demonstra a confiabilidade do modelo em identificar objetos sem muitos falsos alarmes. Para a identificação de sujeira dentro de canos, isso significa menos detecções erradas de sujeira onde não há.

#### Recall
&emsp;&emsp;O recall mede a capacidade do modelo de identificar todos os objetos relevantes em uma imagem. Uma alta taxa de recall é crucial para garantir que o modelo não perca objetos importantes, ou seja, todas as áreas sujas dentro dos canos serão identificadas.

#### mAP50
&emsp;&emsp;O mAP50 (mean Average Precision at 50% IoU) é a média das precisões para diferentes classes de objetos, considerando uma sobreposição mínima de 50% entre a detecção e o objeto real. Essa métrica é útil para avaliar a eficácia do modelo em detectar sujeira de forma geral, com uma exigência de sobreposição moderada.

#### mAP50-95
&emsp;&emsp;O mAP50-95 é uma métrica mais rigorosa que calcula a média das precisões considerando sobreposições de 50% até 95%. Essa métrica oferece uma avaliação mais completa do desempenho do modelo, especialmente em situações que exigem alta precisão na localização da sujeira dentro dos canos.

##### Diferença entre mAP50 e mAP50-95

&emsp;&emsp;As métricas mAP50 e mAP50-95 diferem na forma como a precisão é calculada em relação à sobreposição entre as detecções do modelo e os objetos reais nas imagens.

- **mAP50**: Foca em uma única sobreposição de 50%, oferecendo uma visão de como o modelo se comporta com uma exigência de sobreposição moderada.

- **mAP50-95**: Foca em uma faixa de sobreposições, oferecendo uma avaliação mais rigorosa e abrangente do desempenho do modelo em diferentes níveis de precisão.

&emsp;&emsp;Ambas as métricas são importantes, mas juntas elas fornecem uma visão mais completa da capacidade do modelo de detectar e localizar objetos em diferentes cenários.

#### Fitness

&emsp;&emsp;A métrica de Fitness combina várias métricas para dar uma visão geral do desempenho do modelo. Um bom valor de fitness indica que o modelo tem um desempenho equilibrado, combinando precisão e recall de maneira eficiente. Para nosso projeto, isso significa que com base nos testes executados, o modelo é eficaz em identificar sujeira dentro dos canos de forma precisa e consistente.

### Método de criação dos modelos

&emsp;&emsp;Nessa seção será indicado exatamente como foram criados os modelos utilizados.

### Comparação dos modelos

&emsp;&emsp;Nessa seção será comparado o resultado dos modelos criados, além de informar qual foi o modelo com o melhor resultado.

&emsp;&emsp;O primeiro modelo criado obteve resultados promissores, apresentando 91% de precisão, 87% de recall, 89% de mAP50, 68% de mAP50-95 e 0.70% de fitness. Com o aprimoramento do treinamento do modelo, foi possivel chegar até 95% de precisão, 91% de recall, 95% de mAP50, 71% de mAP50-95 e 0.71% de fitness, um resultado aprimorado comparado ao primeiro treinamento. Assim sendo, esse resultado é adequado ao projeto, já que a importância máxima é de identificar se há sujeiras presentes, então uma precisao e recall acima de 90% atendem o objetivo do projeto de forma satisfatória.

## Treinamento do modelo

&emsp;&emsp;Para realizar o treinamento da IA, foi utilizada a biblioteca YOLO em um código que realiza tanto o treinamento quanto a validação da qualidade desse treinamento por meio das imagens de validação encontradas no banco de dados. No código encontrado no arquivo `yolo.py`, o modelo YOLO é inicializado usando os pesos pré-treinados do arquivo `yolov8n.pt` e treinado utilizando o conjunto de dados especificado no arquivo `data.yaml`. O treinamento ocorre pela quantidade de épocas escolhidas.

```
from ultralytics import YOLO
import cv2

def main():
    model = YOLO("yolov8n.pt")

    model.train(data="../data-base/data.yaml", epochs=70, imgsz=640)
    matrics = model.val()
    print(matrics)

if __name__ == "__main__":
    main()
    
```

&emsp;&emsp;Após o treinamento, o modelo é validado para avaliar seu desempenho. As métricas de validação são armazenadas na variável metrics. Para uma rápida avaliação do treinamento do modelo, foi decidido que seria útil imprimir as métricas de validação no console.

## Teste da IA com os aquivos de treinamento

&emsp;&emsp;Após o treinamento e escolha do modelo, foi decidido que iriamos testar a funcionalidade da IA de duas formas, utilizando a IA para analizar imagens estaticas quando analise por meio de transmissão de video em tempo real.

&emsp;&emsp;Para isso, foi desenvolvido dois codigos, uma para analise exclusiva de imagens salvas em um banco de dados (`yoloImagem.py`) e outro para analisar videos obtidos em tempo real por meio de uma webcam (`yoloVideo.py`). 

&emsp;&emsp;O código encontrado no arquivo `yoloImagem.py` processa uma imagem usando um modelo YOLO para detecção de objetos, visualiza os resultados e salva a imagem processada em um diretório de saída. Para o processamento de imagem foi utilizada a função `process_image` usando o modelo YOLO e salva a imagem processada em um diretório de saída.

```
def process_image(image_path, output_dir="../data-base/imgs_results"):
    
    image_path = "../data-base/imgs/img2.png"
    # Load the image
    img = cv2.imread(image_path)

    # Check if the image was loaded successfully
    if img is None:
        print(f"Error: Unable to load image at {image_path}")
        return

    # Load the YOLO model
    model = YOLO("./home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train4/weights/best.pt")

    # Process the image
    results = model(img)

    # Process results list
    for result in results:
        # Visualize the results on the frame
        img = result.plot()

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Create the output path
    base_name = os.path.basename(image_path)
    name, ext = os.path.splitext(base_name)
    output_path = os.path.join(output_dir, f"{name}_result{ext}")

    # Save the result image
    cv2.imwrite(output_path, img)
    print(f"Processed image saved as {output_path}")
    
```

&emsp;&emsp;O código encontrado no arquivo `yoloVideo.py` utiliza a biblioteca OpenCV para capturar vídeo em tempo real da webcam, aplica um modelo YOLO para detecção e rastreamento de objetos, visualiza os resultados no feed de vídeo, e, se o rastreamento estiver ativado, desenha as trajetórias dos objetos rastreados.

&emsp;&emsp;O loop do codigo tem a função de captura frames da webcam, aplica o modelo YOLO para detecção e rastreamento de objetos, visualiza os resultados e, se necessário, desenha as trajetórias dos objetos rastreados. A execução do loop continua até que a tecla 'q' seja pressionada.

&emsp;&emsp;Esses códigos aplicam um modelo treinado YOLO para detecção e rastreamento de objetos, visualizar os resultados e desenhar as trajetórias dos objetos rastreados. Ele é útil para aplicação desenvolvida nesse projeto, e auxilia no monitoramento e análise de vídeo e de iamgens em tempo real. 