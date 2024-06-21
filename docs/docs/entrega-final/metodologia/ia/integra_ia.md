---
title: Integração da Visão computacional
sidebar-position: 3
---

## Introdução

&emsp;&emsp;A integração de um sistema de Inteligência Artificial (IA) foi feita apartir de um modelo YOLO (You Only Look Once) para analisar imagens capturadas pela câmera do robô. Este sistema identifica automaticamente se a superfície interna do cano está suja ou limpa, e retorna o status ao usuário. Esta funcionalidade é essencial para a manutenção preventiva e eficiente do reboiler, garantindo maior segurança e operabilidade.

## Componentes do Sistema

### 1. Captura de Imagem com a Câmera

&emsp;&emsp;A captura de imagem é feita pela câmera do robô, que está constantemente transmitindo vídeo ao vivo. A função `VideoStream` no arquivo `camera.jsx` gerencia a conexão com o servidor ROS e exibe o stream de vídeo. Vamos dividir o código e explicar cada trecho em detalhes.

#### Definindo o Componente `VideoStream`

```jsx
const VideoStream = () => {
  const videoRef = useRef(null);
  const [latency, setLatency] = useState(0);
  const [sentTime, setSentTime] = useState(null);
  const ros = useRef(null);
```

&emsp;&emsp;Neste trecho, estamos definindo o componente `VideoStream`. Aqui, `videoRef` é uma referência ao elemento de vídeo que será criado. `latency` e `setLatency` são utilizados para armazenar e definir a latência do stream de vídeo, enquanto `sentTime` e `setSentTime` são usados para armazenar e definir o tempo em que a imagem foi enviada. `ros` é uma referência ao objeto ROS que gerencia a conexão com o servidor ROS.

#### Efeito de Conexão com o Servidor ROS

```jsx
  useEffect(() => {
    if (!ros.current) {
      ros.current = new ROSLIB.Ros({
        url: 'ws://10.128.0.50:9090'
      });

      ros.current.on('connection', () => {
        console.log('Camera: Connected to websocket server.');
      });

      ros.current.on('error', (error) => {
        console.log('Camera: Error connecting to websocket server: ', error);
      });

      ros.current.on('close', () => {
        console.log('Connection to websocket server closed.');
      });
    }
```

&emsp;&emsp;O `useEffect` é usado para executar efeitos colaterais em componentes funcionais. Neste caso, estamos verificando se `ros.current` não está inicializado. Se não estiver, inicializamos uma nova conexão com o servidor ROS no endereço especificado (`ws://10.128.0.50:9090`). Também definimos handlers para eventos de conexão, erro e fechamento para registrar esses eventos no console.

#### Subscrição ao Tópico de Vídeo

```jsx
    const videoTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    videoTopic.subscribe((message) => {
      if (videoRef.current) {
        videoRef.current.src = 'data:image/jpeg;base64,' + message.data;
      }
    });

    return () => {
      videoTopic.unsubscribe();
    };
  }, []);
```

&emsp;&emsp;Após estabelecer a conexão com o servidor ROS, definimos um novo tópico de vídeo usando `ROSLIB.Topic`. O nome do tópico é `/video_frames` e o tipo de mensagem é `sensor_msgs/CompressedImage`. Subscritos a este tópico, definimos um callback para atualizar a fonte (`src`) do elemento de vídeo sempre que uma nova mensagem for recebida. Finalmente, retornamos uma função de limpeza para desinscrever o tópico quando o componente for desmontado.


#### 2. Captura de Frame e Envio para Análise

&emsp;&emsp;O componente `AiButton` no arquivo `aiButton.jsx` captura um frame da transmissão de vídeo, converte-o em uma imagem e a envia para o servidor Flask para processamento com o modelo YOLO.

#### Definindo o Componente `AiButton`

```jsx
const AiButton = () => {
  const videoRef = useRef(null);
  const [status, setStatus] = useState('');
```

&emsp;&emsp;Neste trecho, estamos definindo o componente `AiButton`. Aqui, `videoRef` é uma referência ao elemento de vídeo que será criado. `status` e `setStatus` são utilizados para armazenar e definir o status da imagem analisada (sujo ou limpo).

#### Captura do Frame do Vídeo

```jsx
  const handleSaveFrame = async (event) => {
    event.preventDefault();

    if (videoRef.current) {
      const canvas = document.createElement('canvas');
      canvas.width = videoRef.current.videoWidth;
      canvas.height = videoRef.current.videoHeight;
      const context = canvas.getContext('2d');

      if (!context) {
        console.error('Error: Canvas context is null');
        return;
      }

      context.drawImage(videoRef.current, 0, 0, canvas.width, canvas.height);
      canvas.toBlob(async (blob) => {
        if (blob) {
          const formData = new FormData();
          formData.append('image', blob, 'frame.png');
```

&emsp;&emsp;Aqui, estamos definindo a função `handleSaveFrame` que será chamada ao clicar no botão de captura. Primeiro, criamos um elemento `canvas` e definimos suas dimensões para corresponder ao vídeo. Em seguida, desenhamos o frame do vídeo no `canvas` e convertê-lo em um `blob`.

#### Envio do Frame para o Servidor

```jsx
          try {
            const response = await axios.post('http://localhost:5000/process_image', formData, {
              headers: {
                'Content-Type': 'multipart/form-data'
              }
            });
            const status = response.data.status;
            setStatus(status);
          } catch (error) {
            console.error('Error processing image', error);
          }
        }
      }, 'image/png');
    }
  };
```

&emsp;&emsp;Após criar o `blob`, enviamos o frame para o servidor Flask utilizando uma requisição `POST` com `axios`. O servidor retorna o status da análise da imagem, que é então armazenado no estado `status`.

#### Renderização do Componente

```jsx
  return (
    <div>
      <form onSubmit={handleSaveFrame}>
        <button
          type="submit"
          className="bg-white bg-opacity-70 active:bg-opacity-100 rounded-xl border border-gray-700 flex items-center justify-center p-6"
        >
          <BsStars size={70} />
        </button>
      </form>
      {status && <p>Status: {status}</p>}
    </div>
  );
};

export default AiButton;
```

&emsp;&emsp;O componente renderiza um botão que, ao ser clicado, captura e envia o frame do vídeo para análise. O status da análise é exibido abaixo do botão.


### 3. Processamento da Imagem com YOLO

&emsp;&emsp;No servidor Flask, a imagem é recebida e processada utilizando o modelo YOLO. O servidor retorna o status de limpeza da superfície.

#### Definindo a Rota para Processamento de Imagem

```python
@app.route('/process_image', methods=['POST'])
def process_image():
    if 'image' not in request.files:
        return jsonify({'error': 'No image provided'}), 400

    file = request.files['image']
    image = cv2.imdecode(np.frombuffer(file.read(), np.uint8), cv2.IMREAD_COLOR)

    if image is None:
        return jsonify({'error': 'Unable to load image'}), 400
```

&emsp;&emsp;Esta rota recebe uma requisição `POST` com a imagem anexada. Primeiro, verificamos se a imagem foi enviada na requisição. Em seguida, lemos a imagem utilizando `cv2.imdecode`.

#### Processamento da Imagem com YOLO

```python
    results = model(image)
    is_dirty = any(box.cls == "sujeira" for result in results for box in result.boxes)

    status = "sujo" if is_dirty else "limpo"
    return jsonify({'status': status})
```

&emsp;&emsp;A imagem é processada pelo modelo YOLO, que retorna os resultados das detecções. Verificamos se alguma detecção corresponde à classe "sujeira" e definimos o status como "sujo" ou "limpo" conforme o resultado.

### 4. Interface do Usuário

&emsp;&emsp;A interface do usuário é gerenciada pelo componente `TeleopScreen` em `teleopScreen.jsx`, que integra todos os componentes acima, incluindo o botão de captura de imagem e a exibição do stream de vídeo.

```jsx
import React, { useState, useEffect } from 'react';
import AiButton from '../components/aiButton/aiButton';
import VideoStream from '../components/camera/camera';
import TurtleBotController from '../components/rosbridge_movement/rosbridge_movement';

function TeleopScreen() {
  return (
    <TurtleBotController>
      {({ movementhandlers, lidarData, collision }) => (
        <div className='relative' style={{ width: "1280px", height: "720px" }}>
          <VideoStream />
          <div className='absolute bottom-32 right-32'>
            <AiButton />
          </div>
        </div>
      )}
    </TurtleBotController>
  );
}

export default TeleopScreen;
```

## Conclusão

&emsp;&emsp;A integração da IA na câmera do robô para analisar a limpeza da superfície interna de canos em um reboiler, é uma adição valiosa para a manutenção preditiva e operação segura do sistema. Utilizando tecnologias como YOLO para a detecção de sujeira, garantimos uma análise precisa e eficiente, reduzindo custos e aumentando a eficiência operacional. Esta documentação detalha a implementação técnica e a importância desta solução para o projeto.
