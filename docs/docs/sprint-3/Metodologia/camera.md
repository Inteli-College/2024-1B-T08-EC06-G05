---
title: Software de transmissão de vídeo
sidebar_position: 1
---
## Introdução
&emsp;&emsp;Durante a terceira sprint do projeto, a equipe SugarZ3ro focou na implementação de uma webcam no robô que transmitisse imagens em tempo real para monitoramento do ambiente de movimentação do robô, além da implementação do cálculo da latência na transmissão das imagens. Para isso, foi utilizada uma webcam do modelo DOBOT Magician, acoplada ao Turtlebot3 por meio de um cabo USB.

## Setup da Webcam
&emsp;&emsp;Após a conexão física da webcam com o robô por meio de uma porta USB disponível na Raspberry Pi, foi desenvolvido um código para a conexão entre o vídeo processado na Raspberry Pi e a interface utilizada pelo usuário, onde há a transmissão em tempo real do vídeo capturado pela câmera. O código desenvolvido utiliza ROSBridge para realizar a comunicação entre o backend e o frontend e está localizado no arquivo `sender.py`. O ROSBridge permite que essa comunicação seja feita através da rede, por isso, é importante que tanto a Raspberry Pi quanto o computador que estiver executando o frontend estejam conectados na mesma rede. O código desenvolvido tem como base a classe "WebcamPublisher" vai ser detalhada a seguir para melhor compreensão da sua utilização.

***sender.py***

&emsp;&emsp;A classe a seguir é usada para capturar frames da webcam, comprimir esses frames como imagens JPEG e publicá-los no tópico ROS (Robot Operating System) `/video_frames`.
```
class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        self.cap = cv2.VideoCapture(0)
        self.latency_thread = threading.Thread(target=self.latencia)
        self.latency_thread.start()
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)
    def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))
        prev_tick = cv2.getTickCount()
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            new_tick = cv2.getTickCount()
            latency = (new_tick - prev_tick) / cv2.getTickFrequency()
            print("Latency: {:.3f} sec".format(latency))
            prev_tick = new_tick
        self.cap.release()
        cv2.destroyAllWindows()

```

&emsp;&emsp;As imagens são publicadas a partir do seguinte comando:
```self.create_publisher(CompressedImage, '/video_frames', 10)```

&emsp;&emsp;O método `latencia` tem a função de medir a latência entre frames e imprime no terminal os valores calculados.

&emsp;&emsp;A função `def main`  inicializa e mantem o nó ROS que captura e publica frames de vídeo da webcam.
```
def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```
&emsp;&emsp;A execução do código cria um nó ROS que continuamente publica frames da webcam e mede a latência entre eles, enquanto realiza a captura de vídeo em um thread separado.

&emsp;&emsp;O código apresentado deve ser executado pelo mesmo computador que iniciar a operação do robô. Para isso, é necessário ter o ROSBridge instalado. A instalação pode ser realizada a partir do seguinte comando:
`sudo apt install ros-humble-rosbridge-suite`

&emsp;&emsp;O servidor websocket também deve ser iniciado nesse mesmo terminal pelo seguinte comando:
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

&emsp;&emsp;Após iniciar o websocket, já é possível executar o arquivo `sender.py` na Raspberry Pi.

## Cálculo da Latência

&emsp;&emsp;Durante o desenvolvimento foi realizada a implementação do cálculo da latência da webcam para analisar sua capacidade de realizar a tarefa de captura de imagens com maior demanda em tempo real. Latência de uma câmera é o atraso entre o momento em que a imagem é capturada pelo sensor da câmera e o momento em que essa imagem é exibida na tela ou processada. No caso da aplicação em desenvolvimento, saber o intervalo de tempo desse processo permite que os desenvolvedores tenham uma visão mais clara do tempo que a imagem leva para percorrer todo o caminho até a interface, onde o vídeo é apresentado, permitindo melhores análises em relação ao desenpenho da nova implementação. 

&emsp;&emsp;Essa funcionalidade foi implemetada no arquivo `sender.py` por meio do método `latencia` presente na classe `WebcamPublisher`. Como foi explicado anteriormente, o metodo calcula a latência entre frames e imprime no terminal os valores obtidos. 

```
def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))
        prev_tick = cv2.getTickCount()
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            new_tick = cv2.getTickCount()
            latency = (new_tick - prev_tick) / cv2.getTickFrequency()
            print("Latency: {:.3f} sec".format(latency))
            prev_tick = new_tick
        self.cap.release()
        cv2.destroyAllWindows()
```


## Reprodução do video na interface

&emsp;&emsp;Para reproduzir o vídeo da webcam, foi desenvolvido um componente para o frontend em React, encontrado no arquivo `camera.jsx`. Esse componente contém um script que recebe as imagens da câmera por meio da comunicação do ROSBridge via WebSocket executada no arquivo `sender.py`. Utilizar o ROSBridge permite que as imagens sejam enviadas em tempo real e que seja possível acompanhar a movimentação do robô por meio do vídeo na interface do usuário.

```

const VideoStream = () => {
  const videoRef = useRef(null);

  useEffect(() => {
    // Conectar ao servidor ROS
    const ros = new ROSLIB.Ros({
      url: 'ws://10.128.0.30:9090'
    });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    ...

    // Assinar ao tópico de vídeonsor_msgs/Compres
    const videoTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    // Função para lidar com os quadros de vídeo recebidos
    videoTopic.subscribe((message) => {
      if (videoRef.current) {
        videoRef.current.src = 'data:image/jpeg;base64,' + message.data;
      }
    });

    // Limpar a assinatura quando o componente for desmontado
    return () => {
      videoTopic.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <div>
      <h1>Real-time Video Stream from ROS2 Topic</h1>
      <img id="videoStream" ref={videoRef} alt="Video Stream" style={{ width: '640px', height: '480px' }} />
    </div>
  );
};

export default VideoStream;
```

&emsp;&emsp;Até o momento, o frontend está sendo executado apenas localmente, por isso é importante verificar se o IP do computador que está executando o frontend é o mesmo IP no arquivo ``camera.jsx``. Caso seja necessario alterar, o IP atual deve ser substituido pelo IP do computador na seguinte linha do arquivo: `` url: 'ws://10.128.0.30:9090' `` 

## Conclusão 

&emsp;&emsp;Durante o desenvolvimento da sprint 3, a equipe SugarZ3ro implementou com sucesso uma webcam no robô, utilizando uma DOBOT Magician acoplada ao Turtlebot3. O codigo de comunicação permite a transmissão de imagens em tempo real por meio do ROSBridge e o cálculo da latência. A interface em React recebe essas imagens, permitindo o monitoramento contínuo do robô. Esta implementação foi testada, cumprindo os objetivos de monitoramento e análise de latência do sistema.
