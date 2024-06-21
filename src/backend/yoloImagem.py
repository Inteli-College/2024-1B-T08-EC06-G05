import rclpy  # Importa a biblioteca rclpy para ROS 2
from rclpy.node import Node  # Importa a classe Node para criar nós ROS 2
from sensor_msgs.msg import CompressedImage  # Importa a mensagem CompressedImage para manipulação de imagens
import numpy as np  # Importa a biblioteca numpy para manipulação de arrays
import cv2  # Importa a biblioteca OpenCV para processamento de imagens
from ultralytics import YOLO  # Importa a biblioteca YOLO para detecção de objetos
import os  # Importa a biblioteca os para manipulação de arquivos e diretórios

# Define a classe YOLOImageProcessor que herda de Node
class YOLOImageProcessor(Node):
    def __init__(self):
        super().__init__('yolo_image_processor')  # Inicializa a classe base Node com o nome do nó
        # Cria uma assinatura para o tópico /save_frame para receber mensagens do tipo CompressedImage
        self.subscription = self.create_subscription(
            CompressedImage,
            '/save_frame',
            self.process_image_callback,
            10
        )
        # Inicializa o modelo YOLO com o caminho do arquivo de pesos
        self.model = YOLO("./src/backend/home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train4/weights/best.pt")

    # Função de callback para processar as imagens recebidas
    def process_image_callback(self, msg):
        # Converte a mensagem ROS para um array numpy
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Decodifica o array numpy para uma imagem OpenCV
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Verifica se a imagem foi carregada corretamente
        if img is None:
            self.get_logger().error("Error: Unable to load image from ROS message")  # Loga um erro se a imagem não foi carregada
            return

        # Processa a imagem com o modelo YOLO
        results = self.model(img)

        # Itera sobre os resultados e desenha as detecções na imagem
        for result in results:
            img = result.plot()

        # Define o diretório de saída para salvar as imagens processadas
        output_dir = "/2024-1B-T08-EC06-G05/src/data-base/imgs-results"
        os.makedirs(output_dir, exist_ok=True)  # Cria o diretório de saída, se não existir
        output_path = os.path.join(output_dir, "processed_frame.png")  # Define o caminho completo do arquivo de saída
        cv2.imwrite(output_path, img)  # Salva a imagem processada no disco
        self.get_logger().info(f"Processed image saved as {output_path}")  # Loga a mensagem de sucesso

# Função principal
def main(args=None):
    rclpy.init(args=args)  # Inicializa a biblioteca rclpy
    yolo_image_processor = YOLOImageProcessor()  # Cria uma instância do nó YOLOImageProcessor
    rclpy.spin(yolo_image_processor)  # Mantém o nó em execução
    yolo_image_processor.destroy_node()  # Destrói o nó após a execução
    rclpy.shutdown()  # Encerra a biblioteca rclpy

if __name__ == '__main__':
    main()  # Chama a função principal quando o script é executado
