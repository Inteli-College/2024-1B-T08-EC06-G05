### Documentação do Sistema de Segurança

#### Introdução
O sistema de segurança implementado no código é responsável por detectar obstáculos na trajetória do robô e tomar medidas preventivas para evitar colisões. Utiliza dados de um sistema LiDAR (Laser Imaging Detection and Ranging) para monitorar a distância de objetos ao redor do turtlebot. Se um objeto for detectado a uma distância perigosa, o sistema ajusta a velocidade do robô para afastá-lo do obstáculo.

#### Componentes do Sistema de Segurança

1. **Inicialização do Subscriber para LiDAR (LaserScan)**
    ```python
    self.scan_subscriber = self.create_subscription(
        LaserScan,
        'scan',
        self.scan_callback,
        qos_profile=qos_profile_sensor_data)
    ```
    - **Descrição**: Este trecho de código cria um subscription para o tópico `scan`, que recebe dados do sistema LiDAR.
    - **Função**: Permite que o nó receba continuamente dados de distância do sistema LiDAR, necessários para detectar obstáculos.

2. **Callback do LaserScan**
    ```python
    def scan_callback(self, msg):
        ranges = [distance for distance in msg.ranges if not distance == float('inf')]
        
        if ranges:
            self.min_distance = min(ranges)
    ```
    - **Descrição**: Esta função é chamada sempre que uma nova mensagem é publicada no tópico `scan`.
    - **Função**: 
        - Filtra os valores infinitos (que representam leituras inválidas).
        - Atualiza a variável `min_distance` com a menor distância válida detectada pelo sensor.
        - **Variável `min_distance`**: Representa a menor distância medida até um obstáculo em metros.

3. **Detecção e Resposta a Obstáculos na Função `run`**
    ```python
    if self.min_distance <= self.stop_distance:
        print("OBSTÁCULO DETECTADO A 30cm!\nAfastando o robô do obstáculo...")
        while self.min_distance <= self.stop_distance:
            self.mensagem = True
            obstacle_twist = Twist()
            obstacle_twist.linear.x = float(target_linear_vel)
            self.publisher_.publish(obstacle_twist)
            target_linear_vel = -1.0
            target_angular_vel = 0.0
        print("O obstáculo não está mais a 30cm do robô.")
    ```
    - **Descrição**: Este bloco de código é responsável por detectar se o robô está próximo de um obstáculo e tomar ações para afastar o robô.
    - **Função**:
        - Verifica se a distância mínima (`min_distance`) é menor ou igual à distância de parada (`stop_distance`), que é definida como 0.3 metros (30 cm).
        - Se um obstáculo é detectado a 30 cm ou menos:
            - Imprime uma mensagem de alerta.
            - Enquanto o obstáculo estiver a 30 cm ou menos, ajusta a velocidade linear do robô para -1.0 (movimento para trás) e a velocidade angular para 0.0 (sem rotação).
            - Publica esses comandos de velocidade no tópico `cmd_vel` para mover o robô para trás até que a distância do obstáculo seja segura.
            - Após afastar-se do obstáculo, imprime uma mensagem indicando que o robô está seguro.

4. **Parâmetros do Sistema de Segurança**
    - **`stop_distance`**: 
        - **Descrição**: A distância mínima segura antes de tomar ação corretiva.
        - **Valor**: 0.3 metros (30 cm).
    - **`min_distance`**:
        - **Descrição**: A menor distância atual até um obstáculo, atualizada pelo callback do LaserScan.

#### Fluxo do Sistema de Segurança

1. **Início**: A função `run` inicia e entra em um loop contínuo enquanto `rclpy.ok()` e `self.running` forem verdadeiros.
2. **Leitura de Dados**: O callback `scan_callback` processa os dados do sistema LiDAR, atualizando `min_distance`.
3. **Verificação de Obstáculos**: 
    - A cada iteração do loop, verifica-se se `min_distance` é menor ou igual a `stop_distance`.
    - Se `min_distance` ≤ `stop_distance`, o sistema imprime uma mensagem de alerta.
4. **Ação Corretiva**: 
    - Enquanto `min_distance` for ≤ `stop_distance`, ajusta-se a velocidade do robô para mover-se para trás.
    - Publica os comandos de velocidade ajustados para afastar o robô do obstáculo.
5. **Segurança Restabelecida**: Quando `min_distance` > `stop_distance`, o sistema imprime uma mensagem de segurança e continua a operação normal.

#### Conclusão

O sistema de segurança é uma parte crucial para garantir que o robô opere de maneira segura, prevenindo colisões. Utiliza um sistema LiDAR para monitorar o ambiente e ajusta a velocidade do robô automaticamente para evitar obstáculos. Este sistema é implementado de forma a ser contínuo e responsivo, garantindo que o turtlebot possa navegar de maneira autônoma sem risco de danos.
