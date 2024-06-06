# Documentação do Sistema de Segurança - Versão Atualizada

## Introdução

&emsp;&emsp;Na sprint 2, foram apresentados dois sistemas de segurança pelo grupo SugarZ3ro atrelados ao controle do Turtlebot, através dos quais o usuário teria mais precisão na movimentação e a opção de acionar uma parada de emergência ao pressionar a tecla `Q`. Durante a sprint 3, o sistema de segurança passou por significativas melhorias para aprimorar a detecção de obstáculos e a prevenção de colisões, utilizando dados de um LiDAR (Laser Imaging Detection and Ranging).

&emsp;&emsp;O sistema atualizado mantém a essência do design original, mas agora é capaz de monitorar a proximidade de objetos ao redor do Turtlebot de forma mais eficiente. Se um objeto for detectado a uma distância perigosa, o sistema ajusta a velocidade do robô para evitar colisões de maneira preventiva.

## Sistema de Segurança - CLI

&emsp;&emsp;O sistema de segurança integrado à CLI foi aprimorado no mesmo script do pacote `SugarZ3ro_pkg`, que controla a movimentação do Turtlebot 3. O arquivo, localizado em `~/src/workspace/src/SugarZ3ro_pkg/SugarZ3ro_pkg/movimentation.py`, agora inclui um sistema anti-colisões mais robusto, capaz de detectar e responder a obstáculos de forma mais eficaz. A seguir, são detalhadas as melhorias feitas no script.

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
        front_angles = range(len(msg.ranges)//2 - 10, len(msg.ranges)//2 + 10)
        left_angles = range(len(msg.ranges)//4*3 - 10, len(msg.ranges)//4*3 + 10)
        right_angles = range(len(msg.ranges)//4 - 10, len(msg.ranges)//4 + 10)
        
        self.min_distance_ahead = min([msg.ranges[i] for i in front_angles if 0 < msg.ranges[i] < float('inf')], default=float('inf'))
        self.min_distance_left = min([msg.ranges[i] for i in left_angles if 0 < msg.ranges[i] < float('inf')], default=float('inf'))
        self.min_distance_right = min([msg.ranges[i] for i in right_angles if 0 < msg.ranges[i] < float('inf')], default=float('inf'))

        self.get_logger().info(f'Distância à frente: {self.min_distance_ahead:.2f}m, à esquerda: {self.min_distance_left:.2f}m, à direita: {self.min_distance_right:.2f}m')
    ```
    - **Descrição**: Esta função é chamada sempre que uma nova mensagem é publicada no tópico `scan`.
    - **Função**:
        - Filtra os valores infinitos (que representam leituras inválidas) e valores muito baixos (ruído).
        - Atualiza as variáveis `min_distance_ahead`, `min_distance_left`, e `min_distance_right` com a menor distância válida detectada pelo sensor em cada direção.
        - **Variáveis `min_distance_ahead`, `min_distance_left`, `min_distance_right`**: Representam a menor distância medida até um obstáculo em metros em cada direção.

3. **Detecção e Resposta a Obstáculos na Função `run`**
    ```python
    if self.min_distance_ahead <= self.stop_distance:
        print("OBSTÁCULO DETECTADO À FRENTE A 30cm! Movimentação para frente bloqueada.")
    if self.min_distance_left <= self.stop_distance:
        print("OBSTÁCULO DETECTADO À ESQUERDA A 30cm! Movimentação para esquerda bloqueada.")
    if self.min_distance_right <= self.stop_distance:
        print("OBSTÁCULO DETECTADO À DIREITA A 30cm! Movimentação para direita bloqueada.")

    with self.lock:
        key = self.key_pressed
        last_key = self.last_key_pressed

    if key == 'q':
        print("Foi bom te conhecer...")
        self.running = False
        break

    if key != last_key:
        if key in ['w', 's', None]:
            target_angular_vel = 0.0
        if key in ['a', 'd', 's', None]:
            target_linear_vel = 0.0

    if key == 'w':
        if self.min_distance_ahead > self.stop_distance:
            if self.mensagem:
                print("Andando para frente...")
            self.mensagem = False
            target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
        else:
            if self.mensagem:
                print("Obstáculo à frente! Movimentação para frente bloqueada.")
            target_linear_vel = 0.0

    elif key == 'a':
        if self.min_distance_left > self.stop_distance:
            if self.mensagem:
                print("Virando para esquerda...")
            self.mensagem = False
            target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
        else:
            if self.mensagem:
                print("Obstáculo à esquerda! Movimentação para esquerda bloqueada.")
            target_angular_vel = 0.0

    elif key == 'd':
        if self.min_distance_right > self.stop_distance:
            if self.mensagem:
                print("Virando para direita...")
            self.mensagem = False
            target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
        else:
            if self.mensagem:
                print("Obstáculo à direita! Movimentação para direita bloqueada.")
            target_angular_vel = 0.0

    elif key == 's':
        if self.mensagem:
            print("Parando...")
        self.mensagem = True
        target_linear_vel = -min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
        target_angular_vel = 0.0

    self.last_key_pressed = key

    twist = Twist()
    twist.linear.x = float(target_linear_vel)
    twist.angular.z = float(target_angular_vel)
    self.publisher_.publish(twist)
    time.sleep(0.1)
    ```
    - **Descrição**: Este bloco de código é responsável por detectar se o Turtlebot está próximo de um obstáculo e tomar ações para afastar o robô.
    - **Função**:
        - Verifica se a distância mínima (`min_distance_ahead`, `min_distance_left`, `min_distance_right`) é menor ou igual à distância de parada (`stop_distance`), que é definida como 0.3 metros (30 cm).
        - Se um obstáculo é detectado a 30 cm ou menos:
            - Imprime uma mensagem de alerta.
            - Ajusta a velocidade do robô para impedir a movimentação nas direções perigosas, permitindo apenas a movimentação para trás quando necessário.

4. **Parâmetros do Sistema de Segurança**
    - **`stop_distance`**: 
        - **Descrição**: A distância mínima segura antes de tomar ação corretiva.
        - **Valor**: 0.3 metros (30 cm).
    - **`min_distance_ahead`, `min_distance_left`, `min_distance_right`**:
        - **Descrição**: A menor distância atual até um obstáculo em cada direção, atualizada pelo callback do LaserScan.

### Fluxo do Sistema de Segurança - CLI

1. **Início**: A função `run` inicia e entra em um loop contínuo enquanto `rclpy.ok()` e `self.running` forem verdadeiros.
2. **Leitura de Dados**: O callback `scan_callback` processa os dados do sistema LiDAR, atualizando `min_distance_ahead`, `min_distance_left`, `min_distance_right`.
3. **Verificação de Obstáculos**: 
    - A cada iteração do loop, verifica-se se `min_distance_ahead`, `min_distance_left`, ou `min_distance_right` é menor ou igual a `stop_distance`.
    - Se qualquer uma das distâncias for ≤ `stop_distance`, o sistema imprime uma mensagem de alerta.
4. **Ação Corretiva**: 
    - Ajusta a velocidade do Turtlebot para impedir movimentações para frente ou para os lados se houver um obstáculo detectado nessas direções.
    - Permite apenas a movimentação para trás quando necessário.
5. **Segurança Restabelecida**: Quando `min_distance_ahead`, `min_distance_left`, e `min_distance_right` > `stop_distance`, o sistema imprime uma mensagem de segurança e continua a operação normal.

## Conclusão

&emsp;&emsp;O sistema de segurança é uma parte crucial para garantir que o robô opere de maneira segura, prevenindo colisões. Utiliza um sistema LiDAR para monitorar o ambiente e ajusta a velocidade do Turtlebot automaticamente para evitar obstáculos. Este sistema é implementado de forma a ser contínuo e responsivo, garantindo que o Turtlebot possa navegar de maneira autônoma sem risco de danos.
