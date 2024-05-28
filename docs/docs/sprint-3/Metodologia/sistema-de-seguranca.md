# Documentação do Sistema de Segurança

## Introdução

&emsp;&emsp;Na sprint 2, foram apresentados dois sistemas de segurança pelo grupo SugarZ3ro, atrelados ao controle do turtlebot, onde o usuário teria mais precisão e a opção de acionar uma parada de emergência ao pressionar a tecla ```Q```. Agora, na sprint 3, foram feitas mudanças significativas no sistema de segurança, implementando um código responsável por detectar obstáculos na trajetória do robô e tomar medidas preventivas para **evitar colisões**.

&emsp;&emsp;O sistema de segurança utiliza dados de um LiDAR (Laser Imaging Detection and Ranging) para monitorar a distância de objetos ao redor do turtlebot. Se um objeto for detectado a uma distância pré-definida como perigosa, o sistema ajusta a velocidade do robô para que seja possível afastá-lo do obstáculo.

## Sistema de Segurança - Integração ao frontend

&emsp;&emsp;Para tornar a utilização do robô mais segura e acessível para os usuários da solução, a equipe SugarZero implementou o sistema de segurança também na interface gráfica de movimentação do robô. O sistema de segurança consiste num sistema anti-colisão baseado no LiDAR que, ao detectar um obstáculo dentro de um raio de 10 cm, impede que o usuário aperte qualquer tecla de movimentação que não seja para trás. Dessa forma, evita-se que o robô colida com obstáculos à sua frente.

<p style={{textAlign: 'center'}}>Figura 1 - Printscreen da tela de teleoperação</p>

<div style={{textAlign: 'center'}}>
    ![Tela de Teleoperação](https://github.com/Inteli-College/2024-1B-T08-EC06-G05/assets/123904558/29694c53-8e87-4216-ace6-8cfe1977dd1c)
</div>

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe SugarZ3ro</p>

&emsp;&emsp;No print, é vista a tela de teleoperação do sistema. Ao detectar o obstáculo, um alerta aparece na parte superior da interface e os botões de movimentação, com excessão do botão de movimentação para trás, são desativados. Assim, o usuário tem somente a opção de mover-se para trás, afastando-se do objeto e evitando uma possível colisão.

## Sistema de Segurança - CLI

&emsp;&emsp;O sistema de segurança integrado à CLI foi feito no mesmo script do pacote ```SugarZ3ro_pkg``` responsável pela movimentação do Turtlebot 3. O arquivo, que pode ser encontrado em `~/src/workspace/src/SugarZ3ro_pkg/SugarZ3ro_pkg/movimentation.py`, integra o sistema anti-colisões de modo que o robô se afasta automaticamente de obstáculos detectados. A seguir, há uma explicação do script em relação a cada trecho referente ao sistema de segurança. 

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
    - **Descrição**: Este bloco de código é responsável por detectar se o turtlebot está próximo de um obstáculo e tomar ações para afastar o robô.
    - **Função**:
        - Verifica se a distância mínima (`min_distance`) é menor ou igual à distância de parada (`stop_distance`), que é definida como 0.3 metros (30 cm).
        - Se um obstáculo é detectado a 30 cm ou menos:
            - Imprime uma mensagem de alerta.
            - Enquanto o obstáculo estiver a 30 cm ou menos, ajusta a velocidade linear do robô para -1.0 (movimento para trás) e a velocidade angular para 0.0 (sem rotação).
            - Publica esses comandos de velocidade no tópico `cmd_vel` para mover o turtlebot para trás até que a distância do obstáculo seja segura.
            - Após afastar-se do obstáculo, imprime uma mensagem indicando que o robô está seguro.

4. **Parâmetros do Sistema de Segurança**
    - **`stop_distance`**: 
        - **Descrição**: A distância mínima segura antes de tomar ação corretiva.
        - **Valor**: 0.3 metros (30 cm).
    - **`min_distance`**:
        - **Descrição**: A menor distância atual até um obstáculo, atualizada pelo callback do LaserScan.

### Fluxo do Sistema de Segurança - CLI

1. **Início**: A função `run` inicia e entra em um loop contínuo enquanto `rclpy.ok()` e `self.running` forem verdadeiros.
2. **Leitura de Dados**: O callback `scan_callback` processa os dados do sistema LiDAR, atualizando `min_distance`.
3. **Verificação de Obstáculos**: 
    - A cada iteração do loop, verifica-se se `min_distance` é menor ou igual a `stop_distance`.
    - Se `min_distance` ≤ `stop_distance`, o sistema imprime uma mensagem de alerta.
4. **Ação Corretiva**: 
    - Enquanto `min_distance` for ≤ `stop_distance`, ajusta-se a velocidade do turtlebot para mover-se para trás.
    - Publica os comandos de velocidade ajustados para afastar o robô do obstáculo.
5. **Segurança Restabelecida**: Quando `min_distance` > `stop_distance`, o sistema imprime uma mensagem de segurança e continua a operação normal.

## Conclusão

&emsp;&emsp;O sistema de segurança é uma parte crucial para garantir que o robô opere de maneira segura, prevenindo colisões. Utiliza um sistema LiDAR para monitorar o ambiente e ajusta a velocidade do turtlebot automaticamente para evitar obstáculos. Este sistema é implementado de forma a ser contínuo e responsivo, garantindo que o turtlebot possa navegar de maneira autônoma sem risco de danos.

## Vídeo de demonstração da solução

&emsp;&emsp;Para demonstrar o funcionamento da solução desenvolvida até o momento, foi gravado um vídeo demonstrando a movimentação do robô, a interface web e o sistema de segurança utilizando o LiDAR e a webcam implementada. O vídeo pode ser encontrado no link a seguir: 

<iframe width="560" height="315" src="https://www.youtube.com/embed/_ec88jlOeUk?si=Mryja9VLimU2gOO5" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Fonte: elaborado por SugarZ3ro

&emsp;&emsp;Neste vídeo, é possível observar o robô realizando movimentos lineares e rotacionando conforme os comandos do usuário. Os o novo sistemas de segurança utilizando o sensor infravermelho também foi demonstrado, alem da vizualização em tempo real das imagens capturadas pela camera acoplada ao robo e da latencia dessas imagens. 
