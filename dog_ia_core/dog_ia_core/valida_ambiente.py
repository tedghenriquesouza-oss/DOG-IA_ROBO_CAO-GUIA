python

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

import cv2

from cv_bridge import CvBridge


class ValidaAmbienteNode(Node):
    """
    Nó responsável por verificar e informar o status
    inicial do ambiente do projeto DOG-IA.
    """

    def __init__(self):
        # Inicializa o nó com o nome valida_ambiente_node.
        super().__init__('valida_ambiente_node')

        # Cria o conversor de imagens entre ROS 2 e OpenCV.
        self.bridge = CvBridge()

        # Cria um publicador de mensagens String no tópico dog_ia/status.
        # O número 10 define a profundidade da fila de mensagens.
        self.publisher_ = self.create_publisher(
            String,
            'dog_ia/status',
            10
        )

        # Configura um temporizador para publicar mensagens a cada 2 segundos.
        self.timer = self.create_timer(
            2.0,
            self.publica_status
        )

        # Registra no terminal que o nó foi inicializado.
        self.get_logger().info(
            'DOG-IA: Nó de validação iniciado com sucesso!'
        )

        # Informa a versão do OpenCV disponível no ambiente.
        self.get_logger().info(
            f'OpenCV disponível: versão {cv2.__version__}'
        )

    def publica_status(self):
        """
        Publica periodicamente uma mensagem de status do DOG-IA.
        """

        # Cria uma nova mensagem do tipo String.
        msg = String()

        # Define o conteúdo da mensagem.
        msg.data = 'DOG-IA Operacional: ROS 2 + OpenCV Prontos!'

        # Publica a mensagem no tópico dog_ia/status.
        self.publisher_.publish(msg)

        # Registra no terminal a mensagem enviada.
        self.get_logger().info(
            f'Mensagem publicada: "{msg.data}"'
        )


def main(args=None):
    """
    Inicializa o ROS 2, executa o nó e finaliza a comunicação
    quando o programa é encerrado.
    """

    # Inicializa o sistema de comunicação do ROS 2.
    rclpy.init(args=args)

    # Cria uma instância do nó de validação.
    node = ValidaAmbienteNode()

    try:
        # Mantém o nó em execução e processa os eventos recebidos.
        rclpy.spin(node)

    except KeyboardInterrupt:
