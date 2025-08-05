#Import library ROS2 yang diperlukan
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

#Definisikan kelas HubungNode yang merupakan turunan dari Node
#Node ini akan menghubungkan data dari topik autonomous_vel ke cmd_vel dan cmd_type
#Dengan parameter yang diambil dari file config/config.yaml
class HubungNode(Node):
    #Difinesikan konstruktor untuk inisialisasi node
    def __init__(self):
        #Inisialisasi node dengan nama 'hubung_node'
        super().__init__('hubung_node')

        #Membuat publisher untuk topik cmd_vel dan cmd_type
        #cmd_vel untuk mengirimkan data ke robot
        #cmd_type untuk mengirimkan tipe perintah yang digunakan
        #Publisher ini akan mengirimkan pesan dengan tipe Twist untuk cmd_vel dan String untuk cmd
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_type_publisher = self.create_publisher(String, 'cmd_type', 10)

        #Membuat subscriber untuk topik autonomous_vel
        #Subscriber ini akan menerima pesan dengan tipe Twist dari topik autonomous_vel
        self.autonomous_vel_subscriber = self.create_subscription(
            Twist,
            'autonomous_vel',
            self.listener_callback,
            10
        )

    # Callback function yang akan dipanggil ketika menerima pesan dari topik autonomous_vel
    #Fungsi ini akan meneruskan pesan ke cmd_vel dan mengirimkan tipe perintah 'autonomous' ke cmd_type
    def listener_callback(self, msg):
        self.cmd_vel_publisher.publish(msg)

        cmd_type_msg = String()
        cmd_type_msg.data = 'autonomous'
        self.cmd_type_publisher.publish(cmd_type_msg)

#Merupakan fungsi utama untuk menjalankan node
#Fungsi ini akan menginisialisasi rclpy, membuat instance dari HubungNode,
#dan menjalankan node tersebut hingga dihentikan
#Setelah selesai, node akan dihancurkan dan rclpy akan dimatikan
def main(args=None):
    rclpy.init(args=args)

    node = HubungNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()