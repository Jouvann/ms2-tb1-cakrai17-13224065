#Import library ROS2 yang diperlukan
import rclpy
from rclpy.node import Node
from rclpy.time import Time

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


        # DEKLARASI PARAMETER DEFAULT
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('enabled', True)
        self.declare_parameter('log_verbose', True)

        self.declare_parameter('linear_multiplier', 1.0)
        self.declare_parameter('angular_multiplier', 1.0)

        self.declare_parameter('autonomous_vel_topic', '/autonomous_vel')
        self.declare_parameter('joy_vel_topic', '/joy_vel')
        self.declare_parameter('keyboard_vel_topic', '/keyboard_vel')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_type_topic', '/cmd_type')

        # MEMBACA NILAI PARAMETER DARI FILE CONFIG
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enabled = self.get_parameter('enabled').value
        self.log_verbose = self.get_parameter('log_verbose').value

        self.linear_multiplier = self.get_parameter('linear_multiplier').value
        self.angular_multiplier = self.get_parameter('angular_multiplier').value

        self.autonomous_vel_topic = self.get_parameter('autonomous_vel_topic').value
        self.joy_vel_topic = self.get_parameter('joy_vel_topic').value
        self.keyboard_vel_topic = self.get_parameter('keyboard_vel_topic').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_type_topic = self.get_parameter('cmd_type_topic').value


        # PUBLISHER DAN SUBSCRIBER
        #Membuat publisher untuk topik cmd_vel dan cmd_type
        #cmd_vel untuk mengirimkan data ke robot
        #cmd_type untuk mengirimkan tipe perintah yang digunakan
        #Publisher ini akan mengirimkan pesan dengan tipe Twist untuk cmd_vel dan String untuk cmd
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_type_publisher = self.create_publisher(String, self.cmd_type_topic, 10)

        #Membuat subscriber untuk topik autonomous_vel
        #Subscriber ini akan menerima pesan dengan tipe Twist dari topik autonomous_vel, joy_vel, dan keyboard_vel
        self.autonomous_vel_subscriber = self.create_subscription(
            Twist,
            self.autonomous_vel_topic,
            self.autonomous_callback,
            10
        )
        self.joy_vel_subscriber = self.create_subscription(
            Twist,
            self.joy_vel_topic,
            self.joy_callback,
            10
        )
        self.keyboard_vel_subscriber = self.create_subscription(
            Twist,
            self.keyboard_vel_topic,
            self.keyboard_callback,
            10
        )

        #Menyimpan pesan terakhir yang diterima dari masing-masing topik
        self.last_msgs = {
            'keyboard' : (None, Time()),
            'joy' : (None, Time()),
            'autonomous' : (None, Time())
        }

        #Start Timer pembacaan jenis input
        self.timer = self.create_timer(1.0/self.publish_rate , self.publish_decision)

        # Print informasi refresh rate dan status node
        self.get_logger().info(f'Node {self.get_name()} Terinisialisasi dengan Refresh Rate {self.publish_rate} Hz')
        if not self.enabled:
            self.get_logger().warn('Node ini tidak aktif dari file config')

    #Definisi fungsi membaca pesan dari topik autonomous_vel, joy_vel, dan keyboard_vel
    #Fungsi ini akan menyimpan pesan terakhir yang diterima dari masing-masing topik
    def autonomous_callback(self, msg):
        self.last_msgs['autonomous'] = (msg, self.get_clock().now())

    def joy_callback(self, msg):
        self.last_msgs['joy'] = (msg, self.get_clock().now())

    def keyboard_callback(self, msg):
        self.last_msgs['keyboard'] = (msg, self.get_clock().now())

    def publish_decision(self):
        if not self.enabled:
            return
        
        #Insisialisasi waktu sekarang dan variabel untuk menyimpan sumber aktif dan pesan aktif
        now = self.get_clock().now()
        active_source = None
        active_msg = None

        #priotitas keyboard > joy > autonomous
        #Periksa pesan terakhir yang diterima dari masing-masing topik tiap 0.2 detik
        for source in ['keyboard', 'joy', 'autonomous']:
            msg, timestamp = self.last_msgs[source]
            if msg is not None and (now - timestamp).nanoseconds / 1e9 < 0.5:
                active_source = source
                active_msg = msg
                break

        #Jika tidak ada pesan yang diterima, tampilkan peringatan
        if active_msg is None:
            if self.log_verbose:
                self.get_logger().warn('Belum ada pesan masuk')
            return
            
        #Tambah Mekanisme Menskalakan pesan Twist via config
        #Menggunakan parameter linear_multiplier dan angular_multiplier untuk mengubah kecepatan linear dan angular
        #Jujur karena gabut ka, file config kosong banget soalnya
        ScaledMsg = Twist()
        ScaledMsg.linear.x = active_msg.linear.x * self.linear_multiplier
        ScaledMsg.angular.z = active_msg.angular.z * self.angular_multiplier

        self.cmd_vel_publisher.publish(ScaledMsg)   

        cmd_type_msg = String()
        cmd_type_msg.data = active_source
        self.cmd_type_publisher.publish(cmd_type_msg)

        # Verbose log
         # Log informasi tentang pesan yang dipublikasikan
        if self.log_verbose:
            self.get_logger().info(
                f"[hubung_node] Published scaled Twist -> linear.x: {ScaledMsg.linear.x:.2f} m/s, angular.z: {ScaledMsg.angular.z:.2f} rad/s"
            )



   
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