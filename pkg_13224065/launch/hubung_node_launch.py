from launch import LaunchDescription
from launch_ros.actions import Node

# File Launch Digunakan untuk menjalankan node hubung_node
# Node ini menghubungkan data dari topik autonomous_vel ke cmd_vel dan cmd_type
# Dengan parameter yang diambil dari file config/config.yaml
def generate_launch_description():
    #Merupakan fungsi yang mengembalikan LaunchDescription
    return LaunchDescription([    
        # Node hubung_node yang akan dijalankan
        Node(
            package='pkg_13224065',                 # Nama package yang berisi node
            executable='node_hubung',               # Nama executable dari node yang akan dijalankan
            name='hubung_node',                     # Nama node yang akan digunakan
            parameters=['config/config.yaml'],      # Menggunakan file config config.yaml untuk parameter
            output='screen'                         # Output dari node akan ditampilkan di terminal
        )
    ])

