# lidar/config.py

# Define Livox Mid360 specific ports
broadcast_port = 56000  # Broadcast port for device type query
ctrl_cmd_port_device = 56100  # Command port for all operations
ctrl_cmd_port_host = 56101  # Command port for all operations
push_cmd_port_device = 56200  # Push data
push_cmd_port_host = 56201  # Push Data
point_data_port_device = 56300  # Point Cloud Data
point_data_port_host = 56301  # Point Cloud Data
imu_data_port_device = 56400  # IMU Data
imu_data_port_host = 56401  # IMU Data
log_data_port_device = 56500  # LOG Data
log_data_port_host = 56501  # LOG Data

MAXLEN = 1400
MAX_SOCKETS = 20
POINTCLOUDDATAMAX = 96

LIDAR_HOST_IP = "192.168.1.47"
LIDAR_DEVICE_IP = "192.168.1.44"
LIDAR_DEVICE_SN = "Tux-LivoxLidar1"
LIDAR_DEVICE_MASK = "255.255.255.0"
LIDAR_DEVICE_GATEWAY = "192.168.68.1"

DEVICE_MAC = (0x7c, 0x7a, 0x91, 0x33, 0xbe, 0x3b)
