# lidar/device.py
import socket
import struct
from .config import LIDAR_DEVICE_IP, point_data_port_device, LIDAR_HOST_IP, point_data_port_host, DEVICE_MAC

pcl_sockfd = None
pcl_host_socket = None
pcl_file = None
original_ip = None

def get_pcl_host_socket():
    return pcl_host_socket

def set_pcl_socket(ip_addr, pcl_port_host):
    global pcl_sockfd, pcl_host_socket, original_ip

    if pcl_sockfd is not None:
        print(f"Closing existing socket with descriptor: {pcl_sockfd}")
        pcl_sockfd.close()
        pcl_sockfd = None

    try:
        pcl_sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if pcl_sockfd.fileno() < 0:
            raise socket.error(f"Error creating socket. IP: {ip_addr} port: {pcl_port_host}")
        print(f"Socket created successfully. Descriptor: {pcl_sockfd.fileno()}")
        print(f"set_pcl_socket...............IP ADDR : {ip_addr} PORT : {pcl_port_host}")
    except socket.error as e:
        print(f"Error creating socket. IP: {ip_addr} port: {pcl_port_host} Error: {e}")
        return 0

    if isinstance(ip_addr, int):
        ip_addr = socket.inet_ntoa(struct.pack('!I', ip_addr))

    pcl_host_socket = (ip_addr, int(pcl_port_host))
    original_ip = ip_addr  # Store the original IP address

    device_addr = (LIDAR_DEVICE_IP, point_data_port_device)

    print(f"Binding to device address - IP: {LIDAR_DEVICE_IP} Port: {point_data_port_device}")

    try:
        pcl_sockfd.bind(device_addr)
        print(f"Socket successfully bound. Descriptor: {pcl_sockfd.fileno()}")
    except socket.error as e:
        print(f"Error binding to port {point_data_port_device} Error: {e}")
        pcl_sockfd.close()
        pcl_sockfd = None
        return 0

    return pcl_sockfd


def send_pcl_data(buffer2, length):
    global pcl_sockfd, pcl_host_socket, original_ip
    if pcl_sockfd is None or pcl_sockfd.fileno() <= 0:
        print(f"Invalid socket descriptor: {pcl_sockfd}")
        return -1
    correct_port = point_data_port_host

    # Temporarily reverse the IP address
    reversed_ip = socket.inet_ntoa(socket.inet_aton(pcl_host_socket[0])[::-1])
    pcl_host_socket = (reversed_ip, correct_port)

    try:
        bytes_sent = pcl_sockfd.sendto(buffer2[:length], pcl_host_socket)
    except socket.error as e:
        print(f"Error sending message to host. Error: {e}")
        return -1

    if bytes_sent < 0:
        print("Error sending message to host")
        return -1
    else:
        print(f"Sent {bytes_sent} bytes to host")

    # Reset the IP address to the original
    pcl_host_socket = (original_ip, correct_port)
    return bytes_sent


def setup_pcl_file_handle(filename):
    global pcl_file
    pcl_file = open(filename, 'r')
    if pcl_file is None:
        print(f"Failed to open file: {filename}")
        return -1
    for _ in range(11):
        pcl_file.readline()
    return 0



def get_pcl_file():
    return pcl_file
