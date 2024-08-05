import sys
import threading
from lidar.protocol import handle_command_protocol, handle_pcl_data_from_file, init_livox_lidar_info_data
from lidar.device import setup_pcl_file_handle, pcl_sockfd

def main():
    if len(sys.argv) != 2:
        print("Params Invalid, must input config path.")
        sys.exit(-1)

    path = sys.argv[1]

    if setup_pcl_file_handle(path) < 0:
        print("Params Invalid, must input valid file path.")
        sys.exit(-2)

    init_livox_lidar_info_data()
    if pcl_sockfd == 0:
        print("Failed to set up PCL socket.")
        sys.exit(-3)

    t1 = threading.Thread(target=handle_command_protocol)
    t2 = threading.Thread(target=handle_pcl_data_from_file)

    t1.start()
    t2.start()

    try:
        t1.join()
        t2.join()
    except KeyboardInterrupt:
        print("Terminating main threads.")

if __name__ == "__main__":
    main()

