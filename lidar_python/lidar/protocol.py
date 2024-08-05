# lidar/protocol.py
import socket
import struct
from select import poll, POLLIN
import threading
import time
from .packet import *
from .packet import LivoxLidarDeviceAck, LivoxLidarCmdPacket, DirectLidarStateInfo, LivoxLidarEthernetPacket, LivoxLidarCartesianLowRawPoint, LivoxLidarKeyValueParam
from .config import *
from .constants import *
from .crc import calculate_crc16, calculate_crc32
from .device import set_pcl_socket, send_pcl_data, setup_pcl_file_handle, get_pcl_file, pcl_sockfd
from .utils import error, print_buffer
from .enums import LivoxLidarDeviceType, ParamKeyName

g_lidar_info = None

def init_livox_lidar_info_data():
    global g_lidar_info
    g_lidar_info = DirectLidarStateInfo()

def get_livox_lidar_info_data(key):
    kvp = None
    try:
        if key == ParamKeyName.kKeyPclDataType.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.pcl_data_type))
        elif key == ParamKeyName.kKeyPatternMode.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.pattern_mode))
        elif key == ParamKeyName.kKeyLidarIpCfg.value:
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(ip_mask_gw_info.fmt), g_lidar_info.lidar_ipcfg.pack())
        elif key == ParamKeyName.kKeyStateInfoHostIpCfg.value:
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(ip_ports_info.fmt), g_lidar_info.host_info.pack())
        elif key == ParamKeyName.kKeyLidarPointDataHostIpCfg.value:           
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(ip_ports_info.fmt), g_lidar_info.pointcloud_host_ipcfg.pack())
        elif key == ParamKeyName.kKeyLidarImuHostIpCfg.value:
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(ip_ports_info.fmt), g_lidar_info.imu_host_ipcfg.pack())
        elif key == ParamKeyName.kKeyInstallAttitude.value:
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(LivoxLidarInstallAttitude.fmt), g_lidar_info.install_attitude.pack())
        elif key == ParamKeyName.kKeyFovCfg0.value:
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(FovCfg.fmt), g_lidar_info.fov_cfg0.pack())
        elif key == ParamKeyName.kKeyFovCfg1.value:
            kvp = LivoxLidarKeyValueParam(key, struct.calcsize(FovCfg.fmt), g_lidar_info.fov_cfg1.pack())
        elif key == ParamKeyName.kKeyFovCfgEn.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.fov_cfg_en))
        elif key == ParamKeyName.kKeyDetectMode.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.detect_mode))
        elif key == ParamKeyName.kKeyFuncIoCfg.value:
            kvp = LivoxLidarKeyValueParam(key, len(g_lidar_info.func_io_cfg), bytes(g_lidar_info.func_io_cfg))
        elif key == ParamKeyName.kKeyWorkMode.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.work_tgt_mode))
        elif key == ParamKeyName.kKeyImuDataEn.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.imu_data_en))
        elif key == ParamKeyName.kKeySn.value:
            sn_padded = g_lidar_info.sn.ljust(16, b'\x00')
            kvp = LivoxLidarKeyValueParam(key, 16, sn_padded)
        elif key == ParamKeyName.kKeyProductInfo.value:
            product_info_padded = g_lidar_info.product_info + b'\x00' * (64 - len(g_lidar_info.product_info))
            kvp = LivoxLidarKeyValueParam(key, 64, product_info_padded)
        elif key == ParamKeyName.kKeyVersionApp.value:
            kvp = LivoxLidarKeyValueParam(key, 4, bytes(g_lidar_info.version_app))
        elif key == ParamKeyName.kKeyVersionLoader.value:
            kvp = LivoxLidarKeyValueParam(key, 4, bytes(g_lidar_info.version_loader))
        elif key == ParamKeyName.kKeyVersionHardware.value:
            kvp = LivoxLidarKeyValueParam(key, 4, bytes(g_lidar_info.version_hardware))
        elif key == ParamKeyName.kKeyMac.value:
            kvp = LivoxLidarKeyValueParam(key, 6, bytes(g_lidar_info.mac))
        elif key == ParamKeyName.kKeyCurWorkState.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.cur_work_state))
        elif key == ParamKeyName.kKeyCoreTemp.value:
            kvp = LivoxLidarKeyValueParam(key, 4, struct.pack('i', g_lidar_info.core_temp))
        elif key == ParamKeyName.kKeyPowerUpCnt.value:
            kvp = LivoxLidarKeyValueParam(key, 4, struct.pack('I', g_lidar_info.powerup_cnt))
        elif key == ParamKeyName.kKeyLocalTimeNow.value:
            kvp = LivoxLidarKeyValueParam(key, 8, struct.pack('Q', g_lidar_info.local_time_now))
        elif key == ParamKeyName.kKeyLastSyncTime.value:
            kvp = LivoxLidarKeyValueParam(key, 8, struct.pack('Q', g_lidar_info.last_sync_time))
        elif key == ParamKeyName.kKeyTimeOffset.value:
            kvp = LivoxLidarKeyValueParam(key, 8, struct.pack('q', g_lidar_info.time_offset))
        elif key == ParamKeyName.kKeyTimeSyncType.value:
            kvp = LivoxLidarKeyValueParam(key, 1, struct.pack('B', g_lidar_info.time_sync_type))
        elif key == ParamKeyName.kKeyLidarDiagStatus.value:
            kvp = LivoxLidarKeyValueParam(key, 2, struct.pack('H', g_lidar_info.lidar_diag_status))
        elif key == ParamKeyName.kKeyFwType.value:
            try:
                print(f"FW Type: {g_lidar_info.fw_type}")
                fw_type_packed = struct.pack('B', g_lidar_info.fw_type)
                kvp = LivoxLidarKeyValueParam(key, len(fw_type_packed), fw_type_packed)
            except Exception as e:
                print(f"Error processing kKeyFwType: {str(e)}")
                kvp = None
        elif key == ParamKeyName.kKeyHmsCode.value:
            kvp = LivoxLidarKeyValueParam(key, 32, struct.pack('8I', *g_lidar_info.hms_code))
        if kvp is not None:
            pass
        else:
            print(f"get_livox_lidar_info_data: Key {key} not found or unsupported.")
    except struct.error as e:
        print(f"Struct error in get_livox_lidar_info_data: {e}")
    except Exception as e:
        print(f"Error in get_livox_lidar_info_data: {e}")

    return kvp

def set_livox_lidar_info_data(key, kvp):
    print(f"Processing key: {key}, Value: {kvp.value.hex()}")
    global g_lidar_info, pcl_sockfd

    if key == ParamKeyName.kKeyPclDataType.value:
        print("kKeyPclDataType .......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyPclDataType, got {len(kvp.value)}")
        g_lidar_info.pcl_data_type, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyPatternMode.value:
        print("kKeyPatternMode.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyPatternMode, got {len(kvp.value)}")
        g_lidar_info.pattern_mode, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyLidarIpCfg.value:
        print("kKeyLidarIpCfg.......................................")
        if len(kvp.value) != struct.calcsize(ip_mask_gw_info.fmt):
            raise ValueError(f"Expected length {struct.calcsize(ip_mask_gw_info.fmt)} for kKeyLidarIpCfg, got {len(kvp.value)}")
        g_lidar_info.lidar_ipcfg = ip_mask_gw_info.unpack(kvp.value)

    elif key == ParamKeyName.kKeyStateInfoHostIpCfg.value:
        print("kKeyStateInfoHostIpCfg.......................................")
        if len(kvp.value) != struct.calcsize(ip_ports_info.fmt):
            raise ValueError(f"Expected length {struct.calcsize(ip_ports_info.fmt)} for kKeyStateInfoHostIpCfg, got {len(kvp.value)}")
        g_lidar_info.host_info = ip_ports_info.unpack(kvp.value)
        dest_port = socket.ntohs(g_lidar_info.host_info.dest_port)
        src_port = socket.ntohs(g_lidar_info.host_info.src_port)
        print_ipcfg(g_lidar_info.host_info.dest_ip, dest_port, src_port)

    elif key == ParamKeyName.kKeyLidarPointDataHostIpCfg.value:
        print("kKeyLidarPointDataHostIpCfg.......................................")
        if len(kvp.value) != struct.calcsize(ip_ports_info.fmt):
            raise ValueError(f"Expected length {struct.calcsize(ip_ports_info.fmt)} for kKeyLidarPointDataHostIpCfg, got {len(kvp.value)}")
        g_lidar_info.pointcloud_host_ipcfg = ip_ports_info.unpack(kvp.value)
        dest_port = g_lidar_info.pointcloud_host_ipcfg.dest_port
        dest_ip = g_lidar_info.pointcloud_host_ipcfg.dest_ip
        src_port = socket.ntohs(g_lidar_info.pointcloud_host_ipcfg.src_port)
        print(f"POINT CLOUD DATA.......................................")
        print_ipcfg(dest_ip, dest_port, src_port)
        if not isinstance(dest_port, int):
            raise ValueError(f"Expected integer for dest_port, got {type(dest_port)}")

        if set_pcl_socket(dest_ip, dest_port):
           pass

    elif key == ParamKeyName.kKeyLidarImuHostIpCfg.value:
        print("kKeyLidarImuHostIpCfg.......................................")
        if len(kvp.value) != struct.calcsize(ip_ports_info.fmt):
            raise ValueError(f"Expected length {struct.calcsize(ip_ports_info.fmt)} for kKeyLidarImuHostIpCfg, got {len(kvp.value)}")
        g_lidar_info.imu_host_ipcfg = ip_ports_info.unpack(kvp.value)
        dest_port = socket.ntohs(g_lidar_info.imu_host_ipcfg.dest_port)
        src_port = socket.ntohs(g_lidar_info.imu_host_ipcfg.src_port)
        print_ipcfg(g_lidar_info.imu_host_ipcfg.dest_ip, dest_port, src_port)

    elif key == ParamKeyName.kKeyInstallAttitude.value:
        print("kKeyInstallAttitude.......................................")
        if len(kvp.value) != struct.calcsize(LivoxLidarInstallAttitude.fmt):
            raise ValueError(f"Expected length {struct.calcsize(LivoxLidarInstallAttitude.fmt)} for kKeyInstallAttitude, got {len(kvp.value)}")
        g_lidar_info.install_attitude = LivoxLidarInstallAttitude.unpack(kvp.value)

    elif key == ParamKeyName.kKeyFovCfg0.value:
        print("kKeyFovCfg0.......................................")
        if len(kvp.value) != struct.calcsize(FovCfg.fmt):
            raise ValueError(f"Expected length {struct.calcsize(FovCfg.fmt)} for kKeyFovCfg0, got {len(kvp.value)}")
        g_lidar_info.fov_cfg0 = FovCfg.unpack(kvp.value)

    elif key == ParamKeyName.kKeyFovCfg1.value:
        print("kKeyFovCfg1.......................................")
        if len(kvp.value) != struct.calcsize(FovCfg.fmt):
            raise ValueError(f"Expected length {struct.calcsize(FovCfg.fmt)} for kKeyFovCfg1, got {len(kvp.value)}")
        g_lidar_info.fov_cfg1 = FovCfg.unpack(kvp.value)

    elif key == ParamKeyName.kKeyFovCfgEn.value:
        print("kKeyFovCfgEn.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyFovCfgEn, got {len(kvp.value)}")
        g_lidar_info.fov_cfg_en, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyDetectMode.value:
        print("kKeyDetectMode.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyDetectMode, got {len(kvp.value)}")
        g_lidar_info.detect_mode, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyFuncIoCfg.value:
        print("kKeyFuncIoCfg.......................................")
        g_lidar_info.func_io_cfg = list(kvp.value)

    elif key == ParamKeyName.kKeyWorkMode.value:
        print("kKeyWorkMode.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyWorkMode, got {len(kvp.value)}")
        g_lidar_info.work_tgt_mode, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyImuDataEn.value:
        print("kKeyImuDataEn.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyImuDataEn, got {len(kvp.value)}")
        g_lidar_info.imu_data_en, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeySn.value:
        print("kKeySn.......................................")
        g_lidar_info.sn = kvp.value

    elif key == ParamKeyName.kKeyProductInfo.value:
        print("kKeyProductInfo.......................................")
        g_lidar_info.product_info = kvp.value

    elif key == ParamKeyName.kKeyVersionApp.value:
        print("kKeyVersionApp.......................................")
        if len(kvp.value) != 4:
            raise ValueError(f"Expected length 4 for kKeyVersionApp, got {len(kvp.value)}")
        g_lidar_info.version_app = list(kvp.value)

    elif key == ParamKeyName.kKeyVersionLoader.value:
        print("kKeyVersionLoader.......................................")
        if len(kvp.value) != 4:
            raise ValueError(f"Expected length 4 for kKeyVersionLoader, got {len(kvp.value)}")
        g_lidar_info.version_loader = list(kvp.value)

    elif key == ParamKeyName.kKeyVersionHardware.value:
        print("kKeyVersionHardware.......................................")
        if len(kvp.value) != 4:
            raise ValueError(f"Expected length 4 for kKeyVersionHardware, got {len(kvp.value)}")
        g_lidar_info.version_hardware = list(kvp.value)

    elif key == ParamKeyName.kKeyMac.value:
        print("kKeyMac.......................................")
        if len(kvp.value) != 6:
            raise ValueError(f"Expected length 6 for kKeyMac, got {len(kvp.value)}")
        g_lidar_info.mac = list(kvp.value)

    elif key == ParamKeyName.kKeyCurWorkState.value:
        print("kKeyCurWorkState.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyCurWorkState, got {len(kvp.value)}")
        g_lidar_info.cur_work_state, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyCoreTemp.value:
        print("kKeyCoreTemp.......................................")
        if len(kvp.value) != 4:
            raise ValueError(f"Expected length 4 for kKeyCoreTemp, got {len(kvp.value)}")
        g_lidar_info.core_temp, = struct.unpack('i', kvp.value)

    elif key == ParamKeyName.kKeyPowerUpCnt.value:
        print("kKeyPowerUpCnt.......................................")
        if len(kvp.value) != 4:
            raise ValueError(f"Expected length 4 for kKeyPowerUpCnt, got {len(kvp.value)}")
        g_lidar_info.powerup_cnt, = struct.unpack('I', kvp.value)

    elif key == ParamKeyName.kKeyLocalTimeNow.value:
        print("kKeyLocalTimeNow.......................................")
        if len(kvp.value) != 8:
            raise ValueError(f"Expected length 8 for kKeyLocalTimeNow, got {len(kvp.value)}")
        g_lidar_info.local_time_now, = struct.unpack('Q', kvp.value)

    elif key == ParamKeyName.kKeyLastSyncTime.value:
        print("kKeyLastSyncTime.......................................")
        if len(kvp.value) != 8:
            raise ValueError(f"Expected length 8 for kKeyLastSyncTime, got {len(kvp.value)}")
        g_lidar_info.last_sync_time, = struct.unpack('Q', kvp.value)

    elif key == ParamKeyName.kKeyTimeOffset.value:
        print("kKeyTimeOffset.......................................")
        if len(kvp.value) != 8:
            raise ValueError(f"Expected length 8 for kKeyTimeOffset, got {len(kvp.value)}")
        g_lidar_info.time_offset, = struct.unpack('q', kvp.value)

    elif key == ParamKeyName.kKeyTimeSyncType.value:
        print("kKeyTimeSyncType.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyTimeSyncType, got {len(kvp.value)}")
        g_lidar_info.time_sync_type, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyLidarDiagStatus.value:
        print("kKeyLidarDiagStatus.......................................")
        if len(kvp.value) != 2:
            raise ValueError(f"Expected length 2 for kKeyLidarDiagStatus, got {len(kvp.value)}")
        g_lidar_info.lidar_diag_status, = struct.unpack('H', kvp.value)

    elif key == ParamKeyName.kKeyFwType.value:
        print("kKeyFwType.......................................")
        if len(kvp.value) != 1:
            raise ValueError(f"Expected length 1 for kKeyFwType, got {len(kvp.value)}")
        g_lidar_info.fw_type, = struct.unpack('B', kvp.value)

    elif key == ParamKeyName.kKeyHmsCode.value:
        print("kKeyHmsCode.......................................")
        if len(kvp.value) != 32:
            raise ValueError(f"Expected length 32 for kKeyHmsCode, got {len(kvp.value)}")
        g_lidar_info.hms_code = list(struct.unpack('8I', kvp.value))


def print_ipcfg(ip, dest, src):

    if not isinstance(ip, int):
        raise ValueError(f"IP must be an integer, got {type(ip)}")

    ip_str = socket.inet_ntoa(struct.pack('!I', ip))


def create_lidar_udp_socket(s_addr, port, broadcast_enable=0):
    try:
        sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except socket.error as err:
        print(f"Socket creation failed on port {port}: {err}")
        return None

    if broadcast_enable:
        try:
            sockfd.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except socket.error as err:
            print(f"Set socket broadcast option failed on port {port}: {err}")
            sockfd.close()
            return None

    try:
        sockfd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sockfd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, MAXLEN)
        sockfd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, MAXLEN)
        sockfd.bind((s_addr, port))
    except socket.error as err:
        print(f"Socket bind failed on port {port}: {err}")
        sockfd.close()
        return None

    return sockfd

def handle_device_type_query(sockfd, client_addr, req):
    ack = LivoxLidarDeviceAck(0x00, LivoxLidarDeviceType.kLivoxLidarTypeMid360.value, LIDAR_DEVICE_SN.encode('utf-8'), struct.unpack("!L", socket.inet_aton(LIDAR_DEVICE_IP))[0], ctrl_cmd_port_device)
    data = ack.pack()
    response = LivoxLidarCmdPacket(req.sof, req.version, len(data) + 24, req.seq_num, 0x0000, 0x01, 0x01, req.rsvd, 0, 0, data)
    response.crc16_h = calculate_crc16(response.pack()[:18])
    response.crc32_d = calculate_crc32(response.pack()[24:])
    packed_response = response.pack()
    print_buffer(packed_response)
    sockfd.sendto(packed_response, client_addr)
    print(f"Sent device type query response to {client_addr[0]}:{client_addr[1]}")

def handle_parameter_inquire(sockfd, client_addr, req):
    try:
        print("Received Parameter Inquire request with key list:")
        response_buffer = bytearray(MAXLEN)
        response = LivoxLidarCmdPacket(req.sof, req.version, 0, req.seq_num, req.cmd_id, 0x01, 0x01, req.rsvd, 0, 0, b'')
        response_length = 24

        key_num = struct.unpack_from('<H', req.data, 0)[0]
        print(f"Number of keys: {key_num}")

        ack = struct.pack('<BH', 0x00, key_num)
        response_buffer[24:27] = ack
        response_length += 3

        key_list = req.data[4:4 + 2 * key_num]

        for i in range(key_num):
            key = struct.unpack_from('<H', key_list, i * 2)[0]
            print(f"Key {i}: 0x{key:04x}")
            kvp = get_livox_lidar_info_data(key)
            if kvp is None:
               ack = LivoxLidarParamInquireAck(0x01, 1, 0)
               response_buffer[24:27] = ack.pack()
               response_length = 27
               break
            kvp_packed = kvp.pack()
            response_buffer[response_length:response_length + len(kvp_packed)] = kvp_packed
            response_length += len(kvp_packed)

            print(f"Packed Key-Value Pair {i}: ", end="")
            print_buffer(kvp_packed)

        response.data = response_buffer[24:response_length]
        response.length = response_length
        response.crc16_h = calculate_crc16(response.pack()[:18])
        response.crc32_d = calculate_crc32(response.pack()[24:])
        packed_response = response.pack()

        print_buffer(packed_response)

        sockfd.sendto(packed_response, client_addr)
        print(f"Sent parameter inquire response to {client_addr[0]}:{client_addr[1]}")
    except Exception as e:
        print(f"Error in handle_parameter_inquire: {e}")

def handle_parameter_configuration(sockfd, client_addr, req):
    try:
        kvp_list = []
        data_ptr = 0
        while data_ptr < len(req.data):
            remaining_data = len(req.data) - data_ptr
            expected_header_size = struct.calcsize(LivoxLidarKeyValueParam.fmt)

            if remaining_data < expected_header_size:
                break

            key, length = struct.unpack(LivoxLidarKeyValueParam.fmt, req.data[data_ptr:data_ptr + expected_header_size])
            data_ptr += expected_header_size

            if length == 0:
                continue

            if remaining_data < expected_header_size + length:
                break

            value = req.data[data_ptr:data_ptr + length]
            data_ptr += length
            kvp = LivoxLidarKeyValueParam(key, length, value)
            kvp_list.append(kvp)
            print(f"Unpacked Key: {key}, Length: {length}, Value: {value.hex()}")

        for i, kvp in enumerate(kvp_list):
            print(f"Setting key {i}: {kvp.key}, Value: {kvp.value.hex()}")
            set_livox_lidar_info_data(kvp.key, kvp)

        ack = struct.pack('<BH', 0x00, 0x0000)

        response = LivoxLidarCmdPacket(req.sof, req.version, len(ack) + 24, req.seq_num, req.cmd_id, 0x01, 0x01, req.rsvd, 0, 0, ack)
        response.crc16_h = calculate_crc16(response.pack()[:18])
        response.crc32_d = calculate_crc32(response.pack()[24:])
        packed_response = response.pack()

        print_buffer(packed_response)

        sockfd.sendto(packed_response, client_addr)
        print(f"Sent parameter configuration response to {client_addr[0]}:{client_addr[1]}")
    except Exception as e:
        print(f"Error in handle_parameter_configuration: {e}")
        exit()

def handle_command_protocol():
    sockets = []

    sockets.append(create_lidar_udp_socket('0.0.0.0', broadcast_port, 1))
    sockets.append(create_lidar_udp_socket(LIDAR_DEVICE_IP, ctrl_cmd_port_device, 0))

    fds = []
    for sock in sockets:
        if sock is not None:
            fds.append({'fd': sock, 'events': POLLIN})

    poller = poll()
    for fd in fds:
        poller.register(fd['fd'], fd['events'])

    try:
        while True:
            events = poller.poll()

            for fd, event in events:
                if event & POLLIN:
                    sock = None
                    for s in sockets:
                        if s.fileno() == fd:
                            sock = s
                            break

                    if sock is None:
                        print(f"No matching socket found for fd {fd}")
                        continue

                    buffer = bytearray(MAXLEN)
                    n, client_address = sock.recvfrom_into(buffer)
                    if n < 0:
                        print(f"recvfrom error on socket {sock.fileno()}")
                        continue

                    print(f"CLIENT IP ......{client_address[0]} PORT .............{client_address[1]}")

                    pkt = LivoxLidarCmdPacket.unpack(buffer)
                    cmd_type = pkt.cmd_type
                    cmd_id = pkt.cmd_id

                    print(f"\n cmd_type 0x{cmd_type:x}, cmd_id= 0x{cmd_id:x} length {n}")

                    if cmd_type == 0x00:
                        if cmd_id == 0x0000:
                            handle_device_type_query(sock, client_address, pkt)
                        elif cmd_id == 0x0100:
                            handle_parameter_configuration(sock, client_address, pkt)
                        elif cmd_id == 0x0101:
                            handle_parameter_inquire(sock, client_address, pkt)
                        else:
                            print(f"Unknown command ID: {cmd_id}")
                    else:
                        print(f"Unknown command type: {cmd_type}")
    except KeyboardInterrupt:
        print("Terminating handle_command_protocol.")
    finally:
        for sock in sockets:
            sock.close()

    return 0

def read_pointclouddata_oneline_from_file(file, timestamp):
    while True:
        line = file.readline()
        if not line:
            return None

        tokens = line.split()
        if len(tokens) < 4:
            continue

        try:
            x = int(tokens[0])
            y = int(tokens[1])
            z = int(tokens[2])
            reflectivity = int(tokens[3])
        except ValueError:
            continue

        static_time = getattr(read_pointclouddata_oneline_from_file, "static_time", 0)
        if static_time == 0:
            static_time = timestamp[0]
            setattr(read_pointclouddata_oneline_from_file, "static_time", static_time)
        else:
            static_time += 480765
            setattr(read_pointclouddata_oneline_from_file, "static_time", static_time)
        timestamp[0] = static_time

        point = LivoxLidarCartesianLowRawPoint(x, y, z, reflectivity, 0)
        print(point)
        return point.pack()

def get_livox_lidar_pointcloud_data(response_buff, timestamp):
    data = read_pointclouddata_oneline_from_file(get_pcl_file(), timestamp)
    if data is None:
        return 0

    response_buff.extend(data)
    return len(data)

def handle_parameter_pointcloud_data(resp_buffer, data_seq):
    local_buffer = bytearray()
    save_buffer_start = bytes(resp_buffer)
    resp_buffer_len = 0
    len1 = struct.calcsize(LivoxLidarEthernetPacket.fmt)
    dot_num = 0
    timestamp = [0]

    while dot_num < POINTCLOUDDATAMAX:
        resp_buffer_len += len1

        len1 = get_livox_lidar_pointcloud_data(local_buffer, timestamp)
        if len1 == 0:
            break
        dot_num += 1

    print(f"shamil length:::::::::::::::::{resp_buffer_len}")

    packet = LivoxLidarEthernetPacket(
        version=0,
        length=resp_buffer_len,
        time_interval=0,
        dot_num=dot_num,
        udp_cnt=data_seq,
        frame_cnt=1,
        data_type=2,
        time_type=0,
        rsvd=b'\x00' * 12,
        crc32=0,
        timestamp=23875872534872,
        data=local_buffer
    )
    packed_packet = packet.pack()

    temp = bytearray()
    temp = packed_packet + local_buffer
    hex_data = temp[28:resp_buffer_len].hex()
    crc32_value = calculate_crc32(bytes.fromhex(hex_data))
    packet.crc32 = crc32_value
    packed_packet = packet.pack()
    resp_buffer.extend(packed_packet)
    resp_buffer.extend(local_buffer)

    return resp_buffer_len

def handle_pcl_data_from_file():
    global pcl_sockfd
    buffer = bytearray()
    seq_num = 0
    buffer_len = 0

    time.sleep(10)

    print(f"sizeof data2 {struct.calcsize(LivoxLidarCartesianLowRawPoint.fmt)}")

    try:
        while True:
            print(f"Reading point cloud data for sequence number: {seq_num}")
            buffer_len = handle_parameter_pointcloud_data(buffer, seq_num)
            if buffer_len < 100:
                print("Buffer length less than 100, exiting.")
                return 0
            print(f"Sending point cloud data for sequence number: {seq_num} with buffer length: {buffer_len}")
            if send_pcl_data(buffer, buffer_len) < 0:
                print("Error sending message to device")
                return -1
            print(f"handlePCLdatafromfile .............{seq_num} Length: {buffer_len}")
            buffer.clear()
            seq_num += 1
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Terminating handle_pcl_data_from_file.")

    finally:
        if pcl_sockfd:
            print(f"Closing socket with descriptor: {pcl_sockfd.fileno()}")
            pcl_sockfd.close()

    return 0
