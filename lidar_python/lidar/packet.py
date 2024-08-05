import struct
import socket
from .config import LIDAR_DEVICE_IP, LIDAR_DEVICE_MASK, LIDAR_DEVICE_GATEWAY, LIDAR_HOST_IP, DEVICE_MAC, point_data_port_device, point_data_port_host, push_cmd_port_device, push_cmd_port_host, imu_data_port_device, imu_data_port_host, log_data_port_device, log_data_port_host, LIDAR_DEVICE_SN

class LivoxLidarInfo:
    fmt = 'B16s16s'  # Define the format string for struct packing/unpacking

    def __init__(self, dev_type, sn, lidar_ip):
        self.dev_type = dev_type
        self.sn = sn
        self.lidar_ip = lidar_ip

    def pack(self):
        return struct.pack(self.fmt, self.dev_type, self.sn.encode('utf-8'), self.lidar_ip.encode('utf-8'))

    @classmethod
    def unpack(cls, data):
        dev_type, sn, lidar_ip = struct.unpack(cls.fmt, data)
        return cls(dev_type, sn.decode('utf-8').strip('\x00'), lidar_ip.decode('utf-8').strip('\x00'))

class LivoxLidarDeviceAck:
    fmt = '<BB16s4sH'

    def __init__(self, ret_code, dev_type, sn, lidar_ip, cmd_port):
        self.ret_code = ret_code
        self.dev_type = dev_type
        self.sn = sn
        self.lidar_ip = lidar_ip
        self.cmd_port = cmd_port

    def pack(self):
        return struct.pack(self.fmt, self.ret_code, self.dev_type, self.sn[:16], struct.pack("!L", self.lidar_ip), self.cmd_port)

    @classmethod
    def unpack(cls, buffer):
        ret_code, dev_type, sn, lidar_ip_packed, cmd_port = struct.unpack(cls.fmt, buffer)
        sn = sn.decode('utf-8').rstrip('\x00')
        lidar_ip = struct.unpack("!L", socket.inet_ntoa(lidar_ip_packed).encode('utf-8'))[0]
        return cls(ret_code, dev_type, sn, lidar_ip, cmd_port)


class LivoxLidarParamInquire:
    def __init__(self, key_num, key_list):
        self.key_num = key_num
        self.key_list = key_list

    @staticmethod
    def unpack(buffer):
        key_num, = struct.unpack_from('<H', buffer, 0)
        key_list = []
        offset = 4
        for _ in range(key_num):
            key, = struct.unpack_from('<H', buffer, offset)
            key_list.append(key)
            offset += 2
        return LivoxLidarParamInquire(key_num, key_list)


class LivoxLidarParamInquireAck:
    fmt = "BHH"

    def __init__(self, ret_code, key_num, key_value_list):
        self.ret_code = ret_code
        self.key_num = key_num
        self.key_value_list = key_value_list

    def pack(self):
        return struct.pack(self.fmt, self.ret_code, self.key_num, self.key_value_list)


class LivoxLidarParamConfig:
    fmt = "HH"

    def __init__(self, key_num, rsvd, key_list):
        self.key_num = key_num
        self.rsvd = rsvd
        self.key_list = key_list

    def pack(self):
        return struct.pack(self.fmt, self.key_num, self.rsvd) + b''.join([param.pack() for param in self.key_list])

    @classmethod
    def unpack(cls, data):
        key_num, rsvd = struct.unpack(cls.fmt, data[:struct.calcsize(cls.fmt)])
        key_list = []
        offset = struct.calcsize(cls.fmt)
        for _ in range(key_num):
            param = LivoxLidarKeyValueParam.unpack(data[offset:])
            key_list.append(param)
            offset += struct.calcsize(LivoxLidarKeyValueParam.fmt) + param.length
        return cls(key_num, rsvd, key_list)

class LivoxLidarAsyncControlResponse:
    fmt = "BH"

    def __init__(self, ret_code, error_key):
        self.ret_code = ret_code
        self.error_key = error_key

    def pack(self):
        return struct.pack(self.fmt, self.ret_code, self.error_key)

    @classmethod
    def unpack(cls, data):
        return cls(*struct.unpack(cls.fmt, data[:struct.calcsize(cls.fmt)]))


class LivoxLidarKeyValueParam:
    fmt = "HH"

    def __init__(self, key, length, value):
        self.key = key
        self.length = length
        self.value = value

    def pack(self):
        return struct.pack(self.fmt, self.key, self.length) + self.value

    @classmethod
    def unpack(cls, data):
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError("Data too short to unpack header")
        key, length = struct.unpack(cls.fmt, data[:struct.calcsize(cls.fmt)])
        if len(data) < struct.calcsize(cls.fmt) + length:
            raise ValueError("Data too short to unpack value")
        value = data[struct.calcsize(cls.fmt):struct.calcsize(cls.fmt) + length]
        return cls(key, length, value)

class LivoxLidarEthernetPacket:
    fmt = '<BHHHHBBB12sLQ'

    def __init__(self, version, length, time_interval, dot_num, udp_cnt, frame_cnt, data_type, time_type, rsvd, crc32, timestamp, data):
        self.version = version & 0xFF
        self.length = length
        self.time_interval = time_interval
        self.dot_num = dot_num
        self.udp_cnt = udp_cnt
        self.frame_cnt = frame_cnt & 0xFF
        self.data_type = data_type & 0xFF
        self.time_type = time_type & 0xFF
        self.rsvd = rsvd
        self.crc32 = crc32
        self.timestamp = timestamp
        self.data = data

    def pack(self):
        header = struct.pack(self.fmt, self.version, self.length, self.time_interval, self.dot_num, self.udp_cnt, self.frame_cnt, self.data_type, self.time_type, self.rsvd, self.crc32, self.timestamp)
        return header 

    @classmethod
    def unpack(cls, buffer):
        header_size = struct.calcsize(cls.fmt)
        header = buffer[:header_size]
        data = buffer[header_size:]
        unpacked_data = struct.unpack(cls.fmt, header)
        return cls(*unpacked_data, data)

class LivoxLidarCmdPacket:
    fmt = '<BBHLHBB6sHL'

    def __init__(self, sof, version, length, seq_num, cmd_id, cmd_type, sender_type, rsvd, crc16_h, crc32_d, data):
        self.sof = sof
        self.version = version
        self.length = length
        self.seq_num = seq_num
        self.cmd_id = cmd_id
        self.cmd_type = cmd_type
        self.sender_type = sender_type
        self.rsvd = rsvd
        self.crc16_h = crc16_h
        self.crc32_d = crc32_d
        self.data = data

    def pack(self):
        header = struct.pack(self.fmt, self.sof, self.version, self.length, self.seq_num, self.cmd_id, self.cmd_type, self.sender_type, self.rsvd, self.crc16_h, self.crc32_d)
        return header + self.data

    @staticmethod
    def unpack(buffer):
        if len(buffer) < struct.calcsize(LivoxLidarCmdPacket.fmt):
            raise ValueError(f"Buffer too short to unpack: length {len(buffer)}")
        unpacked = struct.unpack_from(LivoxLidarCmdPacket.fmt, buffer)
        data = buffer[struct.calcsize(LivoxLidarCmdPacket.fmt):]
        return LivoxLidarCmdPacket(*unpacked, data)

class LivoxLidarCartesianHighRawPoint:
    fmt = "iiiBB"

    def __init__(self, x, y, z, reflectivity, tag=0):
        self.x = x
        self.y = y
        self.z = z
        self.reflectivity = reflectivity
        self.tag = tag

    def pack(self):
        return struct.pack(self.fmt, self.x, self.y, self.z, self.reflectivity, self.tag)

    @classmethod
    def unpack(cls, data):
        unpacked_data = struct.unpack(cls.fmt, data[:struct.calcsize(cls.fmt)])
        return cls(*unpacked_data)


class LivoxLidarCartesianLowRawPoint:
    fmt = '<hhhBB'  # Ensure little-endian format with 'hhhBB'

    def __init__(self, x, y, z, reflectivity, tag):
        self.x = x
        self.y = y
        self.z = z
        self.reflectivity = reflectivity
        self.tag = tag

    def pack(self):
        x = max(-32768, min(32767, self.x))
        y = max(-32768, min(32767, self.y))
        z = max(-32768, min(32767, self.z))
        reflectivity = max(0, min(255, self.reflectivity))
        return struct.pack(self.fmt, x, y, z, reflectivity, self.tag)

    @classmethod
    def unpack(cls, buffer):
        unpacked_data = struct.unpack(cls.fmt, buffer)
        return cls(*unpacked_data)

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z}, :{self.reflectivity}, :{self.tag})"

class ip_mask_gw_info:
    fmt = '!III'

    def __init__(self, ip, mask, gateway):
        self.ip = ip
        self.mask = mask
        self.gateway = gateway

    def pack(self):
        return struct.pack(self.fmt, self.ip, self.mask, self.gateway)

    @classmethod
    def unpack(cls, buffer):
        return cls(*struct.unpack(cls.fmt, buffer))


class ip_ports_info:
    fmt = 'IHH'

    def __init__(self, dest_ip, dest_port, src_port):
        self.dest_ip = dest_ip
        self.dest_port = dest_port
        self.src_port = src_port

    def pack(self):
        packed_data = struct.pack(self.fmt, self.dest_ip, self.dest_port, self.src_port)
        return packed_data

    @classmethod
    def unpack(cls, buffer):
        dest_ip, dest_port, src_port = struct.unpack(cls.fmt, buffer)
        return cls(dest_ip, dest_port, src_port)

class LivoxLidarInstallAttitude:
    fmt = 'fffIII'

    def __init__(self, roll_deg, pitch_deg, yaw_deg, x_mm, y_mm, z_mm):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.z_mm = z_mm

    def pack(self):
        return struct.pack(self.fmt, self.roll_deg, self.pitch_deg, self.yaw_deg, self.x_mm, self.y_mm, self.z_mm)

    @classmethod
    def unpack(cls, buffer):
        return cls(*struct.unpack(cls.fmt, buffer))


class FovCfg:
    fmt = 'iiiiI'

    def __init__(self, yaw_start, yaw_stop, pitch_start, pitch_stop, rsvd):
        self.yaw_start = yaw_start
        self.yaw_stop = yaw_stop
        self.pitch_start = pitch_start
        self.pitch_stop = pitch_stop
        self.rsvd = rsvd

    def pack(self):
        return struct.pack(self.fmt, self.yaw_start, self.yaw_stop, self.pitch_start, self.pitch_stop, self.rsvd)

    @classmethod
    def unpack(cls, buffer):
        return cls(*struct.unpack(cls.fmt, buffer))


class DirectLidarStateInfo:
    fmt = (
        'BBBB'     # pcl_data_type, pattern_mode, dual_emit_en, point_send_en
        'III'      # lidar_ipcfg
        'IHH'      # host_info
        'IHH'      # pointcloud_host_ipcfg
        'IHH'      # imu_host_ipcfg
        '16sHH'    # ctl_host_ipcfg (16s for ip_addr, 2H for ports)
        '16sHH'    # log_host_ipcfg (16s for ip_addr, 2H for ports)
        'ii'       # vehicle_speed, environment_temp
        'fffiii'   # install_attitude
        'I'        # blind_spot_set
        'B'        # frame_rate
        'iiiiI'    # fov_cfg0
        'iiiiI'    # fov_cfg1
        'B'        # fov_cfg_en
        'B'        # detect_mode
        '4B'       # func_io_cfg
        'B'        # work_tgt_mode
        'B'        # glass_heat
        'B'        # imu_data_en
        'B'        # fusa_en
        '16s'      # sn
        '64s'      # product_info
        '4B'       # version_app
        '4B'       # version_loader
        '4B'       # version_hardware
        '6B'       # mac
        'B'        # cur_work_state
        'i'        # core_temp
        'I'        # powerup_cnt
        'Q'        # local_time_now
        'Q'        # last_sync_time
        'q'        # time_offset
        'B'        # time_sync_type
        '32B'      # status_code
        'H'        # lidar_diag_status
        'B'        # lidar_flash_status
        'B'        # fw_type
        '8I'       # hms_code
        'B'        # ROI_Mode
    )

    def __init__(self):
        self.pcl_data_type = 0x01
        self.pattern_mode = 0x00
        self.dual_emit_en = 0x00
        self.point_send_en = 0x00
        self.lidar_ipcfg = ip_mask_gw_info(
            struct.unpack("!I", socket.inet_aton(LIDAR_DEVICE_IP))[0],
            struct.unpack("!I", socket.inet_aton(LIDAR_DEVICE_MASK))[0],
            struct.unpack("!I", socket.inet_aton(LIDAR_DEVICE_GATEWAY))[0]
        )
        self.host_info = ip_ports_info(
            struct.unpack("I", socket.inet_aton(LIDAR_HOST_IP))[0],
            56201,  # Correct port for push_cmd_port_host
            56200   # Correct port for push_cmd_port_device
        )
        self.pointcloud_host_ipcfg = ip_ports_info(
            struct.unpack("I", socket.inet_aton(LIDAR_HOST_IP))[0],
            56301,  # Correct port for point_data_port_host
            56300   # Correct port for point_data_port_device
        )
        self.imu_host_ipcfg = ip_ports_info(
            struct.unpack("I", socket.inet_aton(LIDAR_HOST_IP))[0],
            56401,  # Correct port for imu_data_port_host
            56400   # Correct port for imu_data_port_device
        )
        self.ctl_host_ipcfg = (LIDAR_HOST_IP.encode('utf-8'), 56501, 56500)  # Correct ports for log_data_port_host and log_data_port_device
        self.log_host_ipcfg = (LIDAR_HOST_IP.encode('utf-8'), 56501, 56500)  # Correct ports for log_data_port_host and log_data_port_device
        self.vehicle_speed = 0
        self.environment_temp = 0
        self.install_attitude = LivoxLidarInstallAttitude(0.0, 0.0, 0.0, 0, 0, 0)
        self.blind_spot_set = 0
        self.frame_rate = 0x00
        self.fov_cfg0 = FovCfg(0, 360, -10, 60, 0)
        self.fov_cfg1 = FovCfg(0, 360, -10, 60, 0)
        self.fov_cfg_en = 0x03
        self.detect_mode = 0x00
        self.func_io_cfg = [0x00] * 4
        self.work_tgt_mode = 0x01
        self.glass_heat = 0x00
        self.imu_data_en = 0x01
        self.fusa_en = 0x00
        self.sn = LIDAR_DEVICE_SN.encode('utf-8')
        self.product_info = "Livox Lidar Mid-360 2021/12/01".encode('utf-8')
        self.version_app = [0x01, 0x02, 0x03, 0x04]
        self.version_loader = [0x01, 0x02, 0x03, 0x04]
        self.version_hardware = [0x01, 0x02, 0x03, 0x04]
        self.mac = list(DEVICE_MAC)
        self.cur_work_state = 0x01
        self.core_temp = 30
        self.powerup_cnt = 10
        self.local_time_now = 3875213548323846324
        self.last_sync_time = 3875213548323846320
        self.time_offset = 4
        self.time_sync_type = 2
        self.status_code = [0x00] * 32
        self.lidar_diag_status = 0x0000
        self.lidar_flash_status = 0x00
        self.fw_type = 0x01
        self.hms_code = [0x00] * 8
        self.ROI_Mode = 0x00

    def print_info(self):
        print("IP:", self.pointcloud_host_ipcfg.dest_ip)
        print("Dest Port:", self.pointcloud_host_ipcfg.dest_port)
        print("Src Port:", self.pointcloud_host_ipcfg.src_port)
        print("suiii")
        print("Dest Port:", self.host_info.dest_port)
        print("Src Port:", self.host_info.src_port)
        pass

    def pack(self):
        return (
            struct.pack(
                'BBBB',
                self.pcl_data_type,
                self.pattern_mode,
                self.dual_emit_en,
                self.point_send_en,
            )
            + self.lidar_ipcfg.pack()
            + self.host_info.pack()
            + self.pointcloud_host_ipcfg.pack()
            + self.imu_host_ipcfg.pack()
            + struct.pack('16sHH', *self.ctl_host_ipcfg)
            + struct.pack('16sHH', *self.log_host_ipcfg)
            + struct.pack('ii', self.vehicle_speed, self.environment_temp)
            + self.install_attitude.pack()
            + struct.pack('I', self.blind_spot_set)
            + struct.pack('B', self.frame_rate)
            + self.fov_cfg0.pack()
            + self.fov_cfg1.pack()
            + struct.pack('B', self.fov_cfg_en)
            + struct.pack('B', self.detect_mode)
            + struct.pack('4B', *self.func_io_cfg)
            + struct.pack('B', self.work_tgt_mode)
            + struct.pack('B', self.glass_heat)
            + struct.pack('B', self.imu_data_en)
            + struct.pack('B', self.fusa_en)
            + struct.pack('16s', self.sn)
            + struct.pack('64s', self.product_info)
            + struct.pack('4B', *self.version_app)
            + struct.pack('4B', *self.version_loader)
            + struct.pack('4B', *self.version_hardware)
            + struct.pack('6B', *self.mac)
            + struct.pack('B', self.cur_work_state)
            + struct.pack('i', self.core_temp)
            + struct.pack('I', self.powerup_cnt)
            + struct.pack('Q', self.local_time_now)
            + struct.pack('Q', self.last_sync_time)
            + struct.pack('q', self.time_offset)
            + struct.pack('B', self.time_sync_type)
            + struct.pack('32B', *self.status_code)
            + struct.pack('H', self.lidar_diag_status)
            + struct.pack('B', self.lidar_flash_status)
            + struct.pack('B', self.fw_type)
            + struct.pack('8I', *self.hms_code)
            + struct.pack('B', self.ROI_Mode)
        )

