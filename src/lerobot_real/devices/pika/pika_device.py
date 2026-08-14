import time
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('pika_device')


def get_serial_ports(vidpid='1a86:7522'):
    """
    搜索所有指定vidpid的串口
    vidpid: 指定设备的VID:PID字符串, 默认值为'1a86:7522'
    返回找到的所有符合的串口号列表
    """
    from serial.tools import list_ports
    ports = list_ports.comports()
    pika_ports = []
    for port in ports:
        if port.vid is not None and port.pid is not None:
            if '{:04x}:{:04x}'.format(port.vid, port.pid) == vidpid:
                pika_ports.append(port.device)
            # else:
            #     print('pidvid:', '{:04x}:{:04x}'.format(port.vid, port.pid))
    return pika_ports

def check_pika_device(port):
    """
    检测串口对应的Pika设备类型
    返回值:
        -1: 无法打开串口
        0: 不是Pika设备
        1: Pika Sense设备
        2: Pika Gripper设备
    """
    import serial
    try:
        ser = serial.Serial(
            port=port,
            baudrate=460800,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        time.sleep(0.5)  # 等待串口稳定
        data = b''
        expired_time = time.monotonic() + 1.0  # 最多等待1秒
        while time.monotonic() < expired_time:
            if ser.in_waiting > 0:
                data += ser.read(ser.in_waiting)
                if len(data) > 200:  # 足够的数据来判断
                    break
            time.sleep(0.05)
        ser.close()
        data_str = data.decode('utf-8', errors='ignore')
        if '"Command"' in data_str or '"AS5047"' in data_str or '"IMU"' in data_str:
            # logger.info('✓ 检测到 Pika Sense 设备: {}'.format(port))
            return 1
        elif '"motor"' in data_str or '"motorstatus"' in data_str:
            # logger.info('✓ 检测到 Pika Gripper 设备: {}'.format(port))
            return 2
        else:
            # logger.info('✗ 未检测到 Pika 设备: {}, 数据长度: {}'.format(port, len(data)))
            return 0
    except:
        pass
    return -1


class PikaDevice(object):
    # _instance = None
    # _pika_sense_port = None
    # _pika_gripper_port = None
    # _lock = threading.Lock()
    PIKA_DEVICE_MAP = {}

    def __init__(self, dev_type=1, **kwargs):
        """
        port: serial port
        dev_type: 1: sense, 2: gripper
        """
        if dev_type not in [1, 2, 3]:
            raise ValueError('不支持dev_type={}'.format(dev_type))
        
        self._dev_type = dev_type
        self._pika_sense_port = kwargs.get('pika_sense_port', None)
        self._pika_gripper_port = kwargs.get('pika_gripper_port', None)

        use_pika_sense = self._dev_type in [1, 3]
        use_pika_gripper = self._dev_type in [2, 3]

        self._pika_sense = None
        self._pika_gripper = None

        if (use_pika_sense and self._pika_sense_port is None) or (use_pika_gripper and self._pika_gripper_port is None):
            pika_ports = get_serial_ports()
            if not pika_ports:
                logger.error('未找到Pika设备, 请检查连接')
                raise ConnectionError('未找到Pika设备, 请检查连接')

            for port in pika_ports:
                device_type = check_pika_device(port)
                if device_type == 1 and use_pika_sense and self._pika_sense_port is None:
                    self._pika_sense_port = port
                    logger.info('✓ 检测到 Pika Sense 设备: {}'.format(port))
                    if not use_pika_gripper:
                        break
                if device_type == 2 and use_pika_gripper and self._pika_gripper_port is None:
                    self._pika_gripper_port = port
                    logger.info('✓ 检测到 Pika Gripper 设备: {}'.format(port))
                    if not use_pika_sense:
                        break
        
            if use_pika_sense and self._pika_sense_port is None:
                logger.error('未找到Pika Sense设备, 请检查连接')
                raise ConnectionError('未找到Pika Sense设备, 请检查连接')

            if use_pika_gripper and self._pika_gripper_port is None:
                logger.error('未找到Pika Gripper设备, 请检查连接')
                raise ConnectionError('未找到Pika Gripper设备, 请检查连接')

        if use_pika_sense:
            print('Pika Sense设备:', self._pika_sense_port)
        if use_pika_gripper:
            print('Pika Gripper 设备:', self._pika_gripper_port)

        self.pika_tracker_device = kwargs.get('pika_tracker_device', None)
    
    # def __new__(cls, *args, **kwargs):
    #     if not cls._instance:
    #         with cls._lock:
    #             if not cls._instance:
    #                 cls._instance = super().__new__(cls)
    #                 cls._instance.init(*args, *kwargs)
    #     return cls._instance
    
    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    def disconnect(self):
        """Disconnect owned devices and remove stale shared-device entries."""
        first_error = None
        for attr_name, port in (
            ("_pika_sense", getattr(self, "_pika_sense_port", None)),
            ("_pika_gripper", getattr(self, "_pika_gripper_port", None)),
        ):
            device = getattr(self, attr_name, None)
            if device is None:
                continue
            try:
                device.disconnect()
            except Exception as exc:
                first_error = first_error or exc
            finally:
                if self.PIKA_DEVICE_MAP.get(port) is device:
                    self.PIKA_DEVICE_MAP.pop(port, None)
                setattr(self, attr_name, None)
        if first_error is not None:
            raise first_error

    @property
    def pika_sense(self):
        if self._dev_type not in [1, 3]:
            return None
        if self._pika_sense is None:
            if self._pika_sense_port in self.PIKA_DEVICE_MAP:
                self._pika_sense = self.PIKA_DEVICE_MAP[self._pika_sense_port]
                return self._pika_sense
            from pika.sense import Sense
            # 初始化Sense对象
            self._pika_sense = Sense(port=self._pika_sense_port)
            # 连接设备
            if not self._pika_sense.connect():
                logger.error('连接Pika Sense设备失败')
                self.disconnect()
                raise ConnectionError('连接Pika Sense设备失败')
            logger.info('Pika Sense设备连接成功')
            self.PIKA_DEVICE_MAP[self._pika_sense_port] = self._pika_sense  # 注册共享

            # 配置Vive Tracker（可选）
            # sense.set_vive_tracker_config(config_path='path/to/config', lh_config='lighthouse_config')

            tracker = self._pika_sense.get_vive_tracker()
            if not tracker:
                logger.error('Vive Tracker初始化失败')
                self.disconnect()
                raise ConnectionError('Vive Tracker初始化失败')
            logger.info('Vive Tracker初始化成功')
            time.sleep(2)

            expired_time = time.monotonic() + 15.0
            if self.pika_tracker_device:
                while time.monotonic() < expired_time:
                    if self._pika_sense.get_pose(self.pika_tracker_device) is not None:
                        break
                    time.sleep(0.5)
                else:
                    tracker_id = self.pika_tracker_device
                    logger.error(f'未检测到指定Vive Tracker设备: {tracker_id}')
                    self.disconnect()
                    raise ConnectionError(
                        f'Configured Vive Tracker {tracker_id!r} did not produce a fresh pose'
                    )
            else:
                devices = []
                while time.monotonic() < expired_time:
                    devices = self._pika_sense.get_tracker_devices() or []
                    tracker_devices = [
                        device for device in devices if not device.startswith('LH')
                    ]
                    if tracker_devices:
                        break
                    time.sleep(0.5)
                if not devices:
                    logger.error('未检测到Vive Tracker设备')
                    self.disconnect()
                    raise ConnectionError('未检测到Vive Tracker设备')
                logger.info('检测到Vive Tracker设备: {}'.format(devices))

                tracker_devices = [
                    device for device in devices if not device.startswith('LH')
                ]
                if not tracker_devices:
                    logger.error(
                        'No Pika tracker found; only lighthouse devices were detected: {}'.format(
                            devices
                        )
                    )
                    self.disconnect()
                    raise ConnectionError(
                        f'No Pika tracker found; only lighthouse devices were detected: {devices}'
                    )
                for device in tracker_devices:
                    if device.startswith('WM'):
                        self.pika_tracker_device = device
                        break
                else:
                    self.pika_tracker_device = tracker_devices[0]
            logger.info('开始跟踪设备: {}\n'.format(self.pika_tracker_device))
        return self._pika_sense
    
    @property
    def pika_gripper(self):
        if self._dev_type not in [2, 3]:
            return None
        if self._pika_gripper is None:
            if self._pika_gripper_port in self.PIKA_DEVICE_MAP:
                self._pika_gripper = self.PIKA_DEVICE_MAP[self._pika_gripper_port]
                return self._pika_gripper
            from pika.gripper import Gripper
            self._pika_gripper = Gripper(port=self._pika_gripper_port)
            self.PIKA_DEVICE_MAP[self._pika_gripper_port] = self._pika_gripper  # 注册共享
            # 连接设备
            if not self._pika_gripper.connect():
                logger.error('连接Pika Gripper设备失败')
                self.disconnect()
                raise ConnectionError('连接Pika Gripper设备失败')
            logger.info('Pika Gripper设备连接成功')
        return self._pika_gripper


if __name__ == '__main__':
    pika_device1 = PikaDevice(1)
    pika_device1.pika_sense
    pika_device1.pika_gripper
    time.sleep(3)

    # input('=================')

    pika_device2 = PikaDevice(2)
    pika_device2.pika_sense
    pika_device2.pika_gripper
    
    input('=================')

    print(pika_device1)
    print(pika_device1.pika_sense)
    print(pika_device1.pika_gripper)

    print(pika_device2)
    print(pika_device2.pika_sense)
    print(pika_device2.pika_gripper)

    input('=================')
