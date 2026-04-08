import pytest
from collections import deque

from liquidctl import ExpectationNotMet
from liquidctl.driver.commander_core import CommanderCore
from _testutils import noop
from liquidctl.util import u16le_from


def int_to_le(num, length=2, byteorder='little', signed=False):
    return int(num).to_bytes(length=length, byteorder=byteorder, signed=signed)


class MockCommanderCoreDevice:
    def __init__(self, product_id=0x0c1c, packet_length=96):
        self.vendor_id = 0x1b1c
        self.product_id = product_id
        self.address = 'addr'
        self.path = b'path'
        self.release_number = None
        self.serial_number = None
        self.bus = None
        self.port = None

        # wire packet length (without the HID report ID); reads/writes
        # mismatching this size are rejected to mimic hidapi behavior
        self._packet_length = packet_length

        self.open = noop
        self.close = noop
        self.clear_enqueued_reports = noop

        self._read = deque()
        self.sent = list()
        # opcodes (data[2]) seen by .write(), in order
        self.command_log = []
        # subcommand byte (data[4]) for every Read command: 0x01 Initial,
        # 0x02 More, 0x03 Final
        self.read_subcommand_log = []

        self._last_write = bytes()
        self._modes = {}
        self._awake = False

        self.response_prefix = ()
        self.firmware_version = (0x00, 0x00, 0x00)
        self.led_counts = (None, None, None, None, None, None, None)
        self.speeds_mode = (0, 0, 0, 0, 0, 0, 0)
        self.speeds = (None, None, None, None, None, None, None)
        self.fixed_speeds = (0, 0, 0, 0, 0, 0, 0)
        self.temperatures = (None, None)
        self.curve_points_by_device = [[],[],[],[],[],[],[]]
        # mapping of (mode_lo, mode_hi) tuples to a (lo, hi) tuple that the
        # Read Initial response should report as the data type prefix.
        # Used to mimic real Commander Core XT firmwares that return the
        # right body with the wrong data type bytes after a HW_SPEED_MODE
        # write.
        self.lie_data_type_for = {}
        # if set, the next .write() call raises this exception.  used to
        # exercise the wake/sleep cleanup paths in the driver.
        self.raise_on_next_write = None

    def read(self, length):
        data = bytearray([0x00, self._last_write[2], 0x00])
        data.extend(self.response_prefix)

        if self._last_write[2] == 0x02:  # Firmware version
            for i in range(0, 3):
                data.append(self.firmware_version[i])
            # mimic real Commander Core hardware: firmware v1.x.x bumps the
            # report size from 96 to 1024 bytes for all subsequent commands
            if (self.product_id == 0x0c1c
                    and self.firmware_version[0] == 1
                    and self._packet_length == 96):
                self._packet_length = 1024
        if self._awake:
            if self._last_write[2] == 0x08 or self._last_write[2] == 0x09:  # Get data
                channel = self._last_write[3]
                mode = self._modes.get(channel)
                if mode[1] == 0x00:
                    if mode[0] == 0x17:  # Get speeds
                        data.extend([0x06, 0x00])
                        data.append(len(self.speeds))
                        for i in self.speeds:
                            if i is None:
                                data.extend([0x00, 0x00])
                            else:
                                data.extend(int_to_le(i))
                    elif mode[0] == 0x1a:  # Speed devices connected
                        data.extend([0x09, 0x00])
                        data.append(len(self.speeds))
                        for i in self.speeds:
                            data.extend([0x01 if i is None else 0x07])
                    elif mode[0] == 0x20:  # LED detect
                        data.extend([0x0f, 0x00])
                        data.append(len(self.led_counts))
                        for i in self.led_counts:
                            if i is None:
                                # disconnected
                                data.extend(int_to_le(3) + int_to_le(0))
                            elif i == 'not_configured':
                                # XT external 3-pin RGB(C), no auto-detect
                                data.extend(int_to_le(0) + int_to_le(0))
                            else:
                                # connected, i is the LED count
                                data.extend(int_to_le(2))
                                data.extend(int_to_le(i))
                    elif mode[0] == 0x21:  # Get temperatures
                        data.extend([0x10, 0x00])
                        data.append(len(self.temperatures))
                        for i in self.temperatures:
                            if i is None:
                                data.append(1)
                                data.extend(int_to_le(0))
                            else:
                                data.append(0)
                                data.extend(int_to_le(int(i*10)))
                    else:
                        raise NotImplementedError(f'Read for {mode.hex(":")}')
                elif mode[1] == 0x6d:
                    if mode[0] == 0x60:
                        data.extend([0x03, 0x00])
                        data.append(len(self.speeds_mode))
                        for i in self.speeds_mode:
                            data.append(i)
                    elif mode[0] == 0x61:
                        data.extend([0x04, 0x00])
                        data.append(len(self.fixed_speeds))
                        for i in self.fixed_speeds:
                            data.extend(int_to_le(i))
                    elif mode[0] == 0x62:
                        data.extend([0x05, 0x00]) # data type
                        num_ports = len(self.curve_points_by_device)
                        data.append(num_ports)
                        for i in range(num_ports):
                            data.append(0) # temp sensor
                            data.append(len(self.curve_points_by_device[i])) # num curve points
                            for (temp, duty) in self.curve_points_by_device[i]:
                                data.extend(int_to_le(temp * 10)) # temperature
                                data.extend(int_to_le(duty)) # duty
                    else:
                        raise NotImplementedError(f'Read for {mode.hex(":")}')
                else:
                    raise NotImplementedError(f'Read for {mode.hex(":")}')

                # mimic the firmware quirk where some endpoints return the
                # right body but with mislabeled data type bytes; the data
                # type bytes live at offsets 3 and 4 of the response
                lie = self.lie_data_type_for.get(tuple(mode))
                if (lie is not None
                        and self._last_write[2] == 0x08
                        and self._last_write[4] == 0x01):
                    data[3], data[4] = lie
            elif self._last_write[2] == 0x09:  # Get more data
                channel = self._last_write[3]
                mode = self._modes.get(channel)
                if mode[1] == 0x6d:
                    if mode[0] == 0x62:
                        for i in range(3, 7):
                            data.append(0) # temp sensor
                            data.append(len(self.curve_points_by_device[i])) # num curve points
                            for (temp, duty) in self.curve_points_by_device[i]:
                                data.extend(int_to_le(temp * 10))
                                data.extend(int_to_le(duty))



        # cap reads at the device's report size, like hidapi does
        return list(data)[:min(length, self._packet_length)]

    def write(self, data):
        data = bytes(data)  # ensure data is convertible to bytes
        # reject mismatched writes to catch packet length bugs in the driver
        if len(data) != self._packet_length + 1:
            raise ValueError(
                f'host->device packet length {len(data)} does not match '
                f'device report size {self._packet_length + 1}')
        if self.raise_on_next_write is not None:
            exc, self.raise_on_next_write = self.raise_on_next_write, None
            raise exc
        self._last_write = data
        if data[0] != 0x00 or data[1] != 0x08:
            raise ValueError('Start of packets going out should be 00:08')
        self.command_log.append(data[2])
        if data[2] == 0x08:
            self.read_subcommand_log.append(data[4])
        if data[2] == 0x0d:
            channel = data[3]
            if  self._modes.get(channel) is None:
                self._modes[channel] = data[4:6]
            else:
                raise ExpectationNotMet('Previous channel was not reset')
        elif data[2] == 0x05 and data[3] == 0x01:
            self._modes[data[4]] = None
        elif data[2] == 0x01 and data[3] == 0x03 and data[4] == 0x00:
            self._awake = data[5] == 0x02
        elif self._awake:
            channel = data[3]
            mode = self._modes.get(channel)

            if data[2] == 0x06:  # Write command
                self.data_length = u16le_from(data[4:6])
                self.written_data_type = data[8:10]
                self.written_data = data[10:8+self.data_length]
                if len(self.written_data) < self.data_length - 2:
                    return
            if data[2] == 0x07: # Write More command
                self.written_data += data[4:]
                if len(self.written_data) < self.data_length - 2:
                    return

            if data[2] == 0x06 or data[2] == 0x07:
                if mode[1] == 0x6d:
                    if mode[0] == 0x60 and list(self.written_data_type) == [0x03, 0x00]:
                        self.speeds_mode = tuple(self.written_data[i+1] for i in range(0, self.written_data[0]))
                    elif mode[0] == 0x61 and list(self.written_data_type) == [0x04, 0x00]:
                        self.fixed_speeds = tuple(u16le_from(self.written_data[i*2+1:i*2+3]) for i in range(0, self.written_data[0]))
                    elif mode[0] == 0x62 and list(self.written_data_type) == [0x05, 0x00]:
                        curve_index = 1
                        for port_index in range(0, self.written_data[0]):
                            self.curve_points_by_device[port_index] = []
                            # get number of curve points
                            num_points = self.written_data[curve_index + 1]

                            # store temperature and duty
                            for i in range(num_points):
                                point_index = curve_index + 2 + i*4
                                cp_temp = u16le_from(self.written_data[point_index: point_index + 2]) / 10
                                cp_duty = u16le_from(self.written_data[point_index + 2: point_index + 4])
                                self.curve_points_by_device[port_index].append((cp_temp, cp_duty))

                            # update index to next curve
                            curve_index += 4 * num_points + 2
                    else:
                        raise NotImplementedError('Invalid Write command')
                else:
                    raise NotImplementedError('Invalid Write command')

        return len(data)



@pytest.fixture
def commander_core_device():
    device = MockCommanderCoreDevice()
    core = CommanderCore(device, 'Corsair Commander Core', True)
    core.connect()
    return core


@pytest.fixture
def commander_core_xt_device():
    device = MockCommanderCoreDevice(product_id=0x0c2a, packet_length=384)
    core = CommanderCore(device, 'Corsair Commander Core XT', False, packet_length=384)
    core.connect()
    return core


@pytest.fixture
def commander_st_device():
    device = MockCommanderCoreDevice(product_id=0x0c32, packet_length=64)
    core = CommanderCore(device, 'Corsair Commander ST', True, packet_length=64)
    core.connect()
    return core


def test_initialize_commander_core(commander_core_device):
    commander_core_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_core_device.device.speeds = (None, 104, None, None, None, None, 918)
    commander_core_device.device.led_counts = (27, None, 1, 2, 4, 8, 16)
    commander_core_device.device.temperatures = (None, 45.6)
    res = commander_core_device.initialize()

    assert len(res) == 17

    assert res[0][1] == '2.6.201'  # Firmware

    # LED counts
    assert res[1][1] == 27
    assert res[2][1] is None
    assert res[3][1] == 1
    assert res[4][1] == 2
    assert res[5][1] == 4
    assert res[6][1] == 8
    assert res[7][1] == 16

    # Speed devices connected
    assert not res[8][1]
    assert res[9][1]
    assert not res[10][1]
    assert not res[11][1]
    assert not res[12][1]
    assert not res[13][1]
    assert res[14][1]

    # Temperature sensors connected
    assert not res[15][1]
    assert res[16][1]

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_initialize_error_commander_core(commander_core_device):
    """An error mid-initialize must still leave the device in hardware mode."""
    # let initialize get past the firmware query and into the wake context,
    # then have the next write fail
    commander_core_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_core_device.device.raise_on_next_write = OSError('synthetic')

    with pytest.raises(OSError):
        commander_core_device.initialize()

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_status_commander_core(commander_core_device):
    commander_core_device.device.speeds = (2357, 918, 903, 501, 1104, 1824, 104)
    commander_core_device.device.temperatures = (12.3, 45.6)
    res = commander_core_device.get_status()

    assert len(res) == 9

    # Speeds of pump and fans
    assert res[0][1] == 2357
    assert res[1][1] == 918
    assert res[2][1] == 903
    assert res[3][1] == 501
    assert res[4][1] == 1104
    assert res[5][1] == 1824
    assert res[6][1] == 104

    # Temperatures
    assert res[7][1] == 12.3
    assert res[8][1] == 45.6

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_status_error_commander_core(commander_core_device):
    """An error mid-status must still leave the device in hardware mode."""
    commander_core_device.device.raise_on_next_write = OSError('synthetic')

    with pytest.raises(OSError):
        commander_core_device.get_status()

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_set_fixed_speed_fan2_commander_core(commander_core_device):
    """This tests setting the speed of a single channel"""
    commander_core_device.device.speeds_mode = (1, 2, 3, 4, 5, 6, 7)
    commander_core_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13, 14)

    commander_core_device.set_fixed_speed('fan2', 95)

    assert commander_core_device.device.speeds_mode == (1, 2, 0, 4, 5, 6, 7)
    assert commander_core_device.device.fixed_speeds == (8, 9, 95, 11, 12, 13, 14)
    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_set_fixed_speed_fans_commander_core(commander_core_device):
    """This tests setting the speed of all the fans"""
    commander_core_device.device.speeds_mode = (1, 2, 3, 4, 5, 6, 7)
    commander_core_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13, 14)

    commander_core_device.set_fixed_speed('fans', 61)

    assert commander_core_device.device.speeds_mode == (1, 0, 0, 0, 0, 0, 0)
    assert commander_core_device.device.fixed_speeds == (8, 61, 61, 61, 61, 61, 61)
    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_set_fixed_speed_error_commander_core(commander_core_device):
    """An error mid-set_fixed_speed must still leave the device in hardware mode."""
    commander_core_device.device.raise_on_next_write = OSError('synthetic')

    with pytest.raises(OSError):
        commander_core_device.set_fixed_speed('fan1', 95)

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_set_speed_profile_fans_commander_core(commander_core_device):
    """This tests setting the speed of all the fans"""
    commander_core_device.device.speeds_mode = (0, 0, 0, 0, 0, 0, 0)
    commander_core_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13, 14)

    commander_core_device.set_speed_profile('fans', [
        (22, 0), (32, 25), (34, 50), (36, 75), (38,85), (40,95), (42, 100)
    ])

    assert commander_core_device.device.speeds_mode == (0, 2, 2, 2, 2, 2, 2)
    assert commander_core_device.device.curve_points_by_device[0] == []
    assert commander_core_device.device.curve_points_by_device[1] == [(22, 0), (32, 25), (34, 50), (36, 75), (38, 85), (40, 95), (42, 100)]
    assert commander_core_device.device.curve_points_by_device[2] == [(22, 0), (32, 25), (34, 50), (36, 75), (38, 85), (40, 95), (42, 100)]
    assert commander_core_device.device.curve_points_by_device[3] == [(22, 0), (32, 25), (34, 50), (36, 75), (38, 85), (40, 95), (42, 100)]
    assert commander_core_device.device.curve_points_by_device[4] == [(22, 0), (32, 25), (34, 50), (36, 75), (38, 85), (40, 95), (42, 100)]
    assert commander_core_device.device.curve_points_by_device[5] == [(22, 0), (32, 25), (34, 50), (36, 75), (38, 85), (40, 95), (42, 100)]
    assert commander_core_device.device.curve_points_by_device[6] == [(22, 0), (32, 25), (34, 50), (36, 75), (38, 85), (40, 95), (42, 100)]

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake

def test_set_speed_curve_profile_single_channel_commander_core(commander_core_device):
    """This tests setting the speed curve profile of a single fan"""
    commander_core_device.device.speeds_mode = (0, 0, 0, 0, 0, 0, 0)
    commander_core_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13, 14)
    commander_core_device.device.curve_points_by_device[0] = []
    commander_core_device.device.curve_points_by_device[1] = []
    commander_core_device.device.curve_points_by_device[2] = []
    commander_core_device.device.curve_points_by_device[3] = []
    commander_core_device.device.curve_points_by_device[4] = []
    commander_core_device.device.curve_points_by_device[5] = []
    commander_core_device.device.curve_points_by_device[6] = []

    commander_core_device.set_speed_profile('pump', [(22, 0), (42, 100)])
    commander_core_device.set_speed_profile('fan1', [(22, 1), (42, 10)])
    commander_core_device.set_speed_profile('fan2', [(22, 2), (42, 20)])
    commander_core_device.set_speed_profile('fan3', [(22, 3), (42, 30)])
    commander_core_device.set_speed_profile('fan4', [(22, 4), (42, 40)])
    commander_core_device.set_speed_profile('fan5', [(22, 5), (42, 50)])
    commander_core_device.set_speed_profile('fan6', [(22, 6), (42, 60)])

    assert commander_core_device.device.speeds_mode == (2, 2, 2, 2, 2, 2, 2)
    assert commander_core_device.device.curve_points_by_device[0] == [(22, 0), (42, 100)]
    assert commander_core_device.device.curve_points_by_device[1] == [(22, 1), (42, 10)]
    assert commander_core_device.device.curve_points_by_device[2] == [(22, 2), (42, 20)]
    assert commander_core_device.device.curve_points_by_device[3] == [(22, 3), (42, 30)]
    assert commander_core_device.device.curve_points_by_device[4] == [(22, 4), (42, 40)]
    assert commander_core_device.device.curve_points_by_device[5] == [(22, 5), (42, 50)]
    assert commander_core_device.device.curve_points_by_device[6] == [(22, 6), (42, 60)]

    # Ensure device is asleep at end
    assert not commander_core_device.device._awake


def test_parse_channels_commander_core():
    """This test will go through and thoroughly test CommanderCore._parse_channels so we don't have to in other tests"""
    core = CommanderCore(MockCommanderCoreDevice(), 'Corsair Commander Core', True)
    tests = [
        ('pump', [0]), ('fans', [1, 2, 3, 4, 5, 6]),
        ('fan1', [1]), ('fan2', [2]), ('fan3', [3]), ('fan4', [4]), ('fan5', [5]), ('fan6', [6])
    ]

    for (val, answer) in tests:
        assert list(core._parse_channels(val)) == answer



def test_parse_channels_commander_core_xt():
    """Test Core XT-specific channel layout"""
    corext = CommanderCore(MockCommanderCoreDevice(), 'Corsair Commander Core', False)
    tests = [
        ('fans', [0, 1, 2, 3, 4, 5]),
        ('fan1', [0]), ('fan2', [1]), ('fan3', [2]), ('fan4', [3]), ('fan5', [4]), ('fan6', [5])
    ]

    for (val, answer) in tests:
        assert list(corext._parse_channels(val)) == answer


def test_parse_channels_error_commander_core():
    """This tests to make sure we get an error with an invalid channel"""
    core = CommanderCore(MockCommanderCoreDevice(), 'Corsair Commander Core', True)
    with pytest.raises(ValueError):
        core._parse_channels('fan')


# Commander Core XT (PID 0x0c2a) tests.  The Core XT advertises 384-byte HID
# reports; using the wrong size locked it up and required an iCUE reset
# (#520, #583, #598, #623, #705).


def test_initialize_commander_core_xt(commander_core_xt_device):
    # real Core XT hardware returns 7 LED ports (the external 3-pin RGB(C)
    # strip header at index 0 plus 6 per-fan RGB headers) but only 6 speed
    # ports; channel 0 reports mode 0x0000 even after iCUE has set the LED
    # count, so the driver surfaces it as N/A like an empty per-fan header
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds = (733, 709, 696, None, None, None)
    commander_core_xt_device.device.led_counts = ('not_configured', 8, 8, 8, None, None, None)
    commander_core_xt_device.device.temperatures = (None, 32.1)
    res = commander_core_xt_device.initialize()

    # 1 firmware + 7 LED counts + 6 fan ports + 2 temp sensors
    assert len(res) == 16
    assert res[0][1] == '1.4.62'

    assert res[1][0] == 'External LED count'
    assert res[1][1] is None
    assert res[2][0] == 'RGB port 1 LED count'
    assert res[2][1] == 8
    assert res[3][0] == 'RGB port 2 LED count'
    assert res[3][1] == 8
    assert res[4][0] == 'RGB port 3 LED count'
    assert res[4][1] == 8
    assert res[5][0] == 'RGB port 4 LED count'
    assert res[5][1] is None
    assert res[6][0] == 'RGB port 5 LED count'
    assert res[7][0] == 'RGB port 6 LED count'

    assert res[8][0] == 'Fan port 1 connected'
    assert res[8][1]
    assert res[9][1]
    assert res[10][1]
    assert not res[11][1]
    assert not res[12][1]
    assert not res[13][1]

    assert res[14][0] == 'Temperature sensor 1'
    assert not res[14][1]
    assert res[15][0] == 'Temperature sensor 2'
    assert res[15][1]

    assert not commander_core_xt_device.device._awake


def test_initialize_commander_core_xt_independent_rgb_and_fan_headers(commander_core_xt_device):
    # the 6 PWM headers and 6 Corsair RGB headers on the XT are physically
    # independent connectors, so any combination is valid; here all 6 RGB
    # rings are populated but only 3 fans report a tach signal
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds = (None, None, None, 706, 713, 716)
    commander_core_xt_device.device.led_counts = ('not_configured', 8, 8, 8, 8, 8, 8)
    commander_core_xt_device.device.temperatures = (None, None)
    res = commander_core_xt_device.initialize()

    assert res[1][0] == 'External LED count'
    assert res[1][1] is None
    for rgb_idx in range(2, 8):
        assert res[rgb_idx][1] == 8, (
            f'expected 8 LEDs on {res[rgb_idx][0]}, got {res[rgb_idx][1]}'
        )

    assert not res[8][1] and not res[9][1] and not res[10][1]
    assert res[11][1] and res[12][1] and res[13][1]


def test_initialize_commander_core_aio_uses_connected_mode(commander_core_device):
    # the Capellix pump auto-identifies and reports a real LED count;
    # the AIO LED count must come through as an integer
    commander_core_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_core_device.device.led_counts = (29, 8, 8, None, None, None, None)
    commander_core_device.device.speeds = (2300, 800, 800, None, None, None, None)
    commander_core_device.device.temperatures = (35.8, None)
    res = commander_core_device.initialize()

    assert res[1][0] == 'AIO LED count'
    assert res[1][1] == 29


def test_commander_core_xt_skips_continuation_reads(commander_core_xt_device):
    # the XT fits any payload in a single 384-byte report; sending Read More
    # or Read Final on the XT leaves it in an undefined state
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds = (920, 904, None, None, None, None)
    commander_core_xt_device.device.led_counts = (8, 8, None, None, None, None)
    commander_core_xt_device.device.temperatures = (None, 32.1)
    commander_core_xt_device.initialize()

    subcommands = commander_core_xt_device.device.read_subcommand_log
    assert subcommands, 'driver did not issue any read commands'
    assert all(sub == 0x01 for sub in subcommands), (
        f'unexpected continuation read on Commander Core XT: {subcommands}'
    )


def test_commander_core_v2_still_uses_continuation_reads(commander_core_device):
    # the 96-byte v2 path still needs Read More / Read Final for the curve
    # table; pin that to avoid accidentally regressing the original Core
    commander_core_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_core_device.device.speeds = (None, 104, None, None, None, None, 918)
    commander_core_device.device.led_counts = (27, None, 1, 2, 4, 8, 16)
    commander_core_device.device.temperatures = (None, 45.6)
    commander_core_device.initialize()

    subcommands = commander_core_device.device.read_subcommand_log
    assert 0x01 in subcommands
    assert 0x02 in subcommands
    assert 0x03 in subcommands


def test_commander_core_xt_uses_correct_packet_length(commander_core_xt_device):
    # the mock raises ValueError if any write uses the wrong packet length
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds = (920, 904, None, None, None, None)
    commander_core_xt_device.device.led_counts = (8, 8, None, None, None, None)
    commander_core_xt_device.device.temperatures = (None, 32.1)
    commander_core_xt_device.initialize()


def test_set_fixed_speed_commander_core_xt(commander_core_xt_device):
    # the XT has no AIO/pump channel, so fan1 maps to controller channel 0
    commander_core_xt_device.device.speeds_mode = (1, 2, 3, 4, 5, 6)
    commander_core_xt_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13)
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)

    commander_core_xt_device.set_fixed_speed('fan1', 73)

    assert commander_core_xt_device.device.speeds_mode == (0, 2, 3, 4, 5, 6)
    assert commander_core_xt_device.device.fixed_speeds == (73, 9, 10, 11, 12, 13)
    assert not commander_core_xt_device.device._awake


def test_set_fixed_speed_fans_commander_core_xt(commander_core_xt_device):
    commander_core_xt_device.device.speeds_mode = (1, 2, 3, 4, 5, 6)
    commander_core_xt_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13)
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)

    commander_core_xt_device.set_fixed_speed('fans', 50)

    assert commander_core_xt_device.device.speeds_mode == (0, 0, 0, 0, 0, 0)
    assert commander_core_xt_device.device.fixed_speeds == (50, 50, 50, 50, 50, 50)
    assert not commander_core_xt_device.device._awake


# Commander ST (PID 0x0c32) tests.  The ST uses 64-byte HID reports;
# the old hardcoded 96-byte reads overran the response and raised
# IndexError (#705).


def test_initialize_commander_st(commander_st_device):
    commander_st_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_st_device.device.speeds = (2300, 800, 800, None, None, None, None)
    commander_st_device.device.led_counts = (29, 8, 8, None, None, None, None)
    commander_st_device.device.temperatures = (35.8, None)
    res = commander_st_device.initialize()

    assert len(res) == 17
    assert res[0][1] == '2.6.201'

    assert res[1][1] == 29       # AIO LED count
    assert res[8][1]              # AIO connected
    assert res[15][1]             # Water temperature sensor connected
    assert not res[16][1]         # probe sensor disconnected

    assert not commander_st_device.device._awake


def test_set_fixed_speed_commander_st(commander_st_device):
    # regression for #705: 96/64-byte packet mismatch raised IndexError
    commander_st_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_st_device.device.speeds_mode = (1, 2, 3, 4, 5, 6, 7)
    commander_st_device.device.fixed_speeds = (8, 9, 10, 11, 12, 13, 14)

    commander_st_device.set_fixed_speed('fan2', 95)

    assert commander_st_device.device.speeds_mode == (1, 2, 0, 4, 5, 6, 7)
    assert commander_st_device.device.fixed_speeds == (8, 9, 95, 11, 12, 13, 14)
    assert not commander_st_device.device._awake


def test_commander_st_uses_correct_packet_length(commander_st_device):
    # the mock raises ValueError if any write uses the wrong packet length
    commander_st_device.device.firmware_version = (0x02, 0x06, 0xc9)
    commander_st_device.device.speeds = (2300, 800, 800, None, None, None, None)
    commander_st_device.device.led_counts = (29, 8, 8, None, None, None, None)
    commander_st_device.device.temperatures = (35.8, None)
    commander_st_device.initialize()


# Commander Core firmware v1 detection.  Firmware v1.x.x uses 1024-byte HID
# reports instead of the 96-byte reports used by v2.x.x; the driver detects
# this on the fly during initialize() and the mock auto-bumps its packet
# length to mirror real hardware behavior.


def test_initialize_commander_core_v1_firmware(commander_core_device):
    # the full v1 path: firmware query goes out at 96 bytes, the response
    # signals v1, the driver flips to 1024 bytes for everything that follows
    commander_core_device.device.firmware_version = (0x01, 0x02, 0x21)
    commander_core_device.device.speeds = (None, 104, None, None, None, None, 918)
    commander_core_device.device.led_counts = (27, None, 1, 2, 4, 8, 16)
    commander_core_device.device.temperatures = (None, 45.6)
    res = commander_core_device.initialize()

    assert res[0][1] == '1.2.33'
    assert commander_core_device._packet_length == 1024
    assert commander_core_device.device._packet_length == 1024
    assert not commander_core_device.device._awake


def test_commander_core_v1_skips_continuation_reads(commander_core_device):
    # firmware v1's 1024-byte reports always fit any payload in a single
    # response, so the driver should not issue Read More or Read Final
    commander_core_device.device.firmware_version = (0x01, 0x02, 0x21)
    commander_core_device.device.speeds = (None, 104, None, None, None, None, 918)
    commander_core_device.device.led_counts = (27, None, 1, 2, 4, 8, 16)
    commander_core_device.device.temperatures = (None, 45.6)
    commander_core_device.initialize()

    subcommands = commander_core_device.device.read_subcommand_log
    assert subcommands, 'driver did not issue any read commands'
    assert all(sub == 0x01 for sub in subcommands), (
        f'unexpected continuation read on Commander Core firmware v1: {subcommands}'
    )


def test_commander_core_xt_does_not_switch_to_v1_packets(commander_core_xt_device):
    # only the original Commander Core (PID 0x0c1c) carries the v1/v2
    # distinction; an XT firmware whose major happens to be 1 must stay
    # at the 384-byte packet length
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds = (920, 904, None, None, None, None)
    commander_core_xt_device.device.led_counts = (8, 8, None, None, None, None)
    commander_core_xt_device.device.temperatures = (None, 32.1)
    commander_core_xt_device.initialize()

    assert commander_core_xt_device._packet_length == 384


def test_read_data_accepts_lying_data_type_zero(commander_core_xt_device):
    # some Commander Core XT firmwares return a zeroed data type prefix on
    # HW_FIXED_PERCENT after iCUE has touched the endpoint; the body of
    # the response is correct so the driver must accept it rather than
    # raise ExpectationNotMet
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds_mode = (0, 0, 0, 0, 0, 0)
    commander_core_xt_device.device.fixed_speeds = (40, 40, 40, 40, 40, 40)
    commander_core_xt_device.device.lie_data_type_for = {(0x61, 0x6d): (0x00, 0x00)}

    commander_core_xt_device.set_fixed_speed('fan4', 75)

    assert commander_core_xt_device.device.fixed_speeds == (40, 40, 40, 75, 40, 40)
    assert not commander_core_xt_device.device._awake


def test_read_data_accepts_lying_data_type_other_endpoint(commander_core_xt_device):
    # observed in the wild on a Core XT: after writing HW_SPEED_MODE the
    # next read of HW_FIXED_PERCENT returns the correct body with the
    # HW_CURVE_PERCENT data type prefix (0x05:0x00 instead of 0x04:0x00).
    # the driver must trust the OPEN endpoint and accept the response
    commander_core_xt_device.device.firmware_version = (0x01, 0x04, 0x3e)
    commander_core_xt_device.device.speeds_mode = (0, 0, 0, 0, 0, 0)
    commander_core_xt_device.device.fixed_speeds = (40, 40, 40, 40, 40, 40)
    commander_core_xt_device.device.lie_data_type_for = {(0x61, 0x6d): (0x05, 0x00)}

    commander_core_xt_device.set_fixed_speed('fan4', 75)

    assert commander_core_xt_device.device.fixed_speeds == (40, 40, 40, 75, 40, 40)
    assert not commander_core_xt_device.device._awake
