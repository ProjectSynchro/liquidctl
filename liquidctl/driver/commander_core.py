"""liquidctl drivers for Corsair Commander Core.

Supported devices:

- Corsair Commander Core

Copyright ParkerMc and contributors
SPDX-License-Identifier: GPL-3.0-or-later
"""

import logging
from contextlib import contextmanager

from liquidctl.driver.usb import UsbHidDriver
from liquidctl.error import ExpectationNotMet, NotSupportedByDriver, NotSupportedByDevice
from liquidctl.util import clamp, u16le_from

_LOGGER = logging.getLogger(__name__)

# Wire packet length (without the leading HID report ID byte); each variant
# advertises a different HID report size and using the wrong one truncates
# reads and corrupts device state.
_PACKET_LENGTH_CORE_V1 = 1024
_PACKET_LENGTH_CORE_V2 = 96
_PACKET_LENGTH_CORE_XT = 384
_PACKET_LENGTH_CORE_ST = 64

_INTERFACE_NUMBER = 0

_FAN_COUNT = 6

_CMD_WAKE = (0x01, 0x03, 0x00, 0x02)
_CMD_SLEEP = (0x01, 0x03, 0x00, 0x01)
_CMD_GET_FIRMWARE = (0x02, 0x13)
_CMD_CLOSE_ENDPOINT = (0x05, 0x01, 0x00)
_CMD_OPEN_ENDPOINT = (0x0d, 0x00)
_CMD_READ_INITIAL = (0x08, 0x00, 0x01)
_CMD_READ_MORE = (0x08, 0x00, 0x02)
_CMD_READ_FINAL = (0x08, 0x00, 0x03)
_CMD_WRITE = (0x06, 0x00)
_CMD_WRITE_MORE = (0x07, 0x00)

_MODE_LED_COUNT = (0x20,)
_MODE_GET_SPEEDS = (0x17,)
_MODE_GET_TEMPS = (0x21,)
_MODE_CONNECTED_SPEEDS = (0x1a,)
_MODE_HW_SPEED_MODE = (0x60, 0x6d)
_MODE_HW_FIXED_PERCENT = (0x61, 0x6d)
_MODE_HW_CURVE_PERCENT = (0x62, 0x6d)

_DATA_TYPE_SPEEDS = (0x06, 0x00)
_DATA_TYPE_LED_COUNT = (0x0f, 0x00)
_DATA_TYPE_TEMPS = (0x10, 0x00)
_DATA_TYPE_CONNECTED_SPEEDS = (0x09, 0x00)
_DATA_TYPE_HW_SPEED_MODE = (0x03, 0x00)
_DATA_TYPE_HW_FIXED_PERCENT = (0x04, 0x00)
_DATA_TYPE_HW_CURVE_PERCENT = (0x05, 0x00)

_FAN_MODE_FIXED_PERCENT = 0x00
_FAN_MODE_CURVE_PERCENT = 0x02

# RGB port mode values reported by the LED count endpoint.  The 4-pin
# per-fan headers always report CONNECTED or DISCONNECTED; the Commander
# Core XT's 3-pin RGB(C) header always reads back as 0x0000 even when
# iCUE has configured it, so we treat that as N/A rather than guessing.
_RGB_MODE_CONNECTED = 0x02
_RGB_MODE_DISCONNECTED = 0x03

class CommanderCore(UsbHidDriver):
    """Corsair Commander Core"""

    # For a non-exhaustive list of issues, see: #520, #583, #598, #623, #705
    _MATCHES = [
        (0x1b1c, 0x0c1c, 'Corsair Commander Core (broken)',
            {"has_pump": True, "packet_length": _PACKET_LENGTH_CORE_V2}),
        (0x1b1c, 0x0c2a, 'Corsair Commander Core XT (broken)',
            {"has_pump": False, "packet_length": _PACKET_LENGTH_CORE_XT}),
        (0x1b1c, 0x0c32, 'Corsair Commander ST (broken)',
            {"has_pump": True, "packet_length": _PACKET_LENGTH_CORE_ST}),
    ]

    def __init__(self, device, description, has_pump, packet_length=_PACKET_LENGTH_CORE_V2, **kwargs):
        super().__init__(device, description, **kwargs)
        self._has_pump = has_pump
        # may be raised to _PACKET_LENGTH_CORE_V1 in initialize() if this
        # Commander Core turns out to be running firmware v1.x.x
        self._packet_length = packet_length

    def initialize(self, **kwargs):
        """Initialize the device and get the fan modes."""

        # the firmware version query does not require a prior wake, and we
        # need the major version up front to pick the right packet length
        res = self._send_command(_CMD_GET_FIRMWARE)
        fw_version = (res[3], res[4], res[5])

        # the original Commander Core uses 1024-byte HID reports under
        # firmware v1.x.x and 96-byte reports under v2.x.x; the Core XT and
        # Commander ST always use the fixed sizes set at construction time
        if (self.device.product_id == 0x0c1c
                and fw_version[0] == 1
                and self._packet_length != _PACKET_LENGTH_CORE_V1):
            _LOGGER.debug('Commander Core firmware v1 detected, '
                          'switching to %d byte packets',
                          _PACKET_LENGTH_CORE_V1)
            self._packet_length = _PACKET_LENGTH_CORE_V1

        with self._wake_device_context():
            status = [('Firmware version', '{}.{}.{}'.format(*fw_version), '')]

            # Get LEDs per fan; index 0 is the EXT/AIO port on the Core/ST
            # and the 3-pin RGB(C) external strip header on the XT.  Indices
            # 1..6 are the per-fan RGB headers and align with fan numbering.
            res = self._read_data(_MODE_LED_COUNT, _DATA_TYPE_LED_COUNT)
            num_devices = res[0]
            led_data = res[1:1 + num_devices * 4]
            for i in range(0, num_devices):
                mode = u16le_from(led_data, offset=i * 4)
                num_leds = u16le_from(led_data, offset=i * 4 + 2)
                if i == 0:
                    label = 'AIO LED count' if self._has_pump else 'External LED count'
                else:
                    label = f'RGB port {i} LED count'

                status += [(label, num_leds if mode == _RGB_MODE_CONNECTED else None, '')]

            # Get what fans are connected
            res = self._read_data(_MODE_CONNECTED_SPEEDS, _DATA_TYPE_CONNECTED_SPEEDS)
            num_devices = res[0]
            for i in range(0, num_devices):
                if self._has_pump:
                    label = 'AIO port connected' if i == 0 else f'Fan port {i} connected'
                else:
                    label = f'Fan port {i+1} connected'

                status += [(label, res[i + 1] == 0x07, '')]

            # Get what temp sensors are connected
            for i, temp in enumerate(self._get_temps()):
                connected = temp is not None
                if self._has_pump:
                    label = 'Water temperature sensor' if i == 0 and self._has_pump else f'Temperature sensor {i}'
                else:
                    label = f'Temperature sensor {i+1}'

                status += [(label, connected, '')]

        return status

    def get_status(self, **kwargs):
        """Get all the fan speeds and temps"""
        status = []

        with self._wake_device_context():
            for i, speed in enumerate(self._get_speeds()):
                if self._has_pump:
                    label = 'Pump speed' if i == 0 else f'Fan speed {i}'
                else:
                    label = f'Fan speed {i+1}'

                status += [(label, speed, 'rpm')]

            for i, temp in enumerate(self._get_temps()):
                if temp is None:
                    continue

                if self._has_pump:
                    label = 'Water temperature' if i == 0 else f'Temperature {i}'
                else:
                    label = f'Temperature {i}'

                status += [(label, temp, '°C')]

        return status

    def set_color(self, channel, mode, colors, **kwargs):
        raise NotSupportedByDriver

    def set_speed_profile(self, channel, profile, **kwargs):
        channels = self._parse_channels(channel)
        curve_points = list(profile)
        if len(curve_points) < 2:
            ValueError('a minimum of 2 speed curve points must be configured.')
        if len(curve_points) > 7:
            ValueError('a maximum of 7 speed curve points may be configured.')

        with self._wake_device_context():
            # Set hardware speed mode
            res = self._read_data(_MODE_HW_SPEED_MODE, _DATA_TYPE_HW_SPEED_MODE)
            device_count = res[0]

            data = bytearray(res[0:device_count + 1])
            for chan in channels:
                data[chan + 1] = _FAN_MODE_CURVE_PERCENT
            self._write_data(_MODE_HW_SPEED_MODE, _DATA_TYPE_HW_SPEED_MODE, data)


            # Read in data and split by device
            res = self._read_data(_MODE_HW_CURVE_PERCENT, _DATA_TYPE_HW_CURVE_PERCENT)
            device_count = res[0]
            data_by_device = []

            i = 1
            for _ in range(0, device_count):
                count = res[i+1]
                start = i
                end = i + 4 * count + 2
                i = end
                data_by_device.append(res[start:end])

            # Modify data for channels in channels array
            for chan in channels:
                new_data = []

                # set temperature sensor
                new_data.append(b"\x00")

                # set number of curve points
                new_data.append(int.to_bytes(len(curve_points), length=1, byteorder="big"))

                # set curve points
                for (temp, duty) in curve_points:
                    new_data.append(int.to_bytes(temp*10, length=2, byteorder="little", signed=False))
                    new_data.append(int.to_bytes(clamp(duty, 0, 100), length=2, byteorder="little", signed=False))

                # Update device data
                data_by_device[chan] = b''.join(new_data)

            out = bytes([device_count]) + b''.join(data_by_device)
            self._write_data(_MODE_HW_CURVE_PERCENT, _DATA_TYPE_HW_CURVE_PERCENT, out)

    def set_fixed_speed(self, channel, duty, **kwargs):
        channels = self._parse_channels(channel)

        with self._wake_device_context():
            # Set hardware speed mode
            res = self._read_data(_MODE_HW_SPEED_MODE, _DATA_TYPE_HW_SPEED_MODE)
            device_count = res[0]

            data = bytearray(res[0:device_count + 1])
            for chan in channels:
                data[chan + 1] = _FAN_MODE_FIXED_PERCENT
            self._write_data(_MODE_HW_SPEED_MODE, _DATA_TYPE_HW_SPEED_MODE, data)

            # Set speed
            res = self._read_data(_MODE_HW_FIXED_PERCENT, _DATA_TYPE_HW_FIXED_PERCENT)
            device_count = res[0]
            data = bytearray(res[0:device_count * 2 + 1])
            duty_le = int.to_bytes(clamp(duty, 0, 100), length=2, byteorder="little", signed=False)
            for chan in channels:
                i = chan * 2 + 1
                data[i: i + 2] = duty_le  # Update the device speed
            self._write_data(_MODE_HW_FIXED_PERCENT, _DATA_TYPE_HW_FIXED_PERCENT, data)

    @classmethod
    def probe(cls, handle, **kwargs):
        """Ensure we get the right interface"""

        if handle.hidinfo['interface_number'] != _INTERFACE_NUMBER:
            return

        yield from super().probe(handle, **kwargs)

    def _get_speeds(self):
        speeds = []

        res = self._read_data(_MODE_GET_SPEEDS, _DATA_TYPE_SPEEDS)

        num_speeds = res[0]
        speeds_data = res[1:1 + num_speeds * 2]
        for i in range(0, num_speeds):
            speeds.append(u16le_from(speeds_data, offset=i * 2))

        return speeds

    def _get_temps(self):
        temps = []

        res = self._read_data(_MODE_GET_TEMPS, _DATA_TYPE_TEMPS)

        num_temps = res[0]
        temp_data = res[1:1 + num_temps * 3]
        for i in range(0, num_temps):
            connected = temp_data[i * 3] == 0x00
            if connected:
                temps.append(u16le_from(temp_data, offset=i * 3 + 1) / 10)
            else:
                temps.append(None)

        return temps

    # Largest payload we may need to retrieve; bounded by the hardware speed
    # curve table at 7 ports * 30 bytes + 1 count byte = 211 bytes.
    _MAX_PAYLOAD_LEN = 256

    def _read_data(self, mode, data_type):
        self._send_command(_CMD_OPEN_ENDPOINT, mode)
        raw_data = self._send_command(_CMD_READ_INITIAL)

        # Some Commander Core XT firmwares lie about the data type prefix
        # after a HW_SPEED_MODE write: the body is the correct data for the
        # endpoint we just opened, but the data type bytes come back as
        # 0x0000 or even another endpoint's data type.  When the firmware
        # is well-behaved, log a debug message; when it isn't, trust that
        # the response is for the endpoint we just opened (it has to be:
        # we issued an Open Endpoint immediately before this read) and
        # carry on.
        got_data_type = tuple(raw_data[3:5])
        if got_data_type != data_type:
            _LOGGER.debug(
                'unexpected data type for endpoint %s: got %s, expected %s',
                ':'.join(f'{b:02x}' for b in mode),
                ':'.join(f'{b:02x}' for b in got_data_type),
                ':'.join(f'{b:02x}' for b in data_type),
            )

        # only chain Read More / Read Final when a single packet can't hold
        # the largest payload we might ask for; sending them on the XT (or
        # Core firmware v1) leaves the device in an undefined state (#598)
        result = bytearray(raw_data[5:])
        initial_capacity = self._packet_length - 5
        if initial_capacity < self._MAX_PAYLOAD_LEN:
            more_raw_data = self._send_command(_CMD_READ_MORE)
            result.extend(more_raw_data[3:])
            final_raw_data = self._send_command(_CMD_READ_FINAL)
            result.extend(final_raw_data[3:])

        self._send_command(_CMD_CLOSE_ENDPOINT)
        return bytes(result)

    def _send_command(self, command, data=()):
        # self.device.write expects buf[0] to be the report number or 0 if not used
        buf = bytearray(self._packet_length + 1)

        # buf[1] when going out is always 08
        buf[1] = 0x08

        # Indexes for the buffer
        cmd_start = 2
        data_start = cmd_start + len(command)
        data_end = data_start + len(data)

        # Fill in the buffer
        buf[cmd_start:data_start] = command
        buf[data_start:data_end] = data

        self.device.clear_enqueued_reports()
        self.device.write(buf)

        res = self.device.read(self._packet_length)
        while res[0] != 0x00:
            res = self.device.read(self._packet_length)
        buf = bytes(res)
        assert buf[1] == command[0], 'response does not match command'
        return buf

    @contextmanager
    def _wake_device_context(self):
        try:
            self._send_command(_CMD_WAKE)
            yield
        finally:
            self._send_command(_CMD_SLEEP)

    def _write_data(self, mode, data_type, data):
        self._read_data(mode, data_type)  # Will ensure we are writing the correct data type to avoid breakage

        self._send_command(_CMD_OPEN_ENDPOINT, mode)

        # Write data
        data_len = len(data)
        data_start_index = 0
        while (data_start_index < data_len):
            if (data_start_index == 0):
                # First 9 bytes are in use
                packet_data_len = self._packet_length - 9

                if (data_len < packet_data_len):
                    packet_data_len = data_len

                # Num Data Length bytes + 0x05 + 0x06 + Num Data Type bytes + Num Data bytes
                buf = bytearray(2 + 2 + len(data_type) + packet_data_len)

                # Data Length value (includes data type length) - 0x03 and 0x04
                buf[0: 2] = int.to_bytes(data_len + len(data_type), length=2, byteorder="little", signed=False)
                # Data Type value - 0x07 and 0x08
                buf[4: 4 + len(data_type)] = data_type
                # Data - 0x09 onwards
                buf[4 + len(data_type):] = data[0:packet_data_len]

                self._send_command(_CMD_WRITE, buf)
                data_start_index += packet_data_len
            else:
                # First 3 bytes are in use
                packet_data_len = self._packet_length - 3
                if data_len - data_start_index < packet_data_len:
                    packet_data_len = data_len - data_start_index

                self._send_command(_CMD_WRITE_MORE, data[data_start_index:data_start_index + packet_data_len])
                data_start_index += packet_data_len

        self._send_command(_CMD_CLOSE_ENDPOINT)

    def _fan_to_channel(self, fan):
        if self._has_pump:
            return fan
        else:
            # On devices without a pump, channel 0 is fan 1
            return fan - 1

    def _parse_channels(self, channel):
        if self._has_pump and channel == 'pump':
            return [0]
        elif channel == "fans":
            return [self._fan_to_channel(x) for x in range(1, _FAN_COUNT + 1)]
        elif channel.startswith("fan") and channel[3:].isnumeric() and 0 < int(channel[3:]) <= _FAN_COUNT:
            return [self._fan_to_channel(int(channel[3:]))]
        else:
            fan_names = ['fan' + str(i) for i in range(1, _FAN_COUNT + 1)]
            fan_names_part = '", "'.join(fan_names)
            if self._has_pump:
                fan_names.insert(0, "pump")
            raise ValueError(f'unknown channel, should be one of: "{fan_names_part}" or "fans"')

    def set_screen(self, channel, mode, value, **kwargs):
        """Not supported by this device."""
        raise NotSupportedByDevice
