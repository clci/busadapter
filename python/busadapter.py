

import sys, time
from struct import pack, unpack

import serial

CMD_VERSION = 0
CMD_INIT = 1
CMD_GET_BUS_TYPE = 2
CMD_WRITE = 3
CMD_READ = 4
CMD_TRANSACTION = 5
CMD_SET_PIN_MODE = 7
CMD_DIGITAL_WRITE = 8
CMD_DEBUG1 = 51
CMD_DEBUG2 = 52
CMD_DEBUG3 = 53

MODE_I2C_MASTER = 1

RESPONSE_OK = 0
RESPONSE_ERROR = 1
RESPONSE_CMD_UNAVAILABLE = 2
RESPONSE_WRONG_COMMAND = 3
RESPONSE_BAD_PARAMETERS = 4
RESPONSE_SHORT_WRITE = 5
RESPONSE_NACK_ON_ADDRESS = 6
RESPONSE_NACK_ON_DATA = 7
RESPONSE_DATA_TOO_LONG = 8
RESPONSE_OTHER_ERROR = 9

PIN_MODE_DIGITAL_OUT = 0x01
PIN_STATUS_HIGH = 0x10
PIN_STATUS_LOW = 0x11
PIN_STATUS_TOGGLE = 0x12

# --------------------------------------------------------------------------

class Timeout(Exception):
    pass

class BusError(Exception):

    def __init__(self, msg):

        if isinstance(msg, int):
            msg = {
                RESPONSE_SHORT_WRITE: 'short write',
                RESPONSE_NACK_ON_ADDRESS: 'nack on address',
                RESPONSE_NACK_ON_DATA: 'nack on data',
                RESPONSE_DATA_TOO_LONG: 'data too long for transmit buffer',
                RESPONSE_OTHER_ERROR: 'other error'
            }[msg]

        super().__init__(msg)


class ProtocolError(Exception):
    pass


class BusAdapter:

    def __init__(self, port):

        self.sp = serial.Serial(port, baudrate=115200, timeout=10.0)
        self.sp.setDTR(False)
        self.sp.setRTS(False)
        time.sleep(0.6)


    def close(self):

        self.sp.close()
        self.sp = None


    def _send(self, cmd_code, data=b''):
        self.sp.write(pack('BB', len(data) + 1, cmd_code) + data)


    def _recv(self):

        response_size = self.sp.read(1)
        if not response_size:
            raise Timeout

        bytes_to_read = response_size[0]
        response = b''
        while bytes_to_read:
            block = self.sp.read(bytes_to_read)
            if not block:
                raise BusError(f'short response; got {len(response)} bytes, expected {response_size[0]}')
            response += block
            bytes_to_read -= len(block)

        return response[0], response[1:]


    def get_version(self):

        self._send(CMD_VERSION)
        resp_code, response = self._recv()
        assert resp_code == RESPONSE_OK, resp_code
        return response.rstrip(b'\0').decode('ascii')


    def debug1(self):

        rs = self._send(CMD_DEBUG1)
        resp_code, response = self._recv()
        assert resp_code == RESPONSE_OK, resp_code
        return unpack('<hhhh', response)


    def set_pin_mode(self, pin_number, mode):

        try:
            mode = {
                'digital_out': PIN_MODE_DIGITAL_OUT,
                PIN_MODE_DIGITAL_OUT: PIN_MODE_DIGITAL_OUT
            }[mode]
        except KeyError:
            raise ValueError('wrong pin mode {!r}'.format(mode))

        rs = self._send(CMD_SET_PIN_MODE, pack('BB', pin_number, mode))
        resp_code, response = self._recv()
        assert resp_code == RESPONSE_OK, resp_code


    def digital_write(self, pin_number, status):

        try:
            status = {
                'high': PIN_STATUS_HIGH,
                'low': PIN_STATUS_LOW,
                True: PIN_STATUS_HIGH,
                False: PIN_STATUS_LOW,
                PIN_STATUS_HIGH: PIN_STATUS_HIGH,
                PIN_STATUS_LOW: PIN_STATUS_LOW,
                'toggle': PIN_STATUS_TOGGLE
            }[status]
        except KeyError:
            raise ValueError('wrong pin status {!r}'.format(status))

        rs = self._send(CMD_DIGITAL_WRITE, pack('BB', pin_number, status))
        resp_code, response = self._recv()
        assert resp_code == RESPONSE_OK, resp_code


class BusAdapterI2CMaster(BusAdapter):

    def __init__(self, port='/dev/ttyACM0'):

        super().__init__(port)
        self._send(CMD_INIT, pack('<BBH', MODE_I2C_MASTER, 0, 400))
        msg_code, payload = self._recv()
        if msg_code != RESPONSE_OK and payload != b'':
            raise ProtocolError


    def write(self, address, payload):

        self._send(CMD_WRITE, pack('<B', address) + payload)
        msg_code, response = self._recv()
        if len(response) != 0 or msg_code == RESPONSE_BAD_PARAMETERS:
            raise ProtocolError

        if msg_code != RESPONSE_OK:
            raise BusError(msg_code)


    def read(self, address, read_size):

        self._send(CMD_READ, pack('<BB', address, read_size))
        msg_code, response = self._recv()
        if msg_code == RESPONSE_BAD_PARAMETERS:
            raise ProtocolError
        elif msg_code != RESPONSE_OK:
            raise BusError(msg_code)

        if len(response) != read_size:
            raise BusError('short read')

        return response


    def transaction(self, address, payload, read_size):

        if len(payload) < 1:
            raise ValueError('at least one byte of payload is required')

        self._send(CMD_TRANSACTION, pack('<BB', address, read_size) + bytes(payload))
        msg_code, response = self._recv()
        if msg_code == RESPONSE_BAD_PARAMETERS:
            raise ProtocolError
        elif msg_code != RESPONSE_OK:
            raise BusError(msg_code)

        if len(response) != read_size:
            raise BusError(f'short read; got {len(response)} bytes, expected {read_size}')

        return response


if __name__ == '__main__':

    bus = BusAdapterI2CMaster()
    print(':::', bus.get_version())
    print(':::', bus.debug1())
    print(':::', bus.set_pin_mode(3, 'digital_out'))
    print(':::', bus.digital_write(3, True))


