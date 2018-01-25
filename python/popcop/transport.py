#
# The MIT License (MIT)
#
# Copyright (c) 2017-2018 Zubax Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import typing
import time


__all__ = ['FRAME_DELIMITER', 'ESCAPE_CHARACTER', 'Parser', 'encode']


FRAME_DELIMITER  = 0x8E
ESCAPE_CHARACTER = 0x9E


class CRCComputer:
    """
    Implementation of the CRC-32C (Castagnoli) algorithm.
    """

    CRC32C_TABLE = [
        0x00000000, 0xF26B8303, 0xE13B70F7, 0x1350F3F4, 0xC79A971F, 0x35F1141C, 0x26A1E7E8, 0xD4CA64EB, 0x8AD958CF,
        0x78B2DBCC, 0x6BE22838, 0x9989AB3B, 0x4D43CFD0, 0xBF284CD3, 0xAC78BF27, 0x5E133C24, 0x105EC76F, 0xE235446C,
        0xF165B798, 0x030E349B, 0xD7C45070, 0x25AFD373, 0x36FF2087, 0xC494A384, 0x9A879FA0, 0x68EC1CA3, 0x7BBCEF57,
        0x89D76C54, 0x5D1D08BF, 0xAF768BBC, 0xBC267848, 0x4E4DFB4B, 0x20BD8EDE, 0xD2D60DDD, 0xC186FE29, 0x33ED7D2A,
        0xE72719C1, 0x154C9AC2, 0x061C6936, 0xF477EA35, 0xAA64D611, 0x580F5512, 0x4B5FA6E6, 0xB93425E5, 0x6DFE410E,
        0x9F95C20D, 0x8CC531F9, 0x7EAEB2FA, 0x30E349B1, 0xC288CAB2, 0xD1D83946, 0x23B3BA45, 0xF779DEAE, 0x05125DAD,
        0x1642AE59, 0xE4292D5A, 0xBA3A117E, 0x4851927D, 0x5B016189, 0xA96AE28A, 0x7DA08661, 0x8FCB0562, 0x9C9BF696,
        0x6EF07595, 0x417B1DBC, 0xB3109EBF, 0xA0406D4B, 0x522BEE48, 0x86E18AA3, 0x748A09A0, 0x67DAFA54, 0x95B17957,
        0xCBA24573, 0x39C9C670, 0x2A993584, 0xD8F2B687, 0x0C38D26C, 0xFE53516F, 0xED03A29B, 0x1F682198, 0x5125DAD3,
        0xA34E59D0, 0xB01EAA24, 0x42752927, 0x96BF4DCC, 0x64D4CECF, 0x77843D3B, 0x85EFBE38, 0xDBFC821C, 0x2997011F,
        0x3AC7F2EB, 0xC8AC71E8, 0x1C661503, 0xEE0D9600, 0xFD5D65F4, 0x0F36E6F7, 0x61C69362, 0x93AD1061, 0x80FDE395,
        0x72966096, 0xA65C047D, 0x5437877E, 0x4767748A, 0xB50CF789, 0xEB1FCBAD, 0x197448AE, 0x0A24BB5A, 0xF84F3859,
        0x2C855CB2, 0xDEEEDFB1, 0xCDBE2C45, 0x3FD5AF46, 0x7198540D, 0x83F3D70E, 0x90A324FA, 0x62C8A7F9, 0xB602C312,
        0x44694011, 0x5739B3E5, 0xA55230E6, 0xFB410CC2, 0x092A8FC1, 0x1A7A7C35, 0xE811FF36, 0x3CDB9BDD, 0xCEB018DE,
        0xDDE0EB2A, 0x2F8B6829, 0x82F63B78, 0x709DB87B, 0x63CD4B8F, 0x91A6C88C, 0x456CAC67, 0xB7072F64, 0xA457DC90,
        0x563C5F93, 0x082F63B7, 0xFA44E0B4, 0xE9141340, 0x1B7F9043, 0xCFB5F4A8, 0x3DDE77AB, 0x2E8E845F, 0xDCE5075C,
        0x92A8FC17, 0x60C37F14, 0x73938CE0, 0x81F80FE3, 0x55326B08, 0xA759E80B, 0xB4091BFF, 0x466298FC, 0x1871A4D8,
        0xEA1A27DB, 0xF94AD42F, 0x0B21572C, 0xDFEB33C7, 0x2D80B0C4, 0x3ED04330, 0xCCBBC033, 0xA24BB5A6, 0x502036A5,
        0x4370C551, 0xB11B4652, 0x65D122B9, 0x97BAA1BA, 0x84EA524E, 0x7681D14D, 0x2892ED69, 0xDAF96E6A, 0xC9A99D9E,
        0x3BC21E9D, 0xEF087A76, 0x1D63F975, 0x0E330A81, 0xFC588982, 0xB21572C9, 0x407EF1CA, 0x532E023E, 0xA145813D,
        0x758FE5D6, 0x87E466D5, 0x94B49521, 0x66DF1622, 0x38CC2A06, 0xCAA7A905, 0xD9F75AF1, 0x2B9CD9F2, 0xFF56BD19,
        0x0D3D3E1A, 0x1E6DCDEE, 0xEC064EED, 0xC38D26C4, 0x31E6A5C7, 0x22B65633, 0xD0DDD530, 0x0417B1DB, 0xF67C32D8,
        0xE52CC12C, 0x1747422F, 0x49547E0B, 0xBB3FFD08, 0xA86F0EFC, 0x5A048DFF, 0x8ECEE914, 0x7CA56A17, 0x6FF599E3,
        0x9D9E1AE0, 0xD3D3E1AB, 0x21B862A8, 0x32E8915C, 0xC083125F, 0x144976B4, 0xE622F5B7, 0xF5720643, 0x07198540,
        0x590AB964, 0xAB613A67, 0xB831C993, 0x4A5A4A90, 0x9E902E7B, 0x6CFBAD78, 0x7FAB5E8C, 0x8DC0DD8F, 0xE330A81A,
        0x115B2B19, 0x020BD8ED, 0xF0605BEE, 0x24AA3F05, 0xD6C1BC06, 0xC5914FF2, 0x37FACCF1, 0x69E9F0D5, 0x9B8273D6,
        0x88D28022, 0x7AB90321, 0xAE7367CA, 0x5C18E4C9, 0x4F48173D, 0xBD23943E, 0xF36E6F75, 0x0105EC76, 0x12551F82,
        0xE03E9C81, 0x34F4F86A, 0xC69F7B69, 0xD5CF889D, 0x27A40B9E, 0x79B737BA, 0x8BDCB4B9, 0x988C474D, 0x6AE7C44E,
        0xBE2DA0A5, 0x4C4623A6, 0x5F16D052, 0xAD7D5351,
    ]

    def __init__(self):
        assert len(self.CRC32C_TABLE) == 256
        self._value = 0xFFFFFFFF

    def add(self, b: typing.Union[int, bytes, bytearray]):
        if isinstance(b, int):
            if 0 <= b <= 0xFF:
                self._value = self.CRC32C_TABLE[b ^ (self._value & 0xFF)] ^ (self._value >> 8)
            else:
                raise ValueError('Invalid value for byte: %r' % b)
        elif isinstance(b, (bytes, bytearray)):
            for x in b:
                self.add(x)
        else:
            raise TypeError('Cannot compute CRC of %r' % type(b))

        return self

    @property
    def value(self):
        assert 0 <= self._value <= 0xFFFFFFFF
        return self._value ^ 0xFFFFFFFF

    def is_residue_correct(self):
        assert 0 <= self._value <= 0xFFFFFFFF
        return self._value == 0xB798B438


class ReceivedFrame:
    """
    Representation of a frame received from the comm channel.
    """
    def __init__(self, frame_type_code: int, payload: bytes, timestamp):
        self.frame_type_code = frame_type_code
        self.payload = payload
        self.timestamp = timestamp

    def __str__(self):
        return '0x%02x:%s' % (self.frame_type_code, self.payload.hex())

    __repr__ = __str__


class Parser:
    """
    Protocol parser. Accepts arbitrarily sized chunks of data read from the channel;
    reports parsed blocks of payload via callback.
    """

    PAYLOAD_OVERHEAD_NOT_INCLUDING_DELIMITERS = 5

    def __init__(self,
                 callback: typing.Optional[typing.Callable]=None,
                 max_payload_size: typing.Optional[int]=None,
                 frame_timeout: typing.Optional[float]=None):
        """
        :param callback:            The callback that will be invoked upon reception of a frame.
                                    It is always invoked with one argument; the type of the argument can be either:
                                     - ReceivedFrame - for received frames.
                                     - bytes - for extraneous data.
        :param max_payload_size:    Maximum permissible payload size. Frames that are longer will be considered invalid.
                                    This value cannot be less than 1024 bytes. By default, max payload size is
                                    (virtually) unlimited.
                                    Note that either or both max_payload_size or frame_timeout must be set;
                                    if none are set, the constructor will throw ValueError.
        :param frame_timeout:       Amount of time in seconds in which reception of a frame must be completed.
                                    If the amount of time that passed since beginning of the frame (i.e. since the
                                    moment when the last frame delimiter was received) exceeds this value,
                                    the parser will abort the current frame and report it to the application as
                                    extraneous data. By default an infinite timeout is used (i.e. time is unrestricted).
                                    Note that either or both max_payload_size or frame_timeout must be set;
                                    if none are set, the constructor will throw ValueError.
        """
        if (max_payload_size is None) and (frame_timeout is None):
            raise ValueError('Neither max_payload_size nor frame_timeout are set. This is unsafe. '
                             'Please configure either or both of them.')

        self._callback = callback or (lambda *_: None)
        self._unescape_next = False
        self._buffer = bytearray()
        self._max_payload_size = int(max_payload_size or 1024 * 1024 * 1024)
        self._current_frame_timestamp = None
        self._frame_timeout = float(frame_timeout or 10**10)

        if self._max_payload_size < 1024:
            raise ValueError('Max payload size is too small: %r' % self._max_payload_size)

    def _check_if_received_frame_valid(self):
        long_enough = len(self._buffer) >= self.PAYLOAD_OVERHEAD_NOT_INCLUDING_DELIMITERS
        # Note that we compute CRC in one go over the entire dataset.
        # It is not a good idea in real-time systems because of non-uniform distribution of
        # computational load, but it's Python, it's non-real-time by design, so it's acceptable.
        # Besides, Python-capable systems typically make heavy use of memory caching, and cache is
        # more efficient when data processing is done in large batches.
        return long_enough and CRCComputer().add(self._buffer).is_residue_correct()

    @property
    def _inside_frame(self):
        return self._current_frame_timestamp is not None

    def _finalize(self, known_invalid=False):
        try:
            if (not known_invalid) and self._check_if_received_frame_valid():
                frame_type_code = int(self._buffer[-self.PAYLOAD_OVERHEAD_NOT_INCLUDING_DELIMITERS])
                payload = bytes(self._buffer[:-self.PAYLOAD_OVERHEAD_NOT_INCLUDING_DELIMITERS])
                self._callback(ReceivedFrame(frame_type_code, payload, self._current_frame_timestamp))
            elif len(self._buffer) > 0:
                self._callback(bytes(self._buffer))
        finally:
            self._unescape_next = False
            self._buffer = bytearray()
            self._current_frame_timestamp = None

    def _parse_byte(self, b: int, timestamp):
        # Reception of a frame delimiter UNCONDITIONALLY terminates the current frame
        if b == FRAME_DELIMITER:
            self._finalize(known_invalid=not self._inside_frame)
            self._current_frame_timestamp = timestamp
            return

        # Unescaping is done ONLY if we're inside a frame currently
        if self._inside_frame:
            if b == ESCAPE_CHARACTER:
                self._unescape_next = True
                return

            if self._unescape_next:
                self._unescape_next = False
                b ^= 0xFF

        # Appending to the buffer always, regardless of whether we're in a frame or not.
        # We may find out that the data does not belong to the protocol only much later; can't look ahead.
        self._buffer.append(b)

    def parse(self, chunk: typing.Union[bytes, bytearray], timestamp=None):
        """
        Invoke this method to process an arbitrary chunk of bytes received from the communication channel.
        :param chunk:       Bytes of data from the channel.
                            Can be an empty bytes() object - in this case the method will simply check frame
                            timeouts and return immediately.
        :param timestamp:   An optional object that contains the time when the data bytes were received from the
                            channel. This value is then passed into the callback with each received frame or
                            extraneous data. Also it is used to detect frame reception timeouts. The object must define
                            the subtraction operator (-) that returns a float number of seconds between the two
                            timestamps. In the simplest case it could be just monotonic time in seconds as a float,
                            i.e. as returned by the standard library function time.monotonic().
                            If this value is not provided, the library populates the value with time.monotonic()
                            automatically. This method of timestamping is highly imprecise but better than nothing.
        """
        if not isinstance(chunk, (bytes, bytearray)):
            raise TypeError('Expected bytes or bytearray, got %r' % type(chunk))

        timestamp = timestamp if timestamp is not None else time.monotonic()

        for b in chunk:
            self._parse_byte(b, timestamp)

        # Use the robust heuristics to detect if there's any hope to make use of this data we have in the future.
        # If not, report it to the application as extraneous as early as possible.
        # Note that we rely on short-circuiting here - later expressions may fail if evaluated earlier.
        should_abort = ((not self._inside_frame) or
                        (len(self._buffer) > self._max_payload_size) or
                        (float(timestamp - self._current_frame_timestamp) > self._frame_timeout))
        if should_abort:
            self._finalize(known_invalid=True)

    @property
    def callback(self) -> typing.Callable:
        """
        :return: Callback that is invoked upon reception of a frame.
        """
        return self._callback

    @callback.setter
    def callback(self, value: typing.Callable):
        """
        :param value:   The callback that will be invoked upon reception of a frame.
                        It is always invoked with one argument; the type of the argument can be either:
                         - ReceivedFrame - for received frames.
                         - bytes - for extraneous data.
        """
        self._callback = value


def encode(frame_type_code: int, payload: typing.Union[bytes, bytearray]) -> bytearray:
    """
    Creates an encoded representation of the specified frame, ready to be transmitted via the communication channel.
    :param frame_type_code:     Frame type code, 0 to 255 inclusive.
    :param payload:             Payload of the frame as bytes or bytearray.
    :return:                    Bytearray containing the encoded representation.
    """
    frame_type_code = int(frame_type_code)
    if not (0 <= frame_type_code <= 0xFF):
        raise ValueError('Invalid frame type code: %r' % frame_type_code)

    if not isinstance(payload, (bytes, bytearray)):
        raise TypeError('Expected bytes or bytearray, got %r' % type(payload))

    crc = CRCComputer().add(payload).add(frame_type_code).value

    output = bytearray()

    def append(chunk):
        for b in chunk:
            if b in (FRAME_DELIMITER, ESCAPE_CHARACTER):
                output.append(ESCAPE_CHARACTER)
                b ^= 0xFF

            output.append(b)

    # Assembling the packet
    output.append(FRAME_DELIMITER)
    append(payload)
    append((frame_type_code,
            (crc >> 0) & 0xFF,
            (crc >> 8) & 0xFF,
            (crc >> 16) & 0xFF,
            (crc >> 24) & 0xFF))
    output.append(FRAME_DELIMITER)

    return output
