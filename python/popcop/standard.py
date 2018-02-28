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

import enum
import typing
import struct
import datetime
from . import transport
from .transport import ReceivedFrame
from . import STANDARD_FRAME_TYPE_CODE


DEFAULT_STANDARD_REQUEST_TIMEOUT = 1.0
HEADER_SIZE = 2


def _decode_fixed_capacity_string(s: bytes) -> str:
    return s.rstrip(b'\0').decode(errors='ignore')


class MessageBase:
    MESSAGE_ID = None

    SERIALIZER = None

    def _encode(self) -> bytes:
        raise NotImplementedError


class NodeInfoMessage(MessageBase):
    """
    Representation of the standard NodeInfo message.

      Offset    Type    Name
    ---------------------------------------------------
        0       u64         software_image_crc
        8       u32         software_vcs_commit_id
        12      u32         software_build_timestamp_utc        UTC Unix time in seconds
        16      u8          software_version_major
        17      u8          software_version_minor
        18      u8          hardware_version_major
        19      u8          hardware_version_minor
        20      u8          flags                               1 - SW CRC set, 2 - SW release, 4 - SW dirty build
        21      u8          mode                                0 - normal, 1 - bootloader
        22      u8[2]       <reserved>
        24      u8[16]      globally_unique_id
        40      u8[80]      node_name
        120     u8[80]      node_description
        200     u8[80]      build_environment_description
        280     u8[80]      runtime_environment_description
        360     u8[<=255]   certificate_of_authenticity         Until the end of the message
    ---------------------------------------------------
        <=615
    """
    MESSAGE_ID = 0

    SERIALIZER = struct.Struct('< Q L L 6B xx 16s 80s 80s 80s 80s')  # Trailing certificate_of_authenticity excluded

    class Mode(enum.IntEnum):
        NORMAL = 0
        BOOTLOADER = 1

    class _Flags(enum.IntEnum):
        SOFTWARE_IMAGE_CRC_AVAILABLE = 1
        SOFTWARE_RELEASE_BUILD = 2
        SOFTWARE_DIRTY_BUILD = 4

    def __init__(self, decode_from: typing.Optional[bytes]=None):
        self.software_image_crc = None
        self.software_vcs_commit_id = 0
        self.software_build_timestamp_utc = datetime.datetime.utcfromtimestamp(0)
        self.software_version_major = 0
        self.software_version_minor = 0
        self.hardware_version_major = 0
        self.hardware_version_minor = 0
        self.software_release_build = False
        self.software_dirty_build = False
        self.mode = self.Mode.NORMAL
        self.globally_unique_id = bytearray([0] * 16)
        self.node_name = ''
        self.node_description = ''
        self.build_environment_description = ''
        self.runtime_environment_description = ''
        self.certificate_of_authenticity = bytearray()

        if decode_from:
            self.software_image_crc,\
                self.software_vcs_commit_id, \
                build_timestamp_utc, \
                self.software_version_major, \
                self.software_version_minor, \
                self.hardware_version_major, \
                self.hardware_version_minor, \
                flags, \
                mode, \
                self.globally_unique_id, \
                node_name, \
                node_description, \
                build_environment_description, \
                runtime_environment_description, \
                = self.SERIALIZER.unpack(decode_from[:self.SERIALIZER.size])

            if (flags & self._Flags.SOFTWARE_IMAGE_CRC_AVAILABLE) == 0:
                self.software_image_crc = None

            self.software_build_timestamp_utc = datetime.datetime.utcfromtimestamp(build_timestamp_utc)

            self.software_release_build = bool(flags & self._Flags.SOFTWARE_RELEASE_BUILD)
            self.software_dirty_build = bool(flags & self._Flags.SOFTWARE_DIRTY_BUILD)

            self.mode = self.Mode(mode)
            self.node_name = _decode_fixed_capacity_string(node_name)
            self.node_description = _decode_fixed_capacity_string(node_description)
            self.build_environment_description = _decode_fixed_capacity_string(build_environment_description)
            self.runtime_environment_description = _decode_fixed_capacity_string(runtime_environment_description)

            self.certificate_of_authenticity = decode_from[self.SERIALIZER.size:]

    def _encode(self) -> bytes:
        flags = 0
        if isinstance(self.software_image_crc, int):
            flags |= self._Flags.SOFTWARE_IMAGE_CRC_AVAILABLE

        if self.software_release_build:
            flags |= self._Flags.SOFTWARE_RELEASE_BUILD

        if self.software_dirty_build:
            flags |= self._Flags.SOFTWARE_DIRTY_BUILD

        build_timestamp_utc = self.software_build_timestamp_utc or 0
        if hasattr(build_timestamp_utc, 'timestamp'):
            # UTC timestamp conversion https://docs.python.org/3/library/datetime.html#datetime.datetime.timestamp
            build_timestamp_utc = int(build_timestamp_utc.replace(tzinfo=datetime.timezone.utc).timestamp())
        else:
            build_timestamp_utc = int(build_timestamp_utc)

        if not (0 <= build_timestamp_utc < 2**32):
            raise ValueError('Invalid build timestamp: %r' % build_timestamp_utc)

        out = self.SERIALIZER.pack(int(self.software_image_crc or 0),
                                   self.software_vcs_commit_id,
                                   build_timestamp_utc,
                                   self.software_version_major,
                                   self.software_version_minor,
                                   self.hardware_version_major,
                                   self.hardware_version_minor,
                                   flags,
                                   int(self.mode),
                                   bytes(self.globally_unique_id),
                                   self.node_name.encode(),
                                   self.node_description.encode(),
                                   self.build_environment_description.encode(),
                                   self.runtime_environment_description.encode())

        return out + bytes(self.certificate_of_authenticity)

    def __str__(self):
        out = 'sw_crc=%r, sw_vcs=%r, sw_ts=%r, ' % (self.software_image_crc, self.software_vcs_commit_id,
                                                    self.software_build_timestamp_utc)

        out += 'sw_ver=%r.%r, hw_ver=%r.%r, mode=%s, guid=%s, ' % \
               (self.software_version_major, self.software_version_minor,
                self.hardware_version_major, self.hardware_version_minor,
                self.mode, self.globally_unique_id.hex())

        out += 'name=%r, description=%r, bed=%r, red=%r, coa=%s' % \
               (self.node_name, self.node_description, self.build_environment_description,
                self.runtime_environment_description, self.certificate_of_authenticity.hex())
        return out

    __repr__ = __str__


def encode_header(msg_type: typing.Type[MessageBase]) -> bytes:
    return bytes([
        (msg_type.MESSAGE_ID >> 0) & 0xFF,
        (msg_type.MESSAGE_ID >> 8) & 0xFF,
    ])


def decode_header(raw_header_bytes: typing.Union[bytes, bytearray]) -> tuple:
    return raw_header_bytes[0] + (raw_header_bytes[1] << 8),


def encode(msg: MessageBase) -> bytearray:
    """
    Encodes the provided message into a binary frame.
    :param msg:     The message to encode.
    :return:        bytearray.
    """
    header = encode_header(type(msg))
    assert len(header) == HEADER_SIZE

    # noinspection PyProtectedMember
    raw = header + msg._encode()

    return transport.encode(STANDARD_FRAME_TYPE_CODE, raw)


def decode(received_frame: ReceivedFrame) -> typing.Optional[MessageBase]:
    """
    This function attempts to decode the contents of the provided received frame.
    Returns None if the frame is invalid or unknown. May throw if the frame is malformed.
    :param received_frame:  The received frame. The frame type code must be STANDARD_FRAME_TYPE_CODE,
                            otherwise the function throws ValueError.
    :return:    Decoded message object such as NodeInfoMessage, or None.
    """
    if received_frame.frame_type_code != STANDARD_FRAME_TYPE_CODE:
        raise ValueError('This is not a standard frame: %r' % received_frame)

    pl = received_frame.payload
    try:
        header, data = pl[:HEADER_SIZE], pl[HEADER_SIZE:]
    except IndexError:
        return

    message_id, = decode_header(header)

    if message_id == NodeInfoMessage.MESSAGE_ID:
        if len(data) >= NodeInfoMessage.SERIALIZER.size:
            return NodeInfoMessage(data)
