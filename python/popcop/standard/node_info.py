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
import struct
import datetime
from .message_base import MessageBase


class NodeInfoMessage(MessageBase):
    """
    Representation of the standard NodeInfo message.
    An empty message is treated as a request for node info.

      Offset    Type    Name
    ---------------------------------------------------
          0     u64         software_image_crc
          8     u32         software_vcs_commit_id
         12     u32         software_build_timestamp_utc        UTC Unix time in seconds
         16     u8          software_version_major
         17     u8          software_version_minor
         18     u8          hardware_version_major
         19     u8          hardware_version_minor
         20     u8          flags                               1 - SW CRC set, 2 - SW release, 4 - SW dirty build
         21     u8          mode                                0 - normal, 1 - bootloader
         22     u8[2]       <reserved>
         24     u8[16]      globally_unique_id
         40     u8[80]      node_name
        120     u8[80]      node_description
        200     u8[80]      build_environment_description
        280     u8[80]      runtime_environment_description
        360     u8[<=255]   certificate_of_authenticity         Until the end of the message
    ---------------------------------------------------
        360..615
    """
    MESSAGE_ID = 0

    _STRUCT = struct.Struct('< Q L L 6B xx 16s 80s 80s 80s 80s')  # Trailing certificate_of_authenticity excluded

    class Mode(enum.IntEnum):
        NORMAL = 0
        BOOTLOADER = 1

    class _Flags(enum.IntEnum):
        SOFTWARE_IMAGE_CRC_AVAILABLE = 1
        SOFTWARE_RELEASE_BUILD = 2
        SOFTWARE_DIRTY_BUILD = 4

    def __init__(self):
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

    @property
    def is_request(self) -> bool:
        """
        Per Popcop, an empty node info message is a request for node info.
        """
        return (not self.node_description) and (not self.node_name)

    def __str__(self):
        out = 'sw_crc=%r, sw_vcs=%r, sw_ts=%r, ' % (self.software_image_crc, self.software_vcs_commit_id,
                                                    self.software_build_timestamp_utc)

        out += 'sw_ver=%r.%r, hw_ver=%r.%r, mode=%s, guid=%s, ' % \
               (self.software_version_major, self.software_version_minor,
                self.hardware_version_major, self.hardware_version_minor,
                self.mode, self.globally_unique_id.hex())

        out += 'name=%r, desc=%r, bed=%r, red=%r, coa=%s' % \
               (self.node_name, self.node_description, self.build_environment_description,
                self.runtime_environment_description, self.certificate_of_authenticity.hex())
        return out

    __repr__ = __str__

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

        out = self._STRUCT.pack(int(self.software_image_crc or 0),
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

    @staticmethod
    def _decode(encoded: bytes) -> 'NodeInfoMessage':
        msg = NodeInfoMessage()

        if len(encoded) < msg._STRUCT.size:   # An empty serialized payload is just a request for node info
            return msg

        msg.software_image_crc, \
            msg.software_vcs_commit_id, \
            build_timestamp_utc, \
            msg.software_version_major, \
            msg.software_version_minor, \
            msg.hardware_version_major, \
            msg.hardware_version_minor, \
            flags, \
            mode, \
            msg.globally_unique_id, \
            node_name, \
            node_description, \
            build_environment_description, \
            runtime_environment_description, \
            = msg._STRUCT.unpack(encoded[:msg._STRUCT.size])

        if (flags & msg._Flags.SOFTWARE_IMAGE_CRC_AVAILABLE) == 0:
            msg.software_image_crc = None

        msg.software_build_timestamp_utc = datetime.datetime.utcfromtimestamp(build_timestamp_utc)

        msg.software_release_build = bool(flags & msg._Flags.SOFTWARE_RELEASE_BUILD)
        msg.software_dirty_build   = bool(flags & msg._Flags.SOFTWARE_DIRTY_BUILD)

        msg.mode = msg.Mode(mode)
        msg.node_name                       = _decode_fixed_capacity_string(node_name)
        msg.node_description                = _decode_fixed_capacity_string(node_description)
        msg.build_environment_description   = _decode_fixed_capacity_string(build_environment_description)
        msg.runtime_environment_description = _decode_fixed_capacity_string(runtime_environment_description)

        msg.certificate_of_authenticity = encoded[msg._STRUCT.size:]

        return msg


def _decode_fixed_capacity_string(s: bytes) -> str:
    return s.rstrip(b'\0').decode(errors='ignore')
