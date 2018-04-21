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
import typing
from decimal import Decimal
from .message_base import MessageBase, NANOSECONDS_PER_SECOND


class State(enum.IntEnum):
    """
    All possible states of the generic bootloader API. See the following state machine diagram.

        No valid application found ###################### Valid application found
                  /----------------# Bootloader started #----------\ /-------------------------------------------\
                  |                ######################          | |                                           |
                  v                                                v v  Boot delay expired                       |
            +-------------+                               +-----------+  (typically zero)  +-------------+       |
        /-->| NoAppToBoot |        /----------------------| BootDelay |------------------->| ReadyToBoot |       |
        |   +-------------+       /                       +-----------+                    +-------------+       |
        |          |             /                          |Boot cancelled                   |ReadyToBoot is    |
        |Upgrade   |<-----------/                           |e.g. received a state transition |an auxiliary      /
        |failed,   |Upgrade requested,                      |request to BootCancelled.        |state, it is     /
        |no valid  |e.g. received a state transition        v                                 |left automati-  /
        |image is  |request to AppUpgradeInProgress. +---------------+                        |cally ASAP.    /
        |now ava-  |<--------------------------------| BootCancelled |                        v              /
        |ilable    |                                 +---------------+                ###############       /
        |          v                                        ^                         # Booting the #      /
        | +----------------------+ Upgrade failed, but the  |                         # application #     /
        \-| AppUpgradeInProgress |--------------------------/                         ###############    /
          +----------------------+ existing valid image was not                                         /
                   |               altered and remains valid.                                          /
                   |                                                                                  /
                   | Upgrade successful, received image is valid.                                    /
                   \--------------------------------------------------------------------------------/
    """
    NO_APP_TO_BOOT          = 0
    BOOT_DELAY              = 1
    BOOT_CANCELLED          = 2
    APP_UPGRADE_IN_PROGRESS = 3
    READY_TO_BOOT           = 4


class ImageType(enum.IntEnum):
    """
    The bootloader protocol supports several types of images that can be written.
    The target is not required to support all of them.
    """
    APPLICATION                 = 0
    CERTIFICATE_OF_AUTHENTICITY = 1


class StatusRequestMessage(MessageBase):
    """
    Bootloader status request; contains the desired status, the response will contain the actual new status.
    Only the following states can be commanded:
         NoAppToBoot             - erases the application
         AppUpgradeInProgress    - initiates the upgrade process
         ReadyToBoot             - commands to launch the application (if present)
    All other states cannot be used here.

        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u8              desired_state   Which state the bootloader should transition to.
    -----------------------------------------------------------------------------------------------
        1
    """
    MESSAGE_ID = 10

    _STRUCT = struct.Struct('<B')

    def __init__(self, desired_state: State):
        self.desired_state = State(int(desired_state))        # Validness check

    def _encode(self) -> bytes:
        return self._STRUCT.pack(int(self.desired_state))

    @staticmethod
    def _decode(encoded: bytes) -> 'StatusRequestMessage':
        desired_state, = StatusRequestMessage._STRUCT.unpack(encoded)
        return StatusRequestMessage(State(desired_state))


class StatusResponseMessage(MessageBase):
    """
    Bootloader status response; contains the current status and stuff.

        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u64             uptime_ns       Bootloader's uptime in nanoseconds.
        8       u64             flags           Reserved.
        16      u8              state           The current state of the bootloader's standard state machine.
    -----------------------------------------------------------------------------------------------
        17
    """
    MESSAGE_ID = 11

    _STRUCT = struct.Struct('<QQB')

    def __init__(self,
                 uptime: Decimal,
                 flags: int,
                 state: State):
        self.uptime = Decimal(uptime)           # Uptime is in seconds
        self.flags = int(flags)
        self.state = State(int(state))          # Validness check

    def _encode(self) -> bytes:
        return self._STRUCT.pack(int(self.uptime * NANOSECONDS_PER_SECOND),
                                 int(self.flags),
                                 int(self.state))

    @staticmethod
    def _decode(encoded: bytes) -> 'StatusResponseMessage':
        uptime_ns, flags, state = StatusResponseMessage._STRUCT.unpack(encoded)
        return StatusResponseMessage(Decimal(uptime_ns) / NANOSECONDS_PER_SECOND,
                                     flags,
                                     State(state))


class _ImageDataMessageBase(MessageBase):
    """
    This is a common implementation for both request and response image data messages.

        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u64             image_offset    Offset from the beginning of the image. Must grow sequentially.
        1       u8              image_type      Kind of image contained in the message:
                                                    0 - application
                                                    1 - certificate of authenticity
        9       u8[<=256]       image_data      Image data at the specified offset. All messages except the last
                                                one are required to contain exactly 256 bytes of image data.
                                                The last message is required to contain less than 256 bytes of
                                                image data, possibly zero if the image size is 256-byte aligned.
                                                Terminated at the end of the message (implicit length).
    -----------------------------------------------------------------------------------------------
    """
    _STRUCT = struct.Struct('<QB')      # Data omitted

    MAX_IMAGE_DATA_SIZE = 256

    def __init__(self):
        self.image_offset = 0
        self.image_type = ImageType.APPLICATION
        self.image_data = b''

    def _encode(self) -> bytes:
        if len(self.image_data) > self.MAX_IMAGE_DATA_SIZE:
            raise ValueError('Too much image data: %r bytes' % len(self.image_data))

        return self._STRUCT.pack(int(self.image_offset),
                                 int(self.image_type)) + bytes(self.image_data)

    def _decode_in_place(self, encoded: bytes):
        boundary = _ImageDataMessageBase._STRUCT.size
        io, it = _ImageDataMessageBase._STRUCT.unpack(encoded[:boundary])
        self.image_offset = io
        self.image_type = ImageType(it)
        self.image_data = bytes(encoded[boundary:])


class ImageDataRequestMessage(_ImageDataMessageBase):
    """
    This message is used to write new application images via the bootloader.
    It can also be utilized to read data back, but this is not considered useful at the moment.
    """
    MESSAGE_ID = 12

    def __init__(self,
                 image_offset: int=None,
                 image_type: ImageType=None,
                 image_data: typing.Union[bytes, bytearray]=None):
        super(ImageDataRequestMessage, self).__init__()
        self.image_offset = int(image_offset or 0)
        self.image_type = ImageType(int(image_type or 0))    # Validation
        self.image_data = bytes(image_data or b'')

    @staticmethod
    def _decode(encoded: bytes) -> 'ImageDataRequestMessage':
        out = ImageDataRequestMessage()
        out._decode_in_place(encoded)
        return out


class ImageDataResponseMessage(_ImageDataMessageBase):
    """
    The counterpart for the request message.
    The response always contains the actual data from the memory.
    The host compares the written data with the response and determines whether the write was successful.
    All fields except image_data must have the same values as in the request.
    """
    MESSAGE_ID = 13

    def __init__(self,
                 image_offset: int=None,
                 image_type: ImageType=None,
                 image_data: typing.Union[bytes, bytearray]=None):
        super(ImageDataResponseMessage, self).__init__()
        self.image_offset = int(image_offset or 0)
        self.image_type = ImageType(int(image_type or 0))    # Validation
        self.image_data = bytes(image_data or b'')

    @staticmethod
    def _decode(encoded: bytes) -> 'ImageDataResponseMessage':
        out = ImageDataResponseMessage()
        out._decode_in_place(encoded)
        return out
