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
from .message_base import MessageBase


class Command(enum.IntEnum):
    RESTART                     = 0
    POWER_OFF                   = 1
    LAUNCH_BOOTLOADER           = 2
    FACTORY_RESET               = 3
    PRINT_DIAGNOSTICS_BRIEF     = 4
    PRINT_DIAGNOSTICS_VERBOSE   = 5


class CommandExecutionStatus(enum.IntEnum):
    OK              = 0
    BAD_COMMAND     = 1
    MAYBE_LATER     = 2


class CommandRequestMessage(MessageBase):
    """
    Generic device command message.

        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u16             command         Generic command code.
    -----------------------------------------------------------------------------------------------
        2
    """
    MESSAGE_ID = 8

    _STRUCT = struct.Struct('<H')

    def __init__(self, command: Command):
        self.command = Command(int(command))        # Validness check

    def _encode(self) -> bytes:
        return self._STRUCT.pack(int(self.command))

    @staticmethod
    def _decode(encoded: bytes) -> 'CommandRequestMessage':
        command, = CommandRequestMessage._STRUCT.unpack(encoded)
        return CommandRequestMessage(Command(command))


class CommandResponseMessage(MessageBase):
    """
    Generic device command response message.

        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u16             command         Generic command code copied from the request.
        2       u8              status          Command execution status.
    -----------------------------------------------------------------------------------------------
        3
    """
    MESSAGE_ID = 9

    _STRUCT = struct.Struct('<HB')

    def __init__(self,
                 command: Command,
                 status: CommandExecutionStatus):
        self.command = Command(int(command))                # Validness check
        self.status = CommandExecutionStatus(int(status))   # Ditto

    def _encode(self) -> bytes:
        return self._STRUCT.pack(int(self.command), int(self.status))

    @staticmethod
    def _decode(encoded: bytes) -> 'CommandResponseMessage':
        command, status = CommandResponseMessage._STRUCT.unpack(encoded)
        return CommandResponseMessage(Command(command), CommandExecutionStatus(status))
