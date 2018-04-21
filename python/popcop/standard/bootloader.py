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
from .message_base import MessageBase


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


