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
from .. import transport
from ..transport import ReceivedFrame
from .. import STANDARD_FRAME_TYPE_CODE
from .message_base import MessageBase
from .node_info import NodeInfoMessage
from . import register
from . import device_management
from . import bootloader


DEFAULT_STANDARD_REQUEST_TIMEOUT = 1.0
HEADER_SIZE = 2


def encode_header(msg_type: typing.Type[MessageBase]) -> bytes:
    return bytes([
        (msg_type.MESSAGE_ID >> 0) & 0xFF,
        (msg_type.MESSAGE_ID >> 8) & 0xFF,
    ])


def decode_header(raw_header_bytes: typing.Union[bytes, bytearray]) -> tuple:
    return raw_header_bytes[0] + (raw_header_bytes[1] << 8),


def encode(msg: MessageBase) -> bytearray:
    """
    Encodes the provided message into a binary frame (with the correct header).
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

    :return:                Decoded message object such as NodeInfoMessage, or None.
    """
    if received_frame.frame_type_code != STANDARD_FRAME_TYPE_CODE:
        raise ValueError('This is not a standard frame: %r' % received_frame)

    pl = received_frame.payload
    try:
        header, data = pl[:HEADER_SIZE], pl[HEADER_SIZE:]
    except IndexError:
        return

    message_id, = decode_header(header)

    for mt in MessageBase.__subclasses__():
        if mt.MESSAGE_ID == message_id:
            # noinspection PyProtectedMember,PyUnresolvedReferences
            return mt._decode(data)
