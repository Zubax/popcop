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


MAX_NAME_LENGTH = 93
MAX_ENCODED_VALUE_LENGTH = 256


class ValueType(enum.IntEnum):
    """
    The values match those of the type ID field in register data messages; i.e. these are actual wire-compatible values.
    """
    EMPTY           = 0     # Maps to None
    STRING          = 1
    UNSTRUCTURED    = 2
    BOOLEAN         = 3
    I64             = 4
    I32             = 5
    I16             = 6
    I8              = 7
    U64             = 8
    U32             = 9
    U16             = 10
    U8              = 11
    F64             = 12
    F32             = 13


class ValueKind(enum.Enum):
    EMPTY                       = 0
    SINGLE_VARIABLE_SIZE_ITEM   = 1
    ARRAY_OF_SCALARS            = 2


VALUE_TYPE_TO_NATIVE_TYPE = {
    ValueType.EMPTY:        type(None),
    ValueType.STRING:       str,
    ValueType.UNSTRUCTURED: bytes,
    ValueType.BOOLEAN:      bool,
    ValueType.I64:          int,
    ValueType.I32:          int,
    ValueType.I16:          int,
    ValueType.I8:           int,
    ValueType.U64:          int,
    ValueType.U32:          int,
    ValueType.U16:          int,
    ValueType.U8:           int,
    ValueType.F64:          float,
    ValueType.F32:          float,
}


VALUE_TYPE_TO_KIND = {
    ValueType.EMPTY:        ValueKind.EMPTY,
    ValueType.STRING:       ValueKind.SINGLE_VARIABLE_SIZE_ITEM,
    ValueType.UNSTRUCTURED: ValueKind.SINGLE_VARIABLE_SIZE_ITEM,
    ValueType.BOOLEAN:      ValueKind.ARRAY_OF_SCALARS,
    ValueType.I64:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.I32:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.I16:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.I8:           ValueKind.ARRAY_OF_SCALARS,
    ValueType.U64:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.U32:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.U16:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.U8:           ValueKind.ARRAY_OF_SCALARS,
    ValueType.F64:          ValueKind.ARRAY_OF_SCALARS,
    ValueType.F32:          ValueKind.ARRAY_OF_SCALARS,
}


_VALUE_TYPE_TO_STRUCT_FORMAT = {
    ValueType.EMPTY:        '',
    ValueType.STRING:       's',
    ValueType.UNSTRUCTURED: 's',
    ValueType.BOOLEAN:      '?',
    ValueType.I64:          'q',
    ValueType.I32:          'l',
    ValueType.I16:          'h',
    ValueType.I8:           'b',
    ValueType.U64:          'Q',
    ValueType.U32:          'L',
    ValueType.U16:          'H',
    ValueType.U8:           'B',
    ValueType.F64:          'd',
    ValueType.F32:          'f',
}


_VALUE_TYPE_ANNOTATION = typing.Union[
    type(None),
    str,
    bytes,
    typing.List[bool],
    typing.List[int],
    typing.List[float]
]


class RegisterDataRequestMessage(MessageBase):
    def __init__(self):
        self.name = ''
        self.type_id = ValueType.EMPTY
        self.value = None

    def __str__(self):
        return 'name=%r, type_id=%r, value=%r' % (self.name, self.type_id, self.value)

    __repr__ = __str__

    def _encode(self) -> bytes:
        return b''


def _encode_name(name: str) -> bytearray:
    name = name.encode()
    if len(name) > MAX_NAME_LENGTH:
        raise ValueError('The name is too long: %r bytes: %r' % (len(name), name))

    return bytearray([len(name)]) + name


assert _encode_name('') == b'\x00'
assert _encode_name('1') == b'\x011'
assert _encode_name('123') == b'\x03123'


def _encode_value(type_id: ValueType, value: _VALUE_TYPE_ANNOTATION) -> bytearray:
    type_id = ValueType(int(type_id))   # Convert there and back to check correctness of the argument
    kind = VALUE_TYPE_TO_KIND[type_id]

    if kind == ValueKind.EMPTY:
        encoded = b''

    elif kind == ValueKind.SINGLE_VARIABLE_SIZE_ITEM:
        if not value:
            value = bytes()

        if isinstance(value, str):
            value = value.encode()

        if not isinstance(value, bytes):
            value = bytes(value)

        struct_format = str(len(value)) + _VALUE_TYPE_TO_STRUCT_FORMAT[type_id]
        encoded = struct.pack(struct_format, value)

    elif kind == ValueKind.ARRAY_OF_SCALARS:
        try:
            iter(value)             # Implicitly convert scalars into arrays
        except TypeError:
            value = [value]

        native_type = VALUE_TYPE_TO_NATIVE_TYPE[type_id]
        value = [native_type(x) for x in value]
        struct_format = str(len(value)) + _VALUE_TYPE_TO_STRUCT_FORMAT[type_id]
        encoded = struct.pack(struct_format, *value)

    else:
        raise ValueError('Unknown value type kind: %r' % kind)

    if len(encoded) > MAX_ENCODED_VALUE_LENGTH:
        raise ValueError('The value is too long to be encoded: %r' % value)

    return bytearray([int(type_id)]) + encoded


# Is this a new hit in unit testing or what?
assert _encode_value(ValueType.EMPTY, None) == b'\x00'
assert _encode_value(ValueType.EMPTY, '123') == b'\x00'
assert _encode_value(ValueType.STRING, '') == b'\x01'
assert _encode_value(ValueType.STRING, '123') == b'\x01123'
assert _encode_value(ValueType.UNSTRUCTURED, b'') == b'\x02'
assert _encode_value(ValueType.UNSTRUCTURED, b'123') == b'\x02123'
assert _encode_value(ValueType.BOOLEAN, []) == b'\x03'
assert _encode_value(ValueType.BOOLEAN, [0, 1, 2, 3]) == bytes([3, 0, 1, 1, 1])
assert _encode_value(ValueType.I64, []) == bytes([4])
assert _encode_value(ValueType.U16, 0x1234) == bytes([10, 0x34, 0x12])
assert _encode_value(ValueType.U8, [0x12, 0x34, 0x56, 0x78]) == bytes([11, 0x12, 0x34, 0x56, 0x78])
