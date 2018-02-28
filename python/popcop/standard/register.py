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
    ValueType.EMPTY:        ' ',
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


class DataRequestMessage(MessageBase):
    """
    Register read request if the value is empty.
    Register write request if the value is not empty.

        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u8              name_length     Length of the next field.
        1       u8[<=93]        name            ASCII name, not terminated - see the previous field.
        1..94   u8              type_id         Type of the value contained in this message.
        2..95   u8[<=256]       encoded_payload Array of values whose types are defined by type_id.
    -----------------------------------------------------------------------------------------------
      <=351
    """
    MESSAGE_ID = 1

    def __init__(self,
                 name: str=None,
                 type_id: ValueType=None,
                 value: _VALUE_TYPE_ANNOTATION=None):
        self.name = str(name or '')
        self.type_id = ValueType(type_id if type_id is not None else ValueType.EMPTY)
        self.value = value

        # Check whether the value is serializable early!
        try:
            _encode_value(self.type_id, self.value)
        except Exception as ex:
            raise ValueError('The specified value (with type ID %r) cannot be encoded' % self.type_id) from ex

        if (self.type_id == ValueType.EMPTY) and (self.value is not None):
            raise ValueError('Incorrect type ID %r for value of type %r' % (self.type_id, type(self.value)))

    def __str__(self):
        return 'name=%r, type_id=%r, value=%r' % (self.name, self.type_id, self.value)

    __repr__ = __str__

    def _encode(self) -> bytes:
        out = _encode_name(self.name) + _encode_value(self.type_id, self.value)
        assert 2 <= len(out) <= 351     # The limits are defined in the layout specification
        return out

    @staticmethod
    def _decode(encoded: bytes) -> 'DataRequestMessage':
        # We instantiate the message object first and then populate the fields.
        # We could also pass the values to the constructor, but that would trigger the serializability check,
        # which we don't want. The reason we don't want the serializability check when parsing is best
        # explained by the following adage:
        #   Be conservative in what you send, be liberal in what you accept
        # Also known as "Robustness principle".
        msg = DataRequestMessage()
        msg.name, encoded = _decode_name(encoded)
        msg.type_id, msg.value = _decode_value(encoded)
        return msg


def _encode_name(name: str) -> bytearray:
    """
    Encoded register name representation:
        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u8              length          Length of the next field.
        1       u8[<=93]        name            ASCII name, not terminated - see the previous field.
    -----------------------------------------------------------------------------------------------
      <=94
    """
    name = name.encode()
    if len(name) > MAX_NAME_LENGTH:
        raise ValueError('The name is too long: %r bytes: %r' % (len(name), name))

    out = bytearray([len(name)]) + name
    assert 1 <= len(out) <= 94                  # The limits are defined in the layout specification above
    return out


assert _encode_name('') == b'\x00'
assert _encode_name('1') == b'\x011'
assert _encode_name('123') == b'\x03123'


def _decode_name(encoded: bytes) -> typing.Tuple[str, bytes]:
    name_len, encoded = int(encoded[0]), encoded[1:]
    if name_len > MAX_NAME_LENGTH:
        raise ValueError('Invalid name length: %r' % name_len)

    if name_len > len(encoded):
        raise ValueError('Data is not long enough: expected %r bytes, found %r' % (name_len, len(encoded)))

    return encoded[:name_len].decode(errors='replace'), encoded[name_len:]


# Is this a new hit in unit testing or what?
assert _decode_name(b'\x00') == ('', b'')
assert _decode_name(b'\x011234') == ('1', b'234')
assert _decode_name(b'\x031234') == ('123', b'4')
assert _decode_name(b'\x041234') == ('1234', b'')
try:
    _decode_name(b'')           # I love it
    assert False
except IndexError:
    pass
try:
    _decode_name(b'\x05123')    # So hip!
    assert False
except ValueError:
    pass
try:
    _decode_name(b'\x5e' + bytes([0] * 200))
    assert False
except ValueError:
    pass


def _encode_value(type_id: ValueType, value: _VALUE_TYPE_ANNOTATION) -> bytearray:
    """
    Encoded register value representation:
        Offset  Type            Name            Description
    -----------------------------------------------------------------------------------------------
        0       u8              type_id         Type of the value contained in this register.
        1       u8[<=256]       encoded_payload Array of values whose types are defined by type_id.
    -----------------------------------------------------------------------------------------------
      <=257
    """
    type_id = ValueType(int(type_id))   # Convert there and back to check correctness of the argument
    kind = VALUE_TYPE_TO_KIND[type_id]

    if kind == ValueKind.EMPTY:
        encoded = b''

    elif kind == ValueKind.SINGLE_VARIABLE_SIZE_ITEM:
        if value is None:
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
        encoded = struct.pack(struct_format, *value)    # Note that we must pass one scalar per argument

    else:
        raise ValueError('Unknown value type kind: %r' % kind)

    if len(encoded) > MAX_ENCODED_VALUE_LENGTH:
        raise ValueError('The value is too long to be encoded: %r' % value)

    out = bytearray([int(type_id)]) + encoded
    assert 1 <= len(out) <= 257                 # The limits are defined in the layout specification above
    return out


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


def _decode_value(encoded: bytes) -> typing.Tuple[ValueType, _VALUE_TYPE_ANNOTATION]:
    """
    Observe that value is always encoded last of all fields.
    This is because we don't want to pack a length field in front of it; instead, we treat the end of packet
    as the end of the encoded value stream.
    Therefore, this function doesn't return unparsed bytes, because by definition it leaves no byte unparsed \m/.
    """
    type_id, encoded = ValueType(encoded[0]), encoded[1:]
    if len(encoded) > MAX_ENCODED_VALUE_LENGTH:
        raise ValueError('The encoded form of the value is too long: %r' % encoded)

    kind = VALUE_TYPE_TO_KIND[type_id]

    if kind == ValueKind.EMPTY:
        value = None

    elif kind == ValueKind.SINGLE_VARIABLE_SIZE_ITEM:
        struct_format = str(len(encoded)) + _VALUE_TYPE_TO_STRUCT_FORMAT[type_id]
        value, = struct.unpack(struct_format, encoded)
        if type_id == ValueType.STRING:
            value = value.decode(errors='replace')

        value = VALUE_TYPE_TO_NATIVE_TYPE[type_id](value)

    elif kind == ValueKind.ARRAY_OF_SCALARS:
        struct_format = _VALUE_TYPE_TO_STRUCT_FORMAT[type_id]
        element_size = struct.Struct(struct_format).size
        num_elements = len(encoded) // element_size
        struct_format = str(num_elements) + struct_format
        value = list(struct.unpack(struct_format, encoded))

    else:
        raise ValueError('Unknown value type kind: %r' % kind)

    return type_id, value


assert _decode_value(b'\x00123456') == (ValueType.EMPTY, None)
assert _decode_value(b'\x01123456') == (ValueType.STRING, '123456')
assert _decode_value(b'\x02123456') == (ValueType.UNSTRUCTURED, b'123456')
assert _decode_value(b'\x01') == (ValueType.STRING, '')
assert _decode_value(b'\x02') == (ValueType.UNSTRUCTURED, b'')
assert _decode_value(bytes([3])) == (ValueType.BOOLEAN, [])
assert _decode_value(bytes([3, 0, 1, 2, 3])) == (ValueType.BOOLEAN, [False, True, True, True])
assert _decode_value(bytes([4])) == (ValueType.I64, [])
assert _decode_value(bytes([10, 0x12, 0x34])) == (ValueType.U16, [0x3412])
assert _decode_value(bytes([10, 0x12, 0x34, 0x56, 0x78])) == (ValueType.U16, [0x3412, 0x7856])
