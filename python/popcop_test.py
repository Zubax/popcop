#!/usr/bin/env python3
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

import os
import sys
import time
import popcop
import asyncio
import logging
import unittest
import threading
import contextlib
from decimal import Decimal

try:
    import serial
except ImportError:
    if os.environ.get('SERIAL_REQUIRED', 0):
        raise ImportError('Could not import serial, and $SERIAL_REQUIRED is set')

    serial = None


_logger = logging.getLogger(__name__)


def slow_test(fun):
    @unittest.skipIf(os.environ.get('SKIP_SLOW_TESTS', 0), '$SKIP_SLOW_TESTS is set')
    def decorator(*args, **kwargs):
        fun(*args, **kwargs)

    return decorator


class TestCRC(unittest.TestCase):
    def test(self):
        c = popcop.transport.CRCComputer()
        c.add(b'123')
        c.add(ord('4'))
        c.add(ord('5'))
        c.add(ord('6'))
        c.add(b'789')
        self.assertEqual(c.value, 0xE3069283)
        self.assertFalse(c.is_residue_correct())
        c.add(0x83)
        c.add(0x92)
        c.add(0x06)
        c.add(0xE3)
        self.assertTrue(c.is_residue_correct())


class TestParserEncoder(unittest.TestCase):
    def test_parser(self):
        frame = None
        extraneous = None

        def callback(o):
            nonlocal frame, extraneous
            if isinstance(o, bytes):
                frame, extraneous = None, o
            elif isinstance(o, popcop.transport.ReceivedFrame):
                frame, extraneous = o, None
            else:
                raise TypeError('Test error: unexpected type of data: %r' % type(o))

        def match_frame(frame_type_code: int, payload: bytes, timestamp):
            self.assertIsNotNone(frame)
            self.assertEqual(frame_type_code, frame.frame_type_code)
            self.assertEqual(payload, frame.payload)
            self.assertEqual(timestamp, frame.timestamp)

        p = popcop.transport.Parser(callback=callback, max_payload_size=1024, frame_timeout=1)

        p.parse(bytes([0x8E, 123, 0x67, 0xAC, 0x6C, 0xBA, 0x8E]), 123456789)
        match_frame(123, b'', 123456789)
        self.assertIsNone(extraneous)

        p.parse(bytes([42, 12, 34, 56, 78, 90, 0xCE, 0x4E, 0x88, 0xBC, 0x8E]))  # No front delimiter
        match_frame(90, bytes([42, 12, 34, 56, 78]), 123456789)                 # Timestamp from the last test case
        self.assertIsNone(extraneous)

        p.parse(bytes([0x8E,
                       0x9E, 0x8E ^ 0xFF,
                       0x9E, 0x9E ^ 0xFF,
                       0x91, 0x5C, 0xA9, 0xC0,
                       0x8E]), 123)
        match_frame(0x9E, bytes([0x8E]), 123)
        self.assertIsNone(extraneous)

        # Same as above, but split over a large time interval
        frame = None
        p.parse(bytes([0x9E, 0x8E ^ 0xFF]), 123)
        self.assertIsNone(frame)
        self.assertIsNone(extraneous)
        p.parse(bytes([0x9E, 0x9E ^ 0xFF,
                       0x91, 0x5C, 0xA9, 0xC0]), 126)
        self.assertIsNone(frame)
        self.assertIsNotNone(extraneous)
        self.assertEqual(extraneous, bytes([0x8E, 0x9E, 0x91, 0x5C, 0xA9, 0xC0]))

        # Now we're not inside a frame, so no unescaping will be done
        frame = None
        extraneous = None
        p.parse(bytes([0x9E, 0x8E ^ 0xFF,
                       0x9E, 0x9E ^ 0xFF,
                       0x91, 0x5C, 0xA9, 0xC0]), 128)
        self.assertIsNone(frame)
        self.assertIsNotNone(extraneous)
        self.assertEqual(extraneous,
                         bytes([0x9E, 0x8E ^ 0xFF,
                                0x9E, 0x9E ^ 0xFF,
                                0x91, 0x5C, 0xA9, 0xC0]))

        frame = None
        p.parse(b'\x8E\x8E\x8EHello!\x8E')
        self.assertIsNone(frame)
        self.assertEqual(extraneous, b'Hello!')

    def test_encoder(self):
        from popcop.transport import encode
        self.assertEqual(encode(123, b''),
                         bytes([0x8E, 123, 0x67, 0xAC, 0x6C, 0xBA, 0x8E]))

        self.assertEqual(encode(90, bytes([42, 12, 34, 56, 78])),
                         bytes([0x8E, 42, 12, 34, 56, 78, 90, 0xCE, 0x4E, 0x88, 0xBC, 0x8E]))

        self.assertEqual(encode(0x9E, bytes([0x8E])),
                         bytes([0x8E, 0x9E, 0x8E ^ 0xFF, 0x9E, 0x9E ^ 0xFF, 0x91, 0x5C, 0xA9, 0xC0, 0x8E]))


class TestStandardMessages(unittest.TestCase):
    def test_node_info(self):
        carefully_crafted_message = bytes([
            0x00, 0x00,                                         # Message ID

            0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xFF,     # SW CRC
            0xEF, 0xBE, 0xAD, 0xDE,                             # SW VCS ID
            0xD2, 0x00, 0xDF, 0xBA,                             # SW build timestamp UTC
            0x01, 0x02,                                         # SW version
            0x03, 0x04,                                         # HW version
            0x07,                                               # Flags (CRC set, release build, dirty build)
            0x00,                                               # Mode
            0x00, 0x00,                                         # Reserved (always zero!)

            0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09,     # Unique ID
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,

            0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x21, 0x00, 0x00,     # Name
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

            0x53, 0x70, 0x61, 0x63, 0x65, 0x21, 0x00, 0x00,     # Description
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

            0x75, 0x70, 0x79, 0x61, 0x63, 0x68, 0x6b, 0x61,     # Build environment description
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

            0x52, 0x55, 0x4e, 0x54, 0x49, 0x4d, 0x45, 0x21,     # Runtime environment description
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

            0x01, 0x02, 0x03, 0x04
        ])

        # Decode test
        m = popcop.standard.decode(popcop.transport.ReceivedFrame(0xFF, carefully_crafted_message, 0))
        print(m)
        self.assertIsNotNone(m)

        if isinstance(m, popcop.standard.NodeInfoMessage):
            self.assertEqual(m.software_image_crc, 0xFFDEBC9A78563412)
            self.assertEqual(m.software_vcs_commit_id, 0xDEADBEEF)
            self.assertEqual(m.software_build_timestamp_utc.isoformat(), '2069-05-07T18:28:34')
            self.assertEqual(m.software_version_major, 1)
            self.assertEqual(m.software_version_minor, 2)
            self.assertEqual(m.hardware_version_major, 3)
            self.assertEqual(m.hardware_version_minor, 4)
            self.assertTrue(m.software_release_build)
            self.assertTrue(m.software_dirty_build)
            self.assertEqual(m.mode, m.Mode.NORMAL)
            self.assertEqual(m.globally_unique_id, bytes([0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09,
                                                          0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01]))
            self.assertEqual(m.node_name, 'Hello!')
            self.assertEqual(m.node_description, 'Space!')
            self.assertEqual(m.build_environment_description, 'upyachka')
            self.assertEqual(m.runtime_environment_description, 'RUNTIME!')
            self.assertEqual(m.certificate_of_authenticity, bytes([1, 2, 3, 4]))
        else:
            raise ValueError('Expected node info, got %r' % type(m))

        # Encode test
        encoded = popcop.standard.encode(m)
        self.assertEqual(encoded,
                         popcop.transport.encode(popcop.STANDARD_FRAME_TYPE_CODE,
                                                 carefully_crafted_message))

    def test_register_data_request(self):
        from popcop.transport import ReceivedFrame
        from popcop.standard.register import DataRequestMessage, ValueType
        from popcop import STANDARD_FRAME_TYPE_CODE

        self.assertEqual(DataRequestMessage()._encode(), bytes([0, 0]))
        self.assertEqual(DataRequestMessage(name='Hello')._encode(), b'\x05Hello\x00')
        self.assertEqual(DataRequestMessage(value=123, type_id=ValueType.I8). _encode(),
                         bytes([0, 7, 123]))

        msg = popcop.standard.decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE,
                                                   bytes([1, 0, 0, 0]),
                                                   0))
        self.assertIsInstance(msg, DataRequestMessage)
        self.assertEqual(msg.name, '')
        self.assertEqual(msg.type_id, ValueType.EMPTY)
        self.assertIsNone(msg.value)

        msg = popcop.standard.decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE,
                                                   bytes([1, 0, 3, 48, 49, 50, 1, 52, 53, 54]),
                                                   0))
        self.assertIsInstance(msg, DataRequestMessage)
        self.assertEqual(msg.name, '012')
        self.assertEqual(msg.type_id, ValueType.STRING)
        self.assertEqual(msg.value, '456')
        print(msg)

    def test_register_data_response(self):
        from popcop.transport import ReceivedFrame
        from popcop.standard.register import DataResponseMessage, ValueType
        from popcop import STANDARD_FRAME_TYPE_CODE

        self.assertEqual(DataResponseMessage()._encode(), bytes([0, 0, 0, 0, 0, 0, 0, 0,
                                                                 0, 0, 0]))
        self.assertEqual(DataResponseMessage(name='Hello', flags=1)._encode(),
                         b'\0\0\0\0\0\0\0\0\1\x05Hello\x00')

        # >>> '%016x' % (int(Decimal('2.123') * _NANOSECONDS_PER_SECOND))
        # '000000007e8a68c0'        # Don't forget to flip byte order because little-endian
        self.assertEqual(DataResponseMessage(timestamp=Decimal('2.123'), flags=2, name='Hello',
                                             value=123, type_id=ValueType.I8). _encode(),
                         b'\xc0\x68\x8a\x7e\x00\x00\x00\x00'    # Timestamp 2.123 seconds - see computation above
                         b'\x02\x05Hello\x07\x7b')

        msg = popcop.standard.decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE,
                                                   bytes([2, 0,
                                                          0, 0, 0, 0, 0, 0, 0, 0,
                                                          0, 0, 0]),
                                                   0))
        self.assertIsInstance(msg, DataResponseMessage)
        self.assertEqual(msg.timestamp, Decimal(0))
        self.assertEqual(msg.flags.mutable, False)
        self.assertEqual(msg.flags.persistent, False)
        self.assertEqual(msg.name, '')
        self.assertEqual(msg.type_id, ValueType.EMPTY)
        self.assertIsNone(msg.value)

        msg = popcop.standard.decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE,
                                                   bytes([2, 0,
                                                          0xc0, 0x68, 0x8a, 0x7e, 0x0, 0x0, 0x0, 0x0,  # TS as above
                                                          3,
                                                          3, 48, 49, 50,
                                                          1, 52, 53, 54]),
                                                   0))
        self.assertIsInstance(msg, DataResponseMessage)
        self.assertEqual(msg.timestamp, Decimal('2.123'))
        self.assertEqual(msg.flags.mutable, True)
        self.assertEqual(msg.flags.persistent, True)
        self.assertEqual(msg.name, '012')
        self.assertEqual(msg.type_id, ValueType.STRING)
        self.assertEqual(msg.value, '456')
        print(msg)

        msg = popcop.standard.decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE,
                                                   bytes([2, 0,
                                                          0xc0, 0x68, 0x8a, 0x7e, 0x0, 0x0, 0x0, 0x0,  # TS as above
                                                          0,
                                                          16, 99, 116, 108, 46, 110, 117, 109, 95, 97, 116, 116, 101,
                                                          109, 112, 116, 115,
                                                          9,
                                                          100, 0, 0, 0]),
                                                   0))
        self.assertIsInstance(msg, DataResponseMessage)
        self.assertEqual(msg.timestamp, Decimal('2.123'))
        self.assertEqual(msg.flags.mutable, False)
        self.assertEqual(msg.flags.persistent, False)
        self.assertEqual(msg.name, 'ctl.num_attempts')
        self.assertEqual(msg.type_id, ValueType.U32)
        self.assertEqual(msg.value, [100])
        print(msg)

    def test_register_discovery_request(self):
        from popcop.transport import ReceivedFrame
        from popcop.standard import encode, decode
        from popcop.standard.register import DiscoveryRequestMessage
        from popcop import STANDARD_FRAME_TYPE_CODE

        # Encoding a full frame then stripping the delimiters, type code, and CRC.
        self.assertEqual(encode(DiscoveryRequestMessage())[1:-6], bytes([3, 0, 0, 0]))
        self.assertEqual(encode(DiscoveryRequestMessage(12345))[1:-6], bytes([3, 0, 0x39, 0x30]))

        msg = decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE, bytes([3, 0, 0, 0]), 0))
        self.assertIsInstance(msg, DiscoveryRequestMessage)
        self.assertEqual(msg.index, 0)

        msg = decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE, bytes([3, 0, 0x39, 0x30]), 0))
        self.assertIsInstance(msg, DiscoveryRequestMessage)
        self.assertEqual(msg.index, 12345)
        print(msg)

    def test_register_discovery_response(self):
        from popcop.transport import ReceivedFrame
        from popcop.standard import encode, decode
        from popcop.standard.register import DiscoveryResponseMessage
        from popcop import STANDARD_FRAME_TYPE_CODE

        # Encoding a full frame then stripping the delimiters, type code, and CRC.
        self.assertEqual(encode(DiscoveryResponseMessage())[1:-6], bytes([4, 0, 0, 0, 0]))
        self.assertEqual(encode(DiscoveryResponseMessage(12345, 'Hello'))[1:-6],
                         b'\x04\x00\x39\x30\x05Hello')

        msg = decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE, bytes([4, 0, 0, 0, 0]), 0))
        self.assertIsInstance(msg, DiscoveryResponseMessage)
        self.assertEqual(msg.index, 0)
        self.assertEqual(msg.name, '')

        msg = decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE, bytes([4, 0, 0x39, 0x30, 5]) + b'Hello', 0))
        self.assertIsInstance(msg, DiscoveryResponseMessage)
        self.assertEqual(msg.index, 12345)
        self.assertEqual(msg.name, 'Hello')
        print(msg)

    def test_device_management_command_request(self):
        from popcop.transport import ReceivedFrame
        from popcop.standard import encode, decode
        from popcop.standard.device_management import CommandRequestMessage, Command
        from popcop import STANDARD_FRAME_TYPE_CODE

        # Encoding a full frame then stripping the delimiters, type code, and CRC.
        self.assertEqual(encode(CommandRequestMessage(Command.LAUNCH_BOOTLOADER))[1:-6],         bytes([8, 0, 2, 0]))
        self.assertEqual(encode(CommandRequestMessage(Command.PRINT_DIAGNOSTICS_VERBOSE))[1:-6], bytes([8, 0, 5, 0]))

        msg = decode(ReceivedFrame(STANDARD_FRAME_TYPE_CODE, bytes([8, 0, 1, 0]), 0))
        self.assertIsInstance(msg, CommandRequestMessage)
        self.assertEqual(msg.command, Command.POWER_OFF)


@unittest.skipUnless(serial, 'PySerial is not available. Please install it to test this feature.')
class TestSerialMultiprocessing(unittest.TestCase):
    """
    This test requires a port with a loopback connection (RX<->TX).
    By default we're using the special URL "loop://" as a serial port name - it instructs the pyserial library
    to emulate a loopback connection in software. Also interesting option is "spy://" - it might help with debugging.
    More info about serial port URL: https://pythonhosted.org/pyserial/url_handlers.html
    The default can be overridden by setting the environment variable that can be seen below.
    """
    _port_name = os.environ.get('SERIAL_PORT', 'loop://')

    def test_basics(self):
        print('Testing serial multiprocessing with port', self._port_name)

        from popcop.physical.serial_multiprocessing import Channel, ChannelException

        try:
            Channel('nonexistent-serial-port-name', max_payload_size=1024, frame_timeout=1.0)
        except ChannelException as ex:
            print('^^^^^^ THE ERROR MESSAGE ABOVE IS TOTALLY FINE ^^^^^^')
            print('As expected, we got at exception:', ex)

        with contextlib.closing(Channel(self._port_name, max_payload_size=1024, frame_timeout=1.0)) as channel:
            # Empty read
            r = channel.receive(timeout=0.1)
            self.assertIsNone(r)

            # Send string, read string back
            channel.send_raw('Hello world!')
            iteration_limit = 1000
            received = ''
            while received != 'Hello world!':
                r = channel.receive(timeout=0.1)
                self.assertIsInstance(r, bytes)
                received += r.decode()
                print('Message being reconstructed:', received)
                iteration_limit -= 1
                if iteration_limit <= 0:
                    self.fail('Could not reconstruct the loop back message')

            # Send standard message, read standard message back
            msg = popcop.standard.NodeInfoMessage()
            msg.certificate_of_authenticity = b'such certificate much authenticity'
            msg.globally_unique_id[0] = 123
            msg.node_description = 'NODE DESCRIPTION'
            msg.node_name = 'BOB'
            msg.software_image_crc = 0xDEADBEEFCAFEB00B
            channel.send_standard(msg, timeout=0)
            r = channel.receive(timeout=1)
            print('Received NodeInfo:', r)
            if isinstance(r, popcop.standard.NodeInfoMessage):
                self.assertEqual(r.certificate_of_authenticity, msg.certificate_of_authenticity)
                self.assertEqual(str(r), str(msg))
            else:
                self.fail('No idea')

            # Send application-specific message, read it back
            channel.send_application_specific(123, b'hey this is payload' + bytes(range(256)))
            time.sleep(1)
            r = channel.receive(timeout=0)
            print('Received application specific frame:', r)
            if isinstance(r, popcop.transport.ReceivedFrame):
                self.assertEqual(r.frame_type_code, 123)
                self.assertEqual(r.payload, b'hey this is payload' + bytes(range(256)))
                self.assertLess(r.timestamp, time.monotonic())
                self.assertGreater(r.timestamp + 10, time.monotonic())
            else:
                self.fail('No idea')

    @slow_test
    def test_high_throughput(self):
        # Should be larger than the size of the queue in order to test the case when the queue is full
        num_messages = 40000

        # Use higher baud rate in order to complete the test faster if we're using a real physical port
        with contextlib.closing(popcop.physical.serial_multiprocessing.Channel(self._port_name,
                                                                               max_payload_size=1024,
                                                                               baudrate=230400)) as channel:
            def stephen_king():
                print('Sender worker started')
                for i in range(num_messages):
                    byte_i = i % 256
                    msg = popcop.standard.NodeInfoMessage()
                    msg.certificate_of_authenticity = bytes([byte_i] * byte_i)
                    msg.globally_unique_id[0] = byte_i
                    msg.node_description = str(i)
                    msg.node_name = str(i)
                    msg.software_image_crc = i
                    channel.send_standard(msg, timeout=9999)

                print('Sender worker is finished')

            thd = threading.Thread(target=stephen_king, name='bill_denbrough', daemon=True)
            thd.start()

            for i in range(num_messages):
                byte_i = i % 256

                msg = channel.receive(timeout=1)
                self.assertIsInstance(msg, popcop.standard.NodeInfoMessage)

                self.assertEqual(msg.certificate_of_authenticity, bytes([byte_i] * byte_i))
                self.assertEqual(msg.globally_unique_id, bytes([byte_i, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
                self.assertEqual(msg.node_description, str(i))
                self.assertEqual(msg.node_name, str(i))
                self.assertEqual(msg.software_image_crc, i)

                if byte_i == 0:
                    print('\rReceived %.0f%% (%d/%d) of messages...' %
                          (100 * (i + 1) / num_messages, i + 1, num_messages), end='   \r', flush=True)

            print('\nHigh throughput test succeeded')
            thd.join()

            msg = channel.receive(timeout=1)
            self.assertIsNone(msg)

    @slow_test
    def test_orphan(self):
        import queue

        # Create a normal channel
        ch = popcop.physical.serial_multiprocessing.Channel(self._port_name, max_payload_size=1024)

        # And then simply ask its babysitter to quit. The assertion check is to make sure the field exists.
        self.assertFalse(ch._babysitter_should_quit)
        ch._babysitter_should_quit = True

        # Make sure that the worker is alive, this is its normal state.
        self.assertTrue(ch._proc.is_alive())

        # Wait some time for it to notice that the master is now gone.
        print('Waiting for the IO worker to notice that the master is gone...')
        time.sleep(popcop.physical.serial_multiprocessing.MASTER_HEARTBEAT_TIMEOUT + 3)

        # Show logs manually, since there's now no babysitter to do this
        while True:
            try:
                record = ch._log_queue.get_nowait()
                logging.getLogger(record.name).handle(record)
            except queue.Empty:
                break

        # NO GODS NO MASTERS
        self.assertFalse(ch._proc.is_alive())

    def test_async(self):
        ch = popcop.physical.serial_multiprocessing.AsyncChannel(self._port_name, max_payload_size=1024)

        async def writer():
            # Awaiting is optional. However, if the future is not awaited, thrown exceptions will be lost!
            await ch.send_raw(b'123456')
            await ch.send_application_specific(123, b'Hello world!', timeout=2)
            asyncio.gather(ch.send_standard(popcop.standard.NodeInfoMessage, timeout=3),
                           ch.send_standard(popcop.standard.NodeInfoMessage()))
            print('Writer done')

        async def reader():
            while True:
                item = await ch.receive(1)
                self.assertIsNotNone(item)
                if not isinstance(item, bytes):
                    break
                print(item)

            self.assertIsInstance(item, popcop.transport.ReceivedFrame)
            print(item)
            self.assertEqual(item.frame_type_code, 123)
            self.assertEqual(item.payload, b'Hello world!')

            item = await ch.receive(1)
            self.assertIsInstance(item, popcop.standard.NodeInfoMessage)
            self.assertTrue(item.is_request)
            print(item)

            item = await ch.receive(1)
            self.assertIsInstance(item, popcop.standard.NodeInfoMessage)
            self.assertEqual(str(item), str(popcop.standard.NodeInfoMessage()))
            print(item)

            print('Reader done')

        async def run():
            await asyncio.gather(writer(), reader())
            self.assertTrue(ch.is_open)
            await ch.close()
            self.assertFalse(ch.is_open)

        asyncio.get_event_loop().run_until_complete(run())


if __name__ == '__main__':
    # Use PYTHONASYNCIODEBUG=1 env var to debug asyncio related stuff
    logging.basicConfig(stream=sys.stderr,
                        level='DEBUG',
                        format='%(asctime)s %(levelname)s %(process)d %(name)s: %(message)s')
    unittest.main()
