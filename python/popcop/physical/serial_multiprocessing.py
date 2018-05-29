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
import enum
import queue
import typing
import signal
import asyncio
import warnings
import functools
import threading
import concurrent.futures
import multiprocessing
from logging import getLogger
import logging.handlers
from .. import standard, transport

try:
    import serial
except ImportError:
    serial = None

__all__ = [
    'Channel',
    'ChannelException',
    'ChannelInitializationException',
    'ChannelClosedException',
    'ChannelSendTimeoutException'
]


try:
    # noinspection PyUnresolvedReferences
    sys.getwindowsversion()
    RUNNING_ON_WINDOWS = True
except AttributeError:
    RUNNING_ON_WINDOWS = False

# IPC queue size limits
IPC_RX_QUEUE_SIZE = 32767   # http://stackoverflow.com/questions/5900985/multiprocessing-queue-maxsize-limit-is-32767
IPC_TX_QUEUE_SIZE = IPC_RX_QUEUE_SIZE
IPC_LOG_QUEUE_SIZE = IPC_RX_QUEUE_SIZE

# Defined for POPCOP
DEFAULT_BAUD_RATE = 115200

# If there were no packets from the master process in this amount of time, the master will be considered dead,
# and the IO process will exit automatically.
MASTER_HEARTBEAT_TIMEOUT = 10

# Master will be emitting keep-alive commands at this interval.
MASTER_HEARTBEAT_INTERVAL = 1

# On POSIX systems, the process will attempt to increase its priority by this level. Negative means higher priority.
NICENESS_INCREMENT = -18

# The IO process will terminate if any operation has failed this many times in a row.
# This usually indicates that the serial port became unavailable.
MAX_SUCCESSIVE_ERRORS_TO_GIVE_UP = 100

# Serial port read/write timeout. This has no influence on the application-side interface;
# this parameter merely defines the period of certain recurring tasks in the IO worker process.
IO_TIMEOUT = 0.5

# The amount of time given to the worker process to comply with the STOP command before it will be forcibly terminated.
# Note that the worker may take a long time to process IPC commands (STOP in particular) if there are objects
# in the TX queue - the command will not be seen by the worker until all of the entities from the queue before it
# are processed.
WORKER_PROCESS_JOIN_TIMEOUT = 3


class IPCCommand(enum.Enum):
    KEEP_ALIVE = 0
    STOP = 1


class IPCNotification(enum.Enum):
    INITIALIZATION_COMPLETED = 1


_logger = getLogger('popcop.physical.serial_multiprocessing')


def _try_raise_self_process_priority():
    """
    Attempts to raise the priority level of the current process. Tries several methods depending on the platform;
    does nothing if unsuccessful.
    """
    # https://stackoverflow.com/a/6245096/1007777
    if RUNNING_ON_WINDOWS:
        try:
            # noinspection PyUnresolvedReferences
            import win32api
            # noinspection PyUnresolvedReferences
            import win32process
            # noinspection PyUnresolvedReferences
            import win32con
            handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, win32api.GetCurrentProcessId())
            win32process.SetPriorityClass(handle, win32process.REALTIME_PRIORITY_CLASS)
            _logger.info('Windows process priority changed successfully using the win32 API modules')
        except ImportError:
            # noinspection PyUnresolvedReferences
            import psutil
            psutil.Process(os.getpid()).nice(psutil.REALTIME_PRIORITY_CLASS)
            _logger.info('Windows process priority changed successfully using the psutil module')
    else:
        new_niceness = os.nice(NICENESS_INCREMENT)
        _logger.info('POSIX process has been reniced successfully; new niceness %r', new_niceness)


class IOProcess:
    """
    This class encapsulates the logic of the IO process.
    It should never be used directly; refer to the function _io_process_entry_point() for details.
    """

    def __init__(self,
                 channel,           # this is serial.Serial; we can't annotate it because PySerial may fail to import
                 tx_queue:          multiprocessing.Queue,
                 rx_queue:          multiprocessing.Queue,
                 parent_pid:        int,
                 max_payload_size:  typing.Optional[int],
                 frame_timeout:     typing.Optional[float]):
        self._channel = channel
        self._tx_queue = tx_queue
        self._rx_queue = rx_queue
        self._parent_pid = parent_pid

        self._last_master_heartbeat_at = time.monotonic()
        self._should_exit = False

        self._parser = transport.Parser(callback=self._parser_callback,
                                        max_payload_size=max_payload_size,
                                        frame_timeout=frame_timeout)

        self._rx_thread = threading.Thread(target=self._rx_thread_function,
                                           name='popcop_io_process_rx_thread',
                                           daemon=True)

    def _check_if_should_keep_going(self):
        if (time.monotonic() - self._last_master_heartbeat_at) > MASTER_HEARTBEAT_TIMEOUT:
            # Master appears to be dead, so emitting log messages makes no sense since there would be no one
            # to process them on the other side. So we write directly into stderr.
            print('POPCOP IO WORKER: MASTER HEARTBEAT TIMEOUT', file=sys.stderr)
            return False

        if (not RUNNING_ON_WINDOWS) and (os.getppid() != self._parent_pid):
            print('POPCOP IO WORKER: MASTER IS DEAD. getppid() == %r != %r' % (os.getppid(), self._parent_pid),
                  file=sys.stderr)
            return False

        return not self._should_exit

    # noinspection PyBroadException
    def _parser_callback(self, event):
        # The event object can be either transport.ReceivedFrame or bytes.
        # Here we check if it's a standard frame; if so, decode it and report the decoded object rather than raw frame.
        if isinstance(event, transport.ReceivedFrame):
            if event.frame_type_code == standard.STANDARD_FRAME_TYPE_CODE:
                try:
                    parsed_event = standard.decode(event)
                    if parsed_event is not None:
                        event = parsed_event
                except Exception:
                    # Could not decode - report as is, perhaps the application can make sense of it later.
                    _logger.warning('Could not parse standard frame: %r', event, exc_info=True)

        # At this point, the event object can be either:
        #  - transport.ReceivedFrame
        #  - standard.MessageBase
        #  - bytes
        assert isinstance(event, (transport.ReceivedFrame, standard.MessageBase, bytes))
        self._rx_queue.put(event)

    def _tx_loop(self):
        try:
            m = self._tx_queue.get(timeout=IO_TIMEOUT)
        except queue.Empty:
            return

        if isinstance(m, standard.MessageBase):
            m = standard.encode(m)

        elif isinstance(m, tuple) and len(m) == 2:
            frame_type_code, payload = m
            m = transport.encode(int(frame_type_code), payload)

        elif isinstance(m, (bytes, bytearray)):
            m = m

        elif isinstance(m, str):
            m = m.encode('utf8')

        elif isinstance(m, IPCCommand):
            if m == IPCCommand.KEEP_ALIVE:
                pass
            elif m == IPCCommand.STOP:
                self._should_exit = True
                _logger.info('Master wants us to quit. Yes master.')
            else:
                raise ValueError('Unknown IPC command: %r' % m)

            m = bytes()

        else:
            raise TypeError("Don't know how to emit object: %r" % m)

        self._last_master_heartbeat_at = time.monotonic()

        assert isinstance(m, (bytes, bytearray))
        if len(m) > 0:
            self._channel.write(m)

    def _rx_thread_function(self):
        try:
            while self._check_if_should_keep_going():
                chunk = self._channel.read(max(1, self._channel.in_waiting))
                # We have to invoke parse() even if we received zero bytes in order to run timing constraint checks.
                self._parser.parse(chunk, time.monotonic())
        except Exception as ex:
            _logger.error('IO process RX thread failure: %s', ex, exc_info=True)
            self._rx_queue.put(ex)
        finally:
            self._should_exit = True

    def run(self):
        try:
            self._rx_thread.start()
            self._rx_queue.put(IPCNotification.INITIALIZATION_COMPLETED)
            while self._check_if_should_keep_going() and self._rx_thread.is_alive():
                try:
                    self._tx_loop()
                except Exception as ex:
                    _logger.error('IO process TX loop error: %s', ex, exc_info=True)
                    self._rx_queue.put(ex)
        finally:
            self._should_exit = True
            if self._rx_thread.is_alive():
                self._rx_thread.join(10)


# noinspection PyBroadException
def _io_process_entry_point(port_name:          str,
                            tx_queue:           multiprocessing.Queue,
                            rx_queue:           multiprocessing.Queue,
                            log_queue:          multiprocessing.Queue,
                            parent_pid:         int,
                            baudrate:           int,
                            max_payload_size:   typing.Optional[int],
                            frame_timeout:      typing.Optional[float]):
    """
    The entry point of the IO process.

    :param port_name:       Name of the serial port to work with or its URL.
                            See https://pythonhosted.org/pyserial/url_handlers.html#urls for details.

    :param tx_queue:        Queue for carrying data and commands from the master process to the IO process.

    :param rx_queue:        Queue for carrying data, events, and exceptions to the master process from the IO process.

    :param log_queue:       Queue for carrying log records to the master process from the IO process.

    :param parent_pid:      PID of the master process. Used for detecting when the master process dies.

    :param baudrate:        Which baud rate to use. The standard is 115200. This is only applicable to physical
                            serial links; virtual serial links and URLs typically ignore this parameter.

    :param max_payload_size: See transport.Parser.

    :param frame_timeout:   See transport.Parser.
    """
    # First of all, set up logging. If something goes wrong, we need to know what exactly did so.
    getLogger().addHandler(logging.handlers.QueueHandler(log_queue))
    getLogger().setLevel('INFO')

    _logger.info('IO worker process started with PID %r, parent %r; initializing...', os.getpid(), parent_pid)

    # Set up the environment. We don't need stdin, close it so that it doesn't interfere with command-line applications.
    try:
        stdin_fileno = sys.stdin.fileno()
        sys.stdin.close()
        os.close(stdin_fileno)
    except Exception:
        pass

    # Attempt to raise the priority of the process. In many cases it may fail, which is normal.
    try:
        _try_raise_self_process_priority()
    except Exception as ex:
        _logger.info('IO worker process will be operating at the default priority level (reason: %r)', ex)

    # Now the environment is set up, open the specified serial port.
    try:
        channel = serial.serial_for_url(str(port_name),
                                        baudrate=int(baudrate),
                                        timeout=IO_TIMEOUT,
                                        write_timeout=IO_TIMEOUT)
    except Exception as ex:
        _logger.error('Could not open serial port %r: %s', port_name, ex, exc_info=True)
        rx_queue.put(ex)
        return

    # Now everything is set up, construct the runner and run it. Don't forget to close the channel explicitly at exit.
    try:
        _logger.info('Ready to run. Port %r', port_name)
        run_lola = IOProcess(channel=channel,
                             tx_queue=tx_queue,
                             rx_queue=rx_queue,
                             parent_pid=parent_pid,
                             max_payload_size=max_payload_size,
                             frame_timeout=frame_timeout)
        run_lola.run()
    except Exception as ex:
        _logger.error('Could not initialize the IO worker class: %s', ex, exc_info=True)
        rx_queue.put(ex)
    finally:
        channel.close()
        _logger.info('IO tootaja on peatunud. Head aega!')


class ChannelException(Exception):
    """Base exception type for channel errors."""
    pass


class ChannelInitializationException(ChannelException):
    """Thrown from the channel constructor."""
    pass


class ChannelClosedException(ChannelException):
    """Raised when attempting to read from or write to a closed channel."""
    pass


class ChannelSendTimeoutException(ChannelException):
    """Raised when a channel send operation times out."""
    pass


class Channel:
    """
    This class provides a simple interface that allows the user to exchange arbitrary data
    over a POPCOP channel over a serial port. Parsing, encoding, serialization and deserialization
    are performed in a dedicated high-priority process, completely separate from the process of the caller.
    This allows the library to take advantage of multicore systems, performing all of the real-time
    protocol processing concurrently. Things would be much simpler if not for the bloody GIL.
    """

    def __init__(self,
                 port_name: str,
                 baudrate: int=None,
                 max_payload_size: typing.Optional[int]=None,
                 frame_timeout: typing.Optional[float]=None):
        """
        :param port_name:       Name of the serial port to work with, or its URL.
                                Names are platform-specific, e.g.
                                /dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_1A002B00185732523935382000000000-if00
                                or /dev/ttyUSB0, or COM3.
                                URLs are defined by PySerial: https://pythonhosted.org/pyserial/url_handlers.html#urls
                                for example:
                                loop://
                                spy://COM54?file=log.txt

        :param baudrate:        Which baud rate to use. Default is 115200. This is only applicable to physical
                                serial links; virtual serial links and URLs typically ignore this parameter.

        :param max_payload_size: See transport.Parser.

        :param frame_timeout:   See transport.Parser.
        """
        if serial is None:
            raise ImportError('PySerial is not available on this system. '
                              'Please install it to be able to use this class.')

        if (max_payload_size is None) and (frame_timeout is None):
            raise ValueError('Either max_payload_size or frame_timeout must be set')

        # Windows doesn't support forking.
        self._mp_context = multiprocessing.get_context('spawn')

        self._tx_queue = self._mp_context.Queue(maxsize=IPC_RX_QUEUE_SIZE)
        self._rx_queue = self._mp_context.Queue(maxsize=IPC_TX_QUEUE_SIZE)
        self._log_queue = self._mp_context.Queue(maxsize=IPC_LOG_QUEUE_SIZE)

        kwargs = dict(port_name=str(port_name),
                      tx_queue=self._tx_queue,
                      rx_queue=self._rx_queue,
                      log_queue=self._log_queue,
                      parent_pid=os.getpid(),
                      baudrate=int(baudrate or DEFAULT_BAUD_RATE),
                      max_payload_size=max_payload_size,
                      frame_timeout=frame_timeout)

        self._proc = self._mp_context.Process(target=_io_process_entry_point,
                                              name='popcop_io_worker',
                                              daemon=True,
                                              kwargs=kwargs)

        # We need this to tell babysitter to abandon the patron if we can't terminate it properly.
        self._babysitter_should_quit = False

        self._babysitter_thread = threading.Thread(target=self._babysitter_thread_entry_point,
                                                   name='popcop_io_worker_babysitter',
                                                   daemon=True)

        # Woohoo lift-off
        self._proc.start()
        self._babysitter_thread.start()
        _logger.info('Started IO worker process with PID %r; waiting for initialization...', self._proc.pid)
        init_deadline = time.monotonic() + MASTER_HEARTBEAT_TIMEOUT

        try:
            while True:
                if not self._proc.is_alive():
                    raise ChannelInitializationException('The IO worker process could not be launched')

                if time.monotonic() > init_deadline:
                    raise ChannelInitializationException('The IO worker process initialization has timed out')

                try:
                    out = self._rx_queue.get(timeout=IO_TIMEOUT)
                except queue.Empty:
                    continue

                if isinstance(out, Exception):
                    raise ChannelInitializationException('IO worker initialization failed: %s' % out) from out
                elif isinstance(out, IPCNotification) and (out == IPCNotification.INITIALIZATION_COMPLETED):
                    break
                else:
                    _logger.error('Unexpected response while waiting for initialization confirmation: %r', out)
        except Exception:
            self.close()
            raise

        _logger.info('IO worker process with PID %r initialized successfully', self._proc.pid)

    def __del__(self):
        if self._proc.is_alive():
            warnings.warn('Oh no! The channel is being garbage collected while still open!', RuntimeWarning)

        self.close()

    def _babysitter_thread_entry_point(self):
        try:
            while self._proc.is_alive() and not self._babysitter_should_quit:
                try:
                    record = self._log_queue.get(timeout=MASTER_HEARTBEAT_INTERVAL)
                except queue.Empty:
                    pass
                else:
                    getLogger(record.name).handle(record)

                self._tx_queue.put(IPCCommand.KEEP_ALIVE)
        except Exception as ex:
            _logger.critical('Unhandled exception in the IO worker babysitter thread. '
                             'The babysitter will stop, but the IO worker will continue to run unattended. '
                             'Exception: %s', ex, exc_info=True)
            # Note that we don't do anything special here.
            # We don't ask the worker to stop or anything, we just silently die.
            # The functions of babysitter are not mission-critical, so we don't drag down the rest of the logic.
        else:
            _logger.info('Babysitter is exiting normally. IO worker process is alive: %r, explicit quit request: %r',
                         self._proc.is_alive(), self._babysitter_should_quit)

    def close(self):
        if self._proc.is_alive():
            _logger.info('Stopping the IO worker process with PID %r...', self._proc.pid)

            # Note that if there is a lot of stuff in the TX queue, it may take a while for the worker
            # to get to our STOP command, so the following join() may timeout even if the worker is
            # perfectly functional. This is not really a serious problem, but something perhaps worth
            # looking into someday.
            self._tx_queue.put(IPCCommand.STOP)
            self._proc.join(WORKER_PROCESS_JOIN_TIMEOUT)

            if self._proc.is_alive() or self._proc.exitcode is None:
                _logger.warning('The worker did not stop and will be terminated')
                try:
                    self._proc.terminate()
                except Exception as ex:
                    _logger.error('Failed to terminate the worker [%r]', ex, exc_info=True)

                self._proc.join(1)      # The delay here can be chosen rather arbitrarily.
                try:
                    if self._proc.is_alive():
                        _logger.error('The worker could not be terminated so it will be killed now')
                        os.kill(self._proc.pid, signal.SIGKILL)
                except Exception as ex:
                    _logger.error('Failed to kill the worker [%r]. Hey worker, you suck.', ex, exc_info=True)

            if not self._proc.is_alive():
                _logger.info('IO worker process with PID %r stopped successfully. '
                             'Now waiting for the babysitter thread to join too.',
                             self._proc.pid)

                self._babysitter_should_quit = True
                if self._babysitter_thread.is_alive():  # BABY JOIN ME IN DEATH
                    self._babysitter_thread.join()      # THIS I/O PROCESS MANAGEMENT AIN'T WORTH LIVING

                _logger.info("Babysitter has joined, we're all set.")
            else:
                _logger.critical('IO worker process with PID %r is still alive!', self._proc.pid)

    @property
    def is_open(self):
        return self._proc.is_alive()

    def _do_send(self, entity, timeout):
        if not self.is_open:
            raise ChannelClosedException('Cannot send to the channel because it is not open')

        if timeout is None:
            timeout = 0.0
        else:
            timeout = float(timeout)

        try:
            # Zero and near-zero timeouts go here because put() with zero timeout always throws queue.Empty.
            if timeout < 1e-6:
                self._tx_queue.put_nowait(entity)
            else:
                self._tx_queue.put(entity, timeout=timeout)
        except queue.Full:
            raise ChannelSendTimeoutException('Channel send operation has timed out; timeout: %r, entity: %r',
                                              timeout, entity) from None

    def send_standard(self,
                      msg_or_type,
                      timeout: typing.Optional[float]=None):
        """
        :param msg_or_type: Message to transmit, or its class if the intention is to request the message from the
                            endpoint. Note that the message will be encoded in the IO worker process.
        :param timeout:     Optional timeout in seconds. None or zero for non-blocking operation (this is the default).
        """
        if isinstance(msg_or_type, standard.MessageBase):
            self._do_send(msg_or_type, timeout)
        elif hasattr(msg_or_type, 'MESSAGE_ID') and isinstance(msg_or_type.MESSAGE_ID, int):
            self._do_send((standard.STANDARD_FRAME_TYPE_CODE, standard.encode_header(msg_or_type)), timeout)
        else:
            raise TypeError('This is not a standard message nor its type: %r' % type(msg_or_type))

    def send_application_specific(self,
                                  frame_type_code: int,
                                  payload: typing.Union[bytes, bytearray],
                                  timeout: typing.Optional[float]=None):
        """
        :param frame_type_code:   Valid values are 0..255.
        :param payload:     Bytes or bytearray with the payload.
        :param timeout:     Optional timeout in seconds. None or zero for non-blocking operation (this is the default).
        """
        frame_type_code = int(frame_type_code)
        payload = bytes(payload)
        if not (0 <= frame_type_code <= 0xFF):
            raise ValueError('Invalid frame type code: %r' % frame_type_code)

        self._do_send((frame_type_code, payload), timeout)

    def send_raw(self,
                 bytes_or_string: typing.Union[bytes, bytearray, str],
                 timeout: typing.Optional[float]=None):
        """
        :param bytes_or_string: Raw data to emit; as bytes, bytearray, or string. String will be encoded in UTF-8.
        :param timeout:     Optional timeout in seconds. None or zero for non-blocking operation (this is the default).
        """
        if not isinstance(bytes_or_string, (bytes, bytearray)):
            bytes_or_string = bytes_or_string.encode('utf8')

        self._do_send(bytes_or_string, timeout)

    def receive(self,
                timeout: typing.Optional[float]=None) -> typing.Optional[typing.Union[transport.ReceivedFrame,
                                                                                      standard.MessageBase,
                                                                                      bytes]]:
        """
        Reads one entity from the RX queue and returns it. Returns None on timeout.
        This method may throw ChannelException() containing exceptions originating in the IO process.
        The called must take care to handle them.

        :param timeout: Optional timeout in seconds. If not provided, the operation will be non-blocking.

        :return: One of the following:
                  - transport.ReceivedFrame (where the timestamp is set in the time base of time.monotonic())
                  - standard.MessageBase
                  - bytes
                  - None (if timed out, or if the queue is empty and the operation is non-blocking)
        """
        if not self.is_open:
            raise ChannelClosedException('Cannot receive from the channel because it is not open')

        try:
            if (timeout is None) or (timeout < 1e-6):
                # Zero and near-zero timeouts go here because get() with zero timeout always throws queue.Empty.
                obj = self._rx_queue.get_nowait()
            else:
                obj = self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return
        else:
            if isinstance(obj, Exception):
                raise ChannelException('IO worker process has encountered an unhandled exception: %s' % obj) from obj

            assert isinstance(obj, (transport.ReceivedFrame, standard.MessageBase, bytes))
            return obj


class AsyncChannel:
    """
    This is like Channel, but with an asynchronous interface.
    """

    def __init__(self, *args, **kwargs):
        self._impl = Channel(*args, **kwargs)
        self._executor = concurrent.futures.ThreadPoolExecutor()

    def _execute(self, what, *args, **kwargs) -> typing.Awaitable:
        return asyncio.get_event_loop().run_in_executor(self._executor, functools.partial(what, *args, **kwargs))

    def send_standard(self, *args, **kwargs) -> typing.Awaitable:
        """
        Returns a future that resolves to None.
        See Channel.send_standard()
        """
        return self._execute(self._impl.send_standard, *args, **kwargs)

    def send_application_specific(self, *args, **kwargs) -> typing.Awaitable:
        """
        Returns a future that resolves to None.
        See Channel.send_application_specific()
        """
        return self._execute(self._impl.send_application_specific, *args, **kwargs)

    def send_raw(self, *args, **kwargs) -> typing.Awaitable:
        """
        Returns a future that resolves to None.
        See Channel.send_raw()
        """
        return self._execute(self._impl.send_raw, *args, **kwargs)

    def receive(self, timeout: float) -> typing.Awaitable[typing.Optional[typing.Union[transport.ReceivedFrame,
                                                                                       standard.MessageBase,
                                                                                       bytes]]]:
        """
        Returns a future that resolves either to a received item or None in the case of a timeout.
        FIXME: Currently this method is unsafe for use with asyncio.wait_for() and similar cancelling methods.
        """
        timeout = float(timeout)
        if timeout <= 0:
            raise ValueError('A positive timeout is required')

        # TODO: If the operation is cancelled via a timeout exception (e.g. asyncio.wait_for()),
        # TODO: the worker thread will be left unattended while trying to read the channel.
        # TODO: This can create all sorts of bugs, e.g. messages read by an abandoned worked thread that can't
        # TODO: be passed anywhere because the worker thread is abandoned.
        # TODO: See this for more info https://stackoverflow.com/a/34457515/1007777
        return self._execute(self._impl.receive, timeout)

    async def close(self):
        await self._execute(self._impl.close)
        self._executor.shutdown(wait=False)

    @property
    def is_open(self):
        return self._impl.is_open
