'''
Defines the Engine

General notes for developers
-----------------------------------------------------------------------------
This is a wraper around the NI-DAQmx C API. Refer to the NI-DAQmx C reference
(available as a Windows help file or as HTML documentation on the NI website).
Google can help you quicly find the online documentation).

This code is under heavy development and I may change the API in significant
ways. In general, the only portion of the code you should use in third-party
modules is the `Engine` class. This will serve as the sole communication layer
between the NI hardware and your application. By doing so, this ensures a
sufficient layer of abstraction that helps switch between DAQ hardware from
different vendors (including Measurement Computing and OpenElec).

Some parts of the code takes advantage of generators and coroutines. For details
on this topic, see the following resources:

    http://www.dabeaz.com/coroutines/
    http://www.python.org/dev/peps/pep-0342/

'''

from __future__ import print_function

from collections import OrderedDict
import ctypes

import numpy as np
#import PyDAQmx as mx

import logging
log = logging.getLogger(__name__)


################################################################################
# utility
################################################################################
def read_digital_lines(task, size=1):
    nlines = ctypes.c_uint32()
    mx.DAQmxGetDINumLines(task, '', nlines)
    nsamp = ctypes.c_int32()
    nbytes = ctypes.c_int32()
    data = np.empty(nlines.value*size, dtype=np.uint8)
    mx.DAQmxReadDigitalLines(task, size, 0, mx.DAQmx_Val_GroupByChannel, data,
                             len(data), nsamp, nbytes, None)
    return data


def constant_lookup(value):
    for name in dir(mx.DAQmxConstants):
        if name in mx.DAQmxConstants.constant_list:
            if getattr(mx.DAQmxConstants, name) == value:
                return name
    raise ValueError('Constant {} does not exist'.format(value))


def channel_list(channels, channel_type):
    task = create_task()
    if channel_type == 'digital':
        mx.DAQmxCreateDIChan(task, channels, '', mx.DAQmx_Val_ChanPerLine)
    elif channel_type == 'ao':
        mx.DAQmxCreateAOVoltageChan(task, channels, '', -10, 10,
                                    mx.DAQmx_Val_Volts, '')

    data = ctypes.create_string_buffer('', 1024)
    mx.DAQmxGetTaskChannels(task, data, len(data))
    mx.DAQmxClearTask(task)
    return [c.strip() for c in data.value.split(',')]


def get_name_map(lines, names):
    channels = channel_list(lines, 'digital')
    if len(channels) != len(names):
        m = 'Number of names must match number of lines'
        raise ValueError(m)
    return dict(zip(channels, names))


class CallbackHelper(object):

    def __init__(self, callback):
        self._callback = callback

    def __call__(self, task, *args):
        try:
            result = self._callback(task)
            if result is None:
                return 0
            else:
                return result
        except:
            raise
            return -1


class SamplesGeneratedCallbackHelper(object):

    def __init__(self, callback, n_channels):
        self._callback = callback
        self._n_channels = n_channels
        self._uint64 = ctypes.c_uint64()
        self._uint32 = ctypes.c_uint32()

    def __call__(self, task, event_type, callback_samples, callback_data):
        try:
            mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_CurrWritePos)
            mx.DAQmxSetWriteOffset(task, 0)
            mx.DAQmxGetWriteCurrWritePos(task, self._uint64)
            log.trace('Current write position %d', self._uint64.value)
            mx.DAQmxGetWriteTotalSampPerChanGenerated(task, self._uint64)
            log.trace('Total s/chan generated %d', self._uint64.value)
            mx.DAQmxGetWriteSpaceAvail(task, self._uint32)
            log.trace('Current write space available %d', self._uint32.value)
            if self._uint32.value != 0:
                self._callback(self._uint32.value)
            return 0
        except Exception as e:
            log.exception(e)
            return -1


class SamplesAcquiredCallbackHelper(object):

    def __init__(self, callback, n_channels):
        self._callback = callback
        self._n_channels = n_channels
        self._samples_read_ptr = ctypes.c_int32()

    def __call__(self, task, event_type, callback_samples, callback_data):
        try:
            data = np.empty((self._n_channels, callback_samples))
            mx.DAQmxReadAnalogF64(task, callback_samples, 0,
                                  mx.DAQmx_Val_GroupByChannel, data, data.size,
                                  self._samples_read_ptr, None)
            self._callback(data)
            return 0
        except Exception as e:
            log.exception(e)
            return -1


class ChangeDetectionCallbackHelper(object):

    def __init__(self, lines, callback, initial_state, timer_task):
        self._callback = callback
        self._prev_state = initial_state
        self._lines = lines
        self._timer_task = timer_task

    def _get_event_time(self):
        # Read the value stored
        data = np.empty(100, dtype=np.double)
        result = ctypes.c_int32()
        mx.DAQmxReadCounterF64(self._timer_task, -1, 0, data, len(data), result,
                               None)
        data = data[:result.value]
        if len(data) != 1:
            raise SystemError('Too many event times acquired')
        return data[0]

    def __call__(self, task, *args):
        try:
            current_state = read_digital_lines(task)
            et = self._get_event_time()
            for (l, p, c) in zip(self._lines, self._prev_state, current_state):
                if p != c:
                    edge = 'rising' if p == 0 else 'falling'
                    self._callback(edge, l, et)
            self._prev_state = current_state
            return 0
        except Exception as e:
            log.exception(e)
            return -1


def coroutine(func):
    '''Decorator to auto-start a coroutine.'''
    def start(*args, **kwargs):
        cr = func(*args, **kwargs)
        cr.next()
        return cr
    return start


@coroutine
def capture_epoch(uuid, start, duration, callback):
    '''
    Coroutine to facilitate epoch acquisition
    '''
    accumulated_data = []
    while True:
        tlb, data = (yield)
        samples = data.shape[-1]
        if start < tlb:
            # We have missed the start of the epoch. Notify the callback of this
            # problem.
            callback(uuid, None)
            break
        elif start <= (tlb + samples):
            # The start of the epoch is somewhere inside `data`. Find the start
            # `i` and determine how many samples `d` to extract from `data`.
            # It's possible that data does not contain the entire epoch. In that
            # case, we just pull out what we can and save it in
            # `accumulated_data`. We then update start to point to the last
            # acquired sample `i+d` and update duration to be the number of
            # samples we still need to capture.
            i = start-tlb
            d = min(duration, samples)
            accumulated_data.append(data[i:i+d])
            start = i+d
            duration -= d

            # Check to see if we've finished acquiring the entire epoch. If so,
            # send it to the callback. Also, do a sanity check on the parameters
            # (duration < 0 means that something very bad happened).
            if duration == 0:
                accumulated_data = np.concatenate(accumulated_data, axis=-1)
                callback(uuid, accumulated_data)
                break
            elif duration < 0:
                callback(uuid, None)
                break


@coroutine
def extract_epochs(queue, callback):
    # The variable `tlb` tracks the number of samples that have been acquired
    # and reflects the lower bound of `data`. For example, if we have acquired
    # 300,000 samples, then the next chunk of data received from (yield) will
    # start at sample 300,000 (remember that Python is zero-based indexing, so
    # the first sample has an index of 0).
    tlb = 0
    epoch_coroutines = []
    while True:
        # Wait for new data to become available
        data = (yield)

        # Check to see if more epochs have been requested
        while not queue.empty():
            uuid, start, duration = queue.get()
            epoch_coroutine = capture_epoch(uuid, start, duration, callback)
            epoch_coroutines.append(epoch_coroutine)

        # Send the data to each coroutine. If a StopIteration occurs, this means
        # that the epoch has successfully been acquired and has been sent to the
        # callback and we can remove it. Need to operate on a copy of list since
        # it's bad form to modify a list in-place.
        for epoch_coroutine in epoch_coroutines[:]:
            try:
                epoch_coroutine.send((tlb, data))
            except StopIteration:
                epoch_coroutines.remove(epoch_coroutine)

        tlb = tlb + data.shape[-1]


################################################################################
# configuration
################################################################################
def create_task(name=None):
    '''
    Create niDAQmx task

    Parameters
    ----------
    name : {None, str}
        Task name (optional). Primarily useful only for debugging purposes
        (e.g., this is what's reported in NI error messages)

    Returns
    -------
    task : ctypes pointer
        Pointer to niDAQmx task
    '''
    if name is None:
        name = ''
    task = mx.TaskHandle(0)
    mx.DAQmxCreateTask(name, ctypes.byref(task))
    return task


def setup_event_timer(trigger, counter, clock, callback=None, edge='rising'):
    '''
    Create timer to store timestamp of rising edge of trigger. Useful for
    tracking when changes to digital state occur in high precision.

    Parameters
    ----------
    trigger : str
        Line to monitor for digital edge (e.g., '/Dev1/PFI1')
    counter : str
        Which counter channel to use (e.g., '/Dev1/Ctr0')
    clock : str
        Timebase for counter.  The value read from the counter will be in units
        of the specified clock (e.g., 'ao/SampleClock').
    edge : {'rising', 'falling'}
        Should an event time be acquired on the rising or falling edge of the
        sample clock?

    Returns
    -------
    task : niDAQmx task
        Task configured to acquire event time.
    '''
    task = create_task()
    if edge == 'rising':
        edge_value = mx.DAQmx_Val_Rising
    elif edge == 'falling':
        edge_value = mx.DAQmx_Val_Falling
    else:
        raise ValueError('Unsupported mode, {}, for edge'.format(edge))

    # Set up a counter to count the number of rising edges from an input
    # terminal. The input terminal will be set to the value specified by
    # `clock`. Example terminals include the `ao/SampClock` (in which case the
    # counter will tell us how many samples have been generated by the analog
    # output) or the `10MHzRefClock` (in which case the counter will tell us the
    # the time elapsed since the task began with an accuracy of 1/10e6 seconds).
    # Every time a trigger is detected, the value of the counter will be saved
    # to a buffer and can be read into Python.
    mx.DAQmxCreateCICountEdgesChan(task, counter, '', mx.DAQmx_Val_Rising, 0,
                                   mx.DAQmx_Val_CountUp)
    mx.DAQmxSetCICountEdgesTerm(task, counter, clock)
    mx.DAQmxCfgSampClkTiming(task, trigger, 1, edge_value,
                             mx.DAQmx_Val_ContSamps, 500)

    if callback is not None:
        cb = CallbackHelper(callback)

        # Create the callback. Be sure to store a reference to the callback
        # pointer on the task object otherwise the pointer will be
        # garbage-collected after this function exits. If the pointer is
        # garbage-collected, then the callback no longer exists and the program
        # will segfault when niDAQmx tries to call it.
        cb_ptr = mx.DAQmxSignalEventCallbackPtr(cb)
        mx.DAQmxRegisterSignalEvent(task, mx.DAQmx_Val_SampleCompleteEvent, 0,
                                    cb_ptr, None)
        task.__se_cb_ptr = cb_ptr

    mx.DAQmxTaskControl(task, mx.DAQmx_Val_Task_Commit)
    return task


def setup_change_detect_callback(lines, callback, timer, names=None):
    # Since the user may have specified a range of lines (e.g.,
    # 'Dev2/port0/line0:1') or listed each line individually (e.g.,
    # 'Dev2/port0/line0, Dev2/port0/line1'), we need to extract the individual
    # lines in the task. This allows the callback helper to report the name of
    # the line on which the event was detected.
    lines = channel_list(lines, 'digital')
    if names is not None:
        if len(names) != len(lines):
            raise ValueError('Number of names must match number of lines')
    else:
        names = lines

    task = create_task()
    line_string = ','.join(lines)
    mx.DAQmxCreateDIChan(task, line_string, '', mx.DAQmx_Val_ChanForAllLines)

    # Get the current state of the lines so that we know what happened during
    # the first change detection event.
    mx.DAQmxStartTask(task)
    initial_state = read_digital_lines(task, 1)
    mx.DAQmxStopTask(task)

    # If we're using change detection timing on the digital lines, there's no
    # point in reading them in as a hardware-timed buffered task. Given the
    # event times, we can always reconstruct the state of the line at any given
    # time.
    mx.DAQmxCfgChangeDetectionTiming(task, line_string, line_string,
                                     mx.DAQmx_Val_ContSamps, 200)
    mx.DAQmxCfgInputBuffer(task, 0)

    cb = ChangeDetectionCallbackHelper(names, callback, initial_state, timer)
    cb_ptr = mx.DAQmxSignalEventCallbackPtr(cb)
    mx.DAQmxRegisterSignalEvent(task, mx.DAQmx_Val_ChangeDetectionEvent, 0,
                                cb_ptr, None)
    task._cb_ptr = cb_ptr

    mx.DAQmxTaskControl(task, mx.DAQmx_Val_Task_Commit)
    return task


def setup_hw_ao(fs, lines, expected_range, callback, callback_samples):
    # TODO: DAQmxSetAOTermCfg
    task = create_task()
    lb, ub = expected_range
    mx.DAQmxCreateAOVoltageChan(task, lines, '', lb, ub, mx.DAQmx_Val_Volts, '')
    mx.DAQmxCfgSampClkTiming(task, '', fs, mx.DAQmx_Val_Rising,
                             mx.DAQmx_Val_ContSamps, int(fs))

    # This controls how quickly we can update the buffer on the device.
    mx.DAQmxSetBufOutputOnbrdBufSize(task, 100)

    # If the write reaches the end of the buffer and no new data has been
    # provided, do not loop around to the beginning and start over.
    mx.DAQmxSetWriteRegenMode(task, mx.DAQmx_Val_DoNotAllowRegen)

    result = ctypes.c_uint32()
    mx.DAQmxGetTaskNumChans(task, result)
    n_channels = result.value

    callback_helper = SamplesGeneratedCallbackHelper(callback, n_channels)
    cb_ptr = mx.DAQmxEveryNSamplesEventCallbackPtr(callback_helper)
    mx.DAQmxRegisterEveryNSamplesEvent(task,
                                       mx.DAQmx_Val_Transferred_From_Buffer,
                                       int(callback_samples), 0, cb_ptr, None)
    task._cb_ptr = cb_ptr
    return task


def setup_hw_ai(fs, lines, expected_range, callback, callback_samples, sync_ao):
    # Record AI filter delay
    task = create_task()
    lb, ub = expected_range
    mx.DAQmxCreateAIVoltageChan(task, lines, '', mx.DAQmx_Val_RSE, lb, ub,
                                mx.DAQmx_Val_Volts, '')

    if sync_ao:
        mx.DAQmxCfgDigEdgeStartTrig(task, 'ao/StartTrigger',
                                    mx.DAQmx_Val_Rising)
        mx.DAQmxCfgSampClkTiming(task, 'ao/SampleClock', fs,
                                 mx.DAQmx_Val_Rising, mx.DAQmx_Val_ContSamps,
                                 int(fs))
    else:
        mx.DAQmxCfgSampClkTiming(task, '', fs, mx.DAQmx_Val_Rising,
                                 mx.DAQmx_Val_ContSamps, int(fs))

    result = ctypes.c_uint32()
    mx.DAQmxGetTaskNumChans(task, result)
    n_channels = result.value

    callback_helper = SamplesAcquiredCallbackHelper(callback, n_channels)
    cb_ptr = mx.DAQmxEveryNSamplesEventCallbackPtr(callback_helper)
    mx.DAQmxRegisterEveryNSamplesEvent(task, mx.DAQmx_Val_Acquired_Into_Buffer,
                                       int(callback_samples), 0, cb_ptr, None)
    task._cb_ptr = cb_ptr
    return task


def setup_sw_ao(lines, expected_range):
    # TODO: DAQmxSetAOTermCfg
    task = create_task()
    lb, ub = expected_range
    mx.DAQmxCreateAOVoltageChan(task, lines, '', lb, ub, mx.DAQmx_Val_Volts, '')
    return task


def setup_sw_do(lines):
    task = create_task()
    mx.DAQmxCreateDOChan(task, lines, '', mx.DAQmx_Val_ChanForAllLines)
    return task


################################################################################
# engine
################################################################################
class Engine(object):
    '''
    Hardware interface

    The tasks are started in the order they are configured. Most NI devices can
    only support a single hardware-timed task of a specified type (e.g., analog
    input, analog output, digital input, digital output are all unique task
    types).
    '''
    hw_ao_monitor_period = 1
    hw_ai_monitor_period = 0.25
    hw_ai_buffer_duration = 60


    def __init__(self):
        # Use an OrderedDict to ensure that when we loop through the tasks
        # stored in the dictionary, we process them in the order they were
        # configured.
        self._tasks = OrderedDict()
        self._callbacks = {}
        self._timers = {}

        # These are pointers to C datatypes that are required for communicating
        # with the NI-DAQmx library. When querying various properties of tasks,
        # channels and buffers, the NI-DAQmx function often requires an integer
        # of a specific type (e.g. unsigned 32-bit, unsigned 64-bit, etc.). This
        # integer must be passed by reference, allowing the NI-DAQmx function to
        # modify the value directly. For example:
        #
        #     mx.DAQmxGetWriteSpaceAvail(task, self._uint32)
        #     print(self._uint32.value)
        #
        # The ctypes library facilitates communicating with the NI-DAQmx C-API
        # by providing wrappers around C datatypes that can be passed by
        # reference.
        self._uint32 = ctypes.c_uint32()
        self._uint64 = ctypes.c_uint64()
        self._int32 = ctypes.c_int32()

    def configure_hw_ao(self, fs, lines, expected_range):
        '''
        Initialize hardware-timed analog output

        Parameters
        ----------
        fs : float
            Sampling frequency of output (e.g., 100e3).
        lines : str
            Analog output lines to use (e.g., 'Dev2/ao0:4' to specify a range of
            lines or 'Dev2/ao0,Dev2/ao4' to specify specific lines).
        expected_range : (float, float)
            Tuple of upper/lower end of expected range. The maximum range
            allowed by most NI devices is (-10, 10). Some devices (especially
            newer ones) will optimize the output resolution based on the
            expected range of the signal.
        '''
        task = setup_hw_ao(fs, lines, expected_range, self._samples_needed,
                           int(self.hw_ao_monitor_period*fs))
        self._tasks['hw_ao'] = task

    def configure_hw_ai(self, fs, lines, expected_range, sync_ao=True):
        task = setup_hw_ai(fs, lines, expected_range, self._samples_acquired,
                           int(self.hw_ai_monitor_period*fs), sync_ao)
        task._fs = fs
        self._tasks['hw_ai'] = task

    def configure_sw_ao(self, lines, expected_range, names=None,
                        initial_state=None):
        lines = channel_list(lines, 'ao')
        if names is not None:
            if len(lines) != len(names):
                raise ValueError('Number of names must match number of lines')
        else:
            names = lines
        if initial_state is None:
            initial_state = np.zeros(len(names), dtype=np.double)

        task = setup_sw_ao(','.join(lines), expected_range)
        task._names = names
        self._tasks['sw_ao'] = task
        self.write_sw_ao(initial_state)

    def configure_hw_di(self, fs, lines):
        pass

    def configure_hw_do(self, fs, lines):
        pass

    def configure_sw_do(self, lines, names=None, initial_state=None):
        lines = channel_list(lines, 'digital')
        if names is not None:
            if len(lines) != len(names):
                raise ValueError('Number of names must match number of lines')
        else:
            names = lines

        if initial_state is None:
            initial_state = np.zeros(len(names), dtype=np.uint8)

        task = setup_sw_do(','.join(lines))
        task._names = names
        self._tasks['sw_do'] = task
        self.write_sw_do(initial_state)

    def configure_et(self, lines, clock, names=None):
        # TODO - We need to make this a bit smarter about which device we're
        # using. It shoudln't hard-code the device (or the counter in case we
        # want to use a different counter).
        et_task = setup_event_timer('/Dev2/ChangeDetectionEvent', '/Dev2/Ctr0',
                                    clock)
        cd_task = setup_change_detect_callback(lines, self._et_fired, et_task,
                                               names)
        self._tasks['et_task'] = et_task
        self._tasks['cd_task'] = cd_task

    def register_ao_callback(self, callback):
        self._callbacks.setdefault('ao', []).append(callback)

    def register_ai_callback(self, callback):
        self._callbacks.setdefault('ai', []).append(callback)

    def register_et_callback(self, callback):
        self._callbacks.setdefault('et', []).append(callback)

    def register_ai_epoch_callback(self, callback, queue):
        '''
        Configure epoch-based acquisition. The queue will be used to provide the
        start time and duration of epochs (in seconds) that should be captured
        from the analog input signal. The analog input signal is currently
        buffered with a duration of 60 seconds. This means that if you wait too
        long to notify the Engine that you want to capture a certain epoch, it
        may no longer be available in the buffer.

        Parameters
        ----------
        queue : {threading, multiprocessing, Queue}.Queue class
            This queue will provide the epoch start time and duration

        Example
        -------
        queue = Queue()
        engine.register_ai_epoch_callback(save_epoch, queue)
        ... (some time later)
        queue.put((1, 0.1))
        ... (some time later)
        queue.put((35, 10))
        '''
        task = self._tasks['hw_ai']
        mx.DAQmxGetTaskNumChans(task, self._uint32)
        buffer_size = (self._uint32.value, int(self.hw_ai_buffer_duration*fs))
        ring_buffer = np.empty(buffer_size, np.double)
        generator = extract_epochs(ring_buffer, queue, task, callback)
        self._callbacks.setdefault('ai_epoch', callback)

    def write_hw_ao(self, data, offset=None):
        task = self._tasks['hw_ao']
        if offset is not None:
            # Overwrites data already in the buffer. Used to override changes to
            # the signal.
            mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_FirstSample)
            mx.DAQmxSetWriteOffset(task, offset)
            log.trace('Writing %d samples starting at %d', data.size, offset)
        else:
            # Appends data to the end of the buffer.
            mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_CurrWritePos)
            mx.DAQmxSetWriteOffset(task, 0)
            log.trace('Writing %d samples to end of buffer', data.size)
        mx.DAQmxWriteAnalogF64(task, data.shape[-1], False, 0,
                               mx.DAQmx_Val_GroupByChannel, data, self._int32,
                               None)
        if self._int32.value != data.shape[-1]:
            raise ValueError('Unable to write all samples to channel')

    def write_sw_ao(self, state):
        task = self._tasks['sw_ao']
        state = np.array(state).astype(np.double)
        mx.DAQmxWriteAnalogF64(task, 1, True, 0, mx.DAQmx_Val_GroupByChannel,
                               state, self._int32, None)
        if self._int32.value != 1:
            raise ValueError('Unable to update software-timed AO')
        task._current_state = state

    def write_sw_do(self, state):
        task = self._tasks['sw_do']
        state = np.asarray(state).astype(np.uint8)
        mx.DAQmxWriteDigitalLines(task, 1, True, 0, mx.DAQmx_Val_GroupByChannel,
                                  state, self._int32, None)
        if self._int32.value != 1:
            raise ValueError('Problem writing data to software-timed DO')
        task._current_state = state

    def set_sw_do(self, name, state):
        task = self._tasks['sw_do']
        i = task._names.index(name)
        new_state = task._current_state.copy()
        new_state[i] = state
        self.write_sw_do(new_state)

    def set_sw_ao(self, name, state):
        task = self._tasks['sw_ao']
        i = task._names.index(name)
        new_state = task._current_state.copy()
        new_state[i] = state
        self.write_sw_ao(new_state)

    def fire_sw_do(self, name, duration=0.1):
        # TODO - Store reference to timer so that we can eventually track the
        # state of different timers and cancel pending timers when necessary.
        self.set_sw_do(name, 1)
        timer = Timer(duration, lambda: self.set_sw_do(name, 0))
        timer.start()

    def _et_fired(self, change, line, event_time):
        for cb in self._callbacks.get('et', []):
            cb(change, line, event_time)

    def _samples_acquired(self, samples):
        for cb in self._callbacks.get('ai', []):
            cb(samples)

    def _samples_needed(self, samples):
        for cb in self._callbacks.get('ao', []):
            cb(samples)

    def start(self):
        if 'hw_ao' in self._tasks:
            mx.DAQmxGetBufOutputBufSize(self._tasks['hw_ao'], self._uint32)
            log.info('AO buffer size {} samples'.format(self._uint32.value))
        for task in self._tasks.values():
            mx.DAQmxStartTask(task)

    def stop(self):
        for task in self._tasks.values():
            mx.DAQmxStopTask(task)

    def ao_write_space_available(self, offset=None):
        try:
            task = self._tasks['hw_ao']
            if offset is not None:
                mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_FirstSample)
                mx.DAQmxSetWriteOffset(task, offset)
            else:
                mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_CurrWritePos)
                mx.DAQmxSetWriteOffset(task, 0)
            mx.DAQmxGetWriteSpaceAvail(task, self._uint32)
            log.trace('Current write space available %d', self._uint32.value)
            return self._uint32.value
        except KeyError:
            raise SystemError('No hardware-timed AO task configured')


################################################################################
# demo
################################################################################
def demo_timer():
    '''
    Demonstrates accurate timestamp detection of rising and falling edges on
    PFI1 and PFI2.  To test this, use the Dev2 Test Panels (via the Measurement
    & Automation Explorer) to toggle the state of the digital lines on port 1
    between high and low.  Whenever a line is toggled to high, this will trigger
    an event.
    '''
    def event_timer_callback(task):
        src = ctypes.create_string_buffer('', size=512)
        mx.DAQmxGetSampClkSrc(task, src, 512)
        edge = ctypes.c_int32()
        mx.DAQmxGetSampClkActiveEdge(task, edge)
        data = np.empty(100, dtype=np.double)
        result = ctypes.c_int32()
        mx.DAQmxReadCounterF64(task, -1, 0, data, len(data), result, None)
        m = 'An {} event on {} was detected at time {}' \
            .format(constant_lookup(edge.value), src.value, data[:result.value])
        print(m)

    timebase = '/Dev2/10MHzRefClock'
    t1 = setup_event_timer('/Dev2/PFI1', '/Dev2/Ctr0', timebase,
                           callback=event_timer_callback)
    t2 = setup_event_timer('/Dev2/PFI1', '/Dev2/Ctr1', timebase,
                           callback=event_timer_callback, edge='falling')
    t3 = setup_event_timer('/Dev2/PFI2', '/Dev2/Ctr2', timebase,
                           callback=event_timer_callback)
    mx.DAQmxStartTask(t1)
    mx.DAQmxStartTask(t2)
    mx.DAQmxStartTask(t3)
    raw_input('Demo running. Hit enter to exit.\n')


def demo_change_detect():
    '''
    Demonstrates detection of changes digital state of lines, including an
    accurate event timer using the 10MHz timebase.  To convert the timestamp to
    seconds, divide by 10 MHz.
    '''
    def callback(edge, line, event_time):
        print('{} edge on {} at {}'.format(edge, line, event_time/10e6))

    timer_task = setup_event_timer('/Dev2/ChangeDetectionEvent', '/Dev2/Ctr3',
                                   '/Dev2/10MhzRefClock')
    cd_task = setup_change_detect_callback('/Dev2/port1/line1:2', callback,
                                           timer=timer_task,
                                           names=['spout', 'lick'])
    mx.DAQmxStartTask(timer_task)
    mx.DAQmxStartTask(cd_task)
    raw_input('Demo running. Hit enter to exit.\n')


def simple_engine_demo():
    '''
    This demonstrates the basic Engine interface that we will be using to
    communicate with the DAQ hardware. A subclass of the Engine will implement
    NI hardware-specific logic.  Another subclass will implement MCC
    (TBSI)-specific logic. This enables us to write code that does not care
    whether it's communicating with a NI or MCC device.
    '''
    def ao_callback(samples):
        print('{} samples generated'.format(samples))

    def ai_callback(samples):
        print('{} samples acquired'.format(samples.shape))

    def et_callback(change, line, event_time):
        print('{} edge on {} at {}'.format(change, line, event_time))

    initial_data = np.zeros(1000e3, dtype=np.double)
    engine = Engine()
    engine.configure_hw_ai(20e3, '/Dev2/ai0:4', (-10, 10), sync_ao=False)
    engine.configure_hw_ao(20e3, '/Dev2/ao0', (-10, 10))
    engine.configure_et('/Dev2/port1/line0:7', 'ao/SampleClock')

    engine.register_ao_callback(ao_callback)
    engine.register_ai_callback(ai_callback)
    engine.register_et_callback(et_callback)

    engine.write_hw_ao(initial_data)
    engine.start()
    return engine


class EngineDemo(object):
    '''
    Supporting class for the masker_engine_demo
    '''

    def __init__(self):
        from scipy.io import wavfile

        # Read in the stimulus files and convert them to a floating-point value
        # between 0 and 1 by dividing against the maximum possible value
        # specified by the dtype (e.g., 16 bit signed has a maximum value of
        # 32767).
        fs_masker, masker = wavfile.read('stimuli/supermasker1_1k.wav')
        fs_T00, T00 = wavfile.read('stimuli/T00.wav')
        fs_T01, T01 = wavfile.read('stimuli/T01.wav')
        self.masker = masker.astype(np.double)/np.iinfo(masker.dtype).max
        self.T00 = T00.astype(np.double)/np.iinfo(T00.dtype).max
        self.T01 = T01.astype(np.double)/np.iinfo(T01.dtype).max

        # Ensure sampling rate is identical among all three files otherwise this
        # won't work.
        if (fs_masker != fs_T00) or (fs_masker != fs_T01):
            raise ValueError('Cannot reconcile sampling rate difference')
        self.fs = fs_masker

        # TEMPORARY - use a sine wave as the masker. Discontinuities are more
        # obvious when using this.
        t = np.arange(int(self.fs*5), dtype=np.double)/self.fs
        self.masker = np.sin(2*np.pi*5*t)

        # How soon after the "trigger" (i.e., the decision to start a trial)
        # should the target be inserted? This needs to give Neurobehavior
        # sufficient time to generate the waveform and overwrite the buffer.
        self.update_delay = int(self.fs*0.05)

        # How often should we add new masker data to the buffer?
        self.update_interval = int(self.fs*1)

        # Place to save acquired data
        self._acquired = []
        self._event_times = []
        self._masker_offset = 0

        self.engine = Engine()
        self.engine.configure_hw_ai(self.fs, 'Dev2/ai0', (-10, 10))
        self.engine.configure_hw_ao(self.fs, 'Dev2/ao0', (-10, 10))
        self.engine.configure_et('Dev2/PFI1', 'ao/SampleClock')
        self.engine.write_hw_ao(self.masker)

        self.engine.register_ao_callback(self.samples_needed)
        self.engine.register_ai_callback(self.samples_acquired)
        self.engine.register_et_callback(self.et_fired)

        self.engine.start()

    def et_fired(self, edge, line, timestamp):
        if edge == 'rising' and line == 'Dev2/PFI1':
            log.debug('Detected trigger at %d', timestamp)
            self._event_times.append(timestamp)

            offset = int(timestamp) + self.update_delay
            log.debug('Inserting target at %d', offset)
            duration = self.engine.ao_write_space_available(offset)
            log.debug('Overwriting %d samples in buffer', duration)
            result = self._get_masker(offset, duration)
            result[:self.T01.shape[-1]] += self.T01*0.1
            self.engine.write_hw_ao(result, offset)
            self._masker_offset = offset + result.shape[-1]

    def samples_acquired(self, samples):
        self._acquired.append(samples)

    def samples_needed(self, samples):
        result = self._get_masker(self._masker_offset, samples)
        self.engine.write_hw_ao(result)
        self._masker_offset += samples

    def _get_masker(self, masker_offset, masker_duration):
        '''
        Get the next `duration` samples of the masker starting at `offset`. If
        reading past the end of the array, loop around to the beginning.
        '''
        masker_size = self.masker.shape[-1]
        offset = masker_offset % masker_size
        duration = masker_duration
        result = []
        while True:
            if (offset+duration) < masker_size:
                subset = self.masker[offset:offset+duration]
                duration = 0
            else:
                log.trace('Wrapping to beginning of masker')
                subset = self.masker[offset:]
                offset = 0
                duration = duration-subset.shape[-1]
            result.append(subset)
            if duration == 0:
                break
        return np.concatenate(result, axis=-1)


def masker_engine_demo():
    '''
    Similar to the original demo code. Demonstrates how one might quickly
    overwrite samples in the analog output buffer in response to an event.
    '''
    import pylab as pl
    logging.basicConfig(level='TRACE')
    engine_demo = EngineDemo()
    raw_input('Demo running. Hit enter to exit.\n')
    engine_demo.engine.stop()
    acquired = np.concatenate(engine_demo._acquired, axis=-1)
    event_times = np.array(engine_demo._event_times)/engine_demo.fs
    t = np.arange(acquired.shape[-1], dtype=np.double)/engine_demo.fs
    pl.plot(t, acquired.T, 'k-')
    pl.plot(event_times, np.zeros_like(event_times), 'ro')
    pl.show()


def demo_sw():
    '''
    Demonstrates the use of software-timed analog outputs and digital outputs.
    Software-timed outputs change the state (or value) of an output to the new
    level as soon as requested by the program.

    A common use-case for software-timed digital outputs is to generate a TTL
    (i.e., a square pulse) to trigger something (e.g., a water pump). I have
    included convenience functions (`fire_sw_do`) to facilitate this.

    I also have included name aliases. NI-DAQmx refers to the lines using the
    arcane syntax (e.g., 'Dev2/ao0' or 'Dev2/port0/line0'). However, it is
    probably easier (and generates more readable programs), if we can assign
    aliases to these lines. For example, if 'Dev2/ao0' is connected to a
    Coulbourn shock controller which uses the analog signal to control the
    intensity of the shock, we may want to call that output 'shock_level'.
    Likewise, if 'Dev2/port0/line1' is connected to the trigger port of a pellet
    dispenser, we probably want to call that line 'food_dispense'.
    '''
    import time
    engine = Engine()
    # Configure four digital outputs and give them the aliases 'a', 'b', 'c',
    # and 'd'.
    engine.configure_sw_do('Dev2/port0/line0:3', ['a', 'b', 'c', 'd'],
                           initial_state=[1, 0, 1, 1])
    # Configure two analog otuputs and give them aliases (i.e., assume ao0
    # controls shock level and ao1 controls pump rate).
    engine.configure_sw_ao('Dev2/ao0:1', (-10, 10),
                           ['shock_level', 'pump_rate'])
    time.sleep(1)

    # You can connect ao0 to ai0 and use the analog input panel on the MAX test
    # panels to observe the changes to the analog output.
    engine.set_sw_ao('shock_level', 5)
    engine.set_sw_ao('pump_rate', 4)

     # You can connect the digital lines for port0 to port1 and then monitor the
     # changes on port1 using the digital panel on the MAX test panels.
    time.sleep(1)
    engine.write_sw_do([0, 0, 0, 0])
    engine.start()
    engine.set_sw_do('a', 1)
    time.sleep(0.25)
    engine.set_sw_do('b', 1)
    time.sleep(0.25)
    engine.set_sw_do('c', 1)
    time.sleep(0.25)
    engine.set_sw_do('a', 0)
    time.sleep(0.25)
    engine.set_sw_do('b', 0)
    time.sleep(0.25)
    engine.set_sw_do('c', 0)
    time.sleep(0.25)

    # Generate a TTL pulse of varying duration (in sec).
    engine.fire_sw_do('a', 0.25)
    engine.fire_sw_do('b', 0.5)
    engine.fire_sw_do('c', 1)
    time.sleep(1)


class AccumulatorDemo(object):

    def __init__(self):
        from Queue import Queue
        from uuid import uuid4
        queue = Queue()
        self._accumulated = []
        extractor = extract_epochs(queue, self.epoch_acquired_cb)
        uuid = str(uuid4())
        queue.put((str(uuid4()), 10, 10))
        queue.put((str(uuid4()), 5, 15))
        queue.put((str(uuid4()), 300, 25))
        extractor.send(np.arange(100))
        queue.put((str(uuid4()), 50, 25))
        queue.put((str(uuid4()), 150, 13))
        extractor.send(np.arange(100, 200))
        extractor.send(np.arange(200, 400))
        raw_input('Demo running. Press enter to exit.')

    def epoch_acquired_cb(self, uuid, data):
        if data is None:
            print('epoch {} was lost'.format(uuid))
        else:
            print('epoch {} acquired with shape {}'.format(uuid, data.shape))
            print(data)


################################################################################
# Test cases
################################################################################

if __name__ == '__main__':
    #demo_timer()
    #demo_change_detect()
    #demo_sw()
    #simple_engine_demo()
    #masker_engine_demo()
    AccumulatorDemo()
