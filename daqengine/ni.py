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
from threading import Timer

import numpy as np
import PyDAQmx as mx

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
    data = np.empty((size, nlines.value), dtype=np.uint8)
    mx.DAQmxReadDigitalLines(task, size, 0, mx.DAQmx_Val_GroupByChannel, data,
                             data.size, nsamp, nbytes, None)
    return data.T


def constant_lookup(value):
    for name in dir(mx.DAQmxConstants):
        if name in mx.DAQmxConstants.constant_list:
            if getattr(mx.DAQmxConstants, name) == value:
                return name
    raise ValueError('Constant {} does not exist'.format(value))


def channel_info(channels, channel_type):
    task = create_task()
    if channel_type in ('di', 'do', 'digital'):
        mx.DAQmxCreateDIChan(task, channels, '', mx.DAQmx_Val_ChanPerLine)
    elif channel_type == 'ao':
        mx.DAQmxCreateAOVoltageChan(task, channels, '', -10, 10,
                                    mx.DAQmx_Val_Volts, '')
    elif channel_type == 'ai':
        mx.DAQmxCreateAIVoltageChan(task, channels, '',
                                    mx.DAQmx_Val_Cfg_Default, -10, 10,
                                    mx.DAQmx_Val_Volts, '')

    channels = ctypes.create_string_buffer('', 4096)
    mx.DAQmxGetTaskChannels(task, channels, len(channels))
    devices = ctypes.create_string_buffer('', 4096)
    mx.DAQmxGetTaskDevices(task, devices, len(devices))
    mx.DAQmxClearTask(task)

    return {
        'channels': [c.strip() for c in channels.value.split(',')],
        'devices': [d.strip() for d in devices.value.split(',')],
    }


def channel_list(channels, channel_type):
    return channel_info(channels, channel_type)['channels']


def channel_names(channel_type, lines, names):
    lines = channel_list(lines, channel_type)
    if names is not None:
        if len(lines) != len(names):
            raise ValueError('Number of names must match number of lines')
    else:
        names = lines
    return names


def device_list(channels, channel_type):
    return channel_info(channels, channel_type)['devices']


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
            offset = self._uint64.value
            mx.DAQmxGetWriteTotalSampPerChanGenerated(task, self._uint64)
            generated = self._uint64.value
            mx.DAQmxGetWriteSpaceAvail(task, self._uint32)
            available = self._uint32.value
            log.trace('Current write position %d', offset)
            log.trace('Total s/chan generated %d', generated)
            log.trace('Current write space available %d', available)
            if self._uint32.value != 0:
                self._callback(offset, available)
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


class DigitalSamplesAcquiredCallbackHelper(object):

    def __init__(self, callback):
        self._callback = callback

    def __call__(self, task, event_type, callback_samples, callback_data):
        try:
            data = read_digital_lines(task, callback_samples)
            self._callback(data)
            return 0
        except Exception as e:
            raise
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
        mx.DAQmxReadCounterF64(self._timer_task, -1, 0, data, len(data),
                               result, None)
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
def extract_edges(names, initial_states, min_samples, target):
    coroutines = []
    for name, initial_state in zip(names, initial_states):
        c = _extract_edges(name, initial_state, min_samples, target)
        coroutines.append(c)
    while True:
        new_samples = (yield)
        for cr, line_samples in zip(coroutines, new_samples):
            cr.send(line_samples)


@coroutine
def _extract_edges(name, initial_state, min_samples, target):
    if min_samples < 1:
        raise ValueError('min_samples must be greater than 1')
    prior_samples = np.tile(initial_state, min_samples)
    t_prior = -min_samples
    while True:
        # Wait for new data to become available
        new_samples = (yield)
        samples = np.r_[prior_samples, new_samples]
        ts_change = np.flatnonzero(np.diff(samples, axis=-1)) + 1
        ts_change = np.r_[ts_change, samples.shape[-1]] 

        for tlb, tub in zip(ts_change[:-1], ts_change[1:]):
            if (tub-tlb) >= min_samples:
                if initial_state == samples[tlb]:
                    continue
                edge = 'rising' if samples[tlb] == 1 else 'falling'
                initial_state = samples[tlb]
                target(name, edge, t_prior + tlb)

        t_prior += new_samples.shape[-1]
        prior_samples = samples[..., -min_samples:]


@coroutine
def capture_epoch(names, start, duration, callback):
    '''
    Coroutine to facilitate epoch acquisition
    '''
    # This coroutine will continue until it acquires all the samples it needs.
    # It then provides the samples to the callback function and exits the while
    # loop.
    current_start = start
    remaining_duration = duration
    accumulated_data = []
    while True:
        tlb, data = (yield)
        samples = data.shape[-1]
        if current_start < tlb:
            # We have missed the start of the epoch. Notify the callback of this
            log.debug('Missed samples for epoch of %d samples starting at %d',
                      start, duration)
            callback(names, start, duration, None)
            break
        elif current_start <= (tlb + samples):
            # The start of the epoch is somewhere inside `data`. Find the start
            # `i` and determine how many samples `d` to extract from `data`.
            # It's possible that data does not contain the entire epoch. In that
            # case, we just pull out what we can and save it in
            # `accumulated_data`. We then update start to point to the last
            # acquired sample `i+d` and update duration to be the number of
            # samples we still need to capture.
            i = int(current_start-tlb)
            d = int(min(remaining_duration, samples-i))
            accumulated_data.append(data[..., i:i+d])
            current_start += d
            remaining_duration -= d

            # Check to see if we've finished acquiring the entire epoch. If so,
            # send it to the callback. Also, do a sanity check on the parameters
            # (duration < 0 means that something happened and we can't recover.
            if remaining_duration == 0:
                accumulated_data = np.concatenate(accumulated_data, axis=-1)
                callback(names, start, duration, accumulated_data)
                break
            elif remaining_duration < 0:
                log.debug('Acquired too many samples for epoch of %d samples '
                          'starting at %d', start, duration)
                callback(names, start, duration, None)
                break


@coroutine
def extract_epochs(names, queue, callback, buffer_samples):
    '''
    Parameters
    ----------
    queue
    callback
    buffer_samples
    '''
    # The variable `tlb` tracks the number of samples that have been acquired
    # and reflects the lower bound of `data`. For example, if we have acquired
    # 300,000 samples, then the next chunk of data received from (yield) will
    # start at sample 300,000 (remember that Python is zero-based indexing, so
    # the first sample has an index of 0).
    tlb = 0
    epoch_coroutines = []
    prior_samples = []

    while True:
        # Wait for new data to become available
        data = (yield)

        # Check to see if more epochs have been requested. Information will be
        # provided in seconds, but we need to convert this to number of samples.
        while not queue.empty():
            start, duration = queue.get()
            epoch_coroutine = capture_epoch(names, start, duration, callback)
            # Go through the data we've been caching to facilitate historical
            # acquisition of data.
            epoch_coroutines.append(epoch_coroutine)
            for prior_sample in prior_samples:
                try:
                    epoch_coroutine.send(prior_sample)
                except StopIteration:
                    epoch_coroutines.remove(epoch_coroutine)

        # Send the data to each coroutine. If a StopIteration occurs, this means
        # that the epoch has successfully been acquired and has been sent to the
        # callback and we can remove it. Need to operate on a copy of list since
        # it's bad form to modify a list in-place.
        for epoch_coroutine in epoch_coroutines[:]:
            try:
                epoch_coroutine.send((tlb, data))
            except StopIteration:
                epoch_coroutines.remove(epoch_coroutine)

        prior_samples.append((tlb, data))
        tlb = tlb + data.shape[-1]

        # Check to see if any of the cached samples are older than the specified
        # `buffer_samples`.
        while True:
            tub = prior_samples[0][0] + prior_samples[0][1].shape[-1]
            if tub < (tlb-buffer_samples):
                prior_samples.pop(0)
            else:
                break


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
    mx.DAQmxCfgSampClkTiming(task, trigger, 200e3, edge_value,
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
    # 'Dev1/port0/line0:1') or listed each line individually (e.g.,
    # 'Dev1/port0/line0, Dev1/port0/line1'), we need to extract the individual
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


def setup_hw_ao(fs, lines, expected_range, callback, callback_samples,
                trigger=None):
    # TODO: DAQmxSetAOTermCfg
    task = create_task()
    lb, ub = expected_range
    mx.DAQmxCreateAOVoltageChan(task, lines, '', lb, ub, mx.DAQmx_Val_Volts, '')
    if trigger is not None:
        mx.DAQmxCfgDigEdgeStartTrig(task, trigger, mx.DAQmx_Val_Rising)
    mx.DAQmxCfgSampClkTiming(task, '', fs, mx.DAQmx_Val_Rising,
                             mx.DAQmx_Val_ContSamps, int(fs))

    # This controls how quickly we can update the buffer on the device. On some
    # devices it is not user-settable. On the X-series PCIe-6321 I am able to
    # change it. On the M-xeries PCI 6259 it appears to be fixed at 8191
    # samples.
    mx.DAQmxSetBufOutputOnbrdBufSize(task, 8191)

    # If the write reaches the end of the buffer and no new data has been
    # provided, do not loop around to the beginning and start over.
    mx.DAQmxSetWriteRegenMode(task, mx.DAQmx_Val_DoNotAllowRegen)

    mx.DAQmxSetBufOutputBufSize(task, int(callback_samples*100))

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


def setup_hw_ai(fs, lines, expected_range, callback, callback_samples,
                trigger=None):
    # Record AI filter delay
    task = create_task()
    lb, ub = expected_range
    mx.DAQmxCreateAIVoltageChan(task, lines, '', mx.DAQmx_Val_RSE, lb, ub,
                                mx.DAQmx_Val_Volts, '')
    if trigger is not None:
        mx.DAQmxCfgDigEdgeStartTrig(task, trigger, mx.DAQmx_Val_Rising)

    mx.DAQmxCfgSampClkTiming(task, '', fs, mx.DAQmx_Val_Rising,
                             mx.DAQmx_Val_ContSamps, int(fs))

    result = ctypes.c_uint32()
    mx.DAQmxGetBufInputBufSize(task, result)
    buffer_size = result.value
    mx.DAQmxGetTaskNumChans(task, result)
    n_channels = result.value

    log.debug('Buffer size for %s automatically allocated as %d samples',
              lines, buffer_size)
    log.debug('%d channels in task', n_channels)

    new_buffer_size = np.ceil(buffer_size/callback_samples)*callback_samples
    mx.DAQmxSetBufInputBufSize(task, int(new_buffer_size))

    callback_helper = SamplesAcquiredCallbackHelper(callback, n_channels)
    cb_ptr = mx.DAQmxEveryNSamplesEventCallbackPtr(callback_helper)
    mx.DAQmxRegisterEveryNSamplesEvent(task, mx.DAQmx_Val_Acquired_Into_Buffer,
                                       int(callback_samples), 0, cb_ptr, None)

    mx.DAQmxTaskControl(task, mx.DAQmx_Val_Task_Commit)
    rate = ctypes.c_double()
    mx.DAQmxGetSampClkRate(task, rate)
    log.debug('AI sample rate'.format(rate.value))
    mx.DAQmxGetSampClkTimebaseRate(task, rate)
    log.debug('AI timebase {}'.format(rate.value))
    task._cb_ptr = cb_ptr
    return task


def setup_hw_di(fs, lines, callback, callback_samples, clock=None,
                trigger=None):
    '''
    M series DAQ cards do not have onboard timing engines for digital IO.
    Therefore, we have to create one (e.g., using a counter or by using the
    analog input or output sample clock. 
    '''
    task = create_task()
    mx.DAQmxCreateDIChan(task, lines, '', mx.DAQmx_Val_ChanForAllLines)

    # Get the current state of the lines so that we know what happened during
    # the first change detection event. Do this before configuring the timing
    # of the lines (otherwise we have to start the master clock as well)!
    mx.DAQmxStartTask(task)
    initial_state = read_digital_lines(task, 1)
    mx.DAQmxStopTask(task)

    if clock is None:
        clock = ''

    if 'Ctr' in clock:
        clock_task = create_task()
        mx.DAQmxCreateCOPulseChanFreq(clock_task, clock, '', mx.DAQmx_Val_Hz,
                                      mx.DAQmx_Val_Low, 0, fs, 0.5)
        mx.DAQmxCfgImplicitTiming(clock_task, mx.DAQmx_Val_ContSamps, int(fs))
        clock += 'InternalOutput'
        if trigger is not None:
            mx.DAQmxCfgDigEdgeStartTrig(clock_task, trigger,
                                        mx.DAQmx_Val_Rising)
    else:
        clock_task = None


    mx.DAQmxCfgSampClkTiming(task, clock, fs, mx.DAQmx_Val_Rising,
                             mx.DAQmx_Val_ContSamps, int(fs))

    callback_helper = DigitalSamplesAcquiredCallbackHelper(callback)
    cb_ptr = mx.DAQmxEveryNSamplesEventCallbackPtr(callback_helper)
    mx.DAQmxRegisterEveryNSamplesEvent(task, mx.DAQmx_Val_Acquired_Into_Buffer,
                                       int(callback_samples), 0, cb_ptr, None)

    task._cb_ptr = cb_ptr
    task._initial_state = initial_state

    mx.DAQmxTaskControl(task, mx.DAQmx_Val_Task_Commit)
    rate = ctypes.c_double()
    mx.DAQmxGetSampClkRate(task, rate)
    #print('DI rate', rate.value)
    #mx.DAQmxGetSampClkTimebaseRate(task, rate)
    #print('DI timebase', rate.value)

    return [task, clock_task]


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
    # Poll period (in seconds). This defines how often callbacks for the analog
    # outputs are notified (i.e., to generate additional samples for playout).
    # If the poll period is too long, then the analog output may run out of
    # samples.
    hw_ao_monitor_period = 1

    # Poll period (in seconds). This defines how quickly acquired (analog input)
    # data is downloaded from the buffers (and made available to listeners). If
    # you want to see data as soon as possible, set the poll period to a small
    # value. If your application is stalling or freezing, set this to a larger
    # value.
    hw_ai_monitor_period = 0.1

    # Even though data is written to the analog outputs, it is buffered in
    # computer memory until it's time to be transferred to the onboard buffer of
    # the NI acquisition card. NI-DAQmx handles this behind the scenes (i.e.,
    # when the acquisition card needs additional samples, NI-DAQmx will transfer
    # the next chunk of data from the computer memory). We can overwrite data
    # that's been buffered in computer memory (e.g., so we can insert a target
    # in response to a nose-poke). However, we cannot overwrite data that's
    # already been transfered to the onboard buffer. So, the onboard buffer size
    # determines how quickly we can change the analog output in response to an
    # event.
    hw_ao_onboard_buffer = 8191

    # Since any function call takes a small fraction of time (e.g., nanoseconds
    # to milliseconds), we can't simply overwrite data starting at
    # hw_ao_onboard_buffer+1. By the time the function calls are complete, the
    # DAQ probably has already transferred a couple hundred samples to the
    # buffer. This parameter will likely need some tweaking (i.e., only you can
    # determine an appropriate value for this based on the needs of your
    # program).
    hw_ao_min_writeahead = hw_ao_onboard_buffer + 1000

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

    def configure_hw_ao(self, fs, lines, expected_range, names=None):
        '''
        Initialize hardware-timed analog output

        Parameters
        ----------
        fs : float
            Sampling frequency of output (e.g., 100e3).
        lines : str
            Analog output lines to use (e.gk., 'Dev1/ao0:4' to specify a range of
            lines or 'Dev1/ao0,Dev1/ao4' to specify specific lines).
        expected_range : (float, float)
            Tuple of upper/lower end of expected range. The maximum range
            allowed by most NI devices is (-10, 10). Some devices (especially
            newer ones) will optimize the output resolution based on the
            expected range of the signal.
        '''
        task = setup_hw_ao(fs, lines, expected_range, self._hw_ao_callback,
                           int(self.hw_ao_monitor_period*fs))
        task._names = channel_names('ao', lines, names)
        self._tasks['hw_ao'] = task

        mx.DAQmxGetBufOutputBufSize(self._tasks['hw_ao'], self._uint32)
        self.hw_ao_buffer_samples = self._uint32.value
        log.info('AO buffer size {} samples'.format(self.hw_ao_buffer_samples))

    def configure_hw_ai(self, fs, lines, expected_range, names=None, trigger=None):
        callback_samples = int(self.hw_ai_monitor_period*fs)
        task = setup_hw_ai(fs, lines, expected_range, self._hw_ai_callback,
                           callback_samples, trigger)
        task._fs = fs
        task._names = channel_names('ai', lines, names)
        self._tasks['hw_ai'] = task

    def configure_sw_ao(self, lines, expected_range, names=None,
                        initial_state=None):
        if initial_state is None:
            initial_state = np.zeros(len(names), dtype=np.double)
        task = setup_sw_ao(lines, expected_range)
        task._names = channel_names('ao', lines, names)
        self._tasks['sw_ao'] = task
        self.write_sw_ao(initial_state)

    def configure_hw_di(self, fs, lines, names=None, clock=None, trigger=None):
        names = channel_names('digital', lines, names)
        callback_samples = int(self.hw_ai_monitor_period*fs)
        task, clock_task = setup_hw_di(fs, lines, self._hw_di_callback,
                                       callback_samples, clock, trigger)
        task._names = names
        if clock_task is not None:
            self._tasks['hw_di_clock'] = clock_task
        self._tasks['hw_di'] = task

    def configure_hw_do(self, fs, lines, names):
        raise NotImplementedError

    def configure_sw_do(self, lines, names=None, initial_state=None):
        names = channel_names('digital', lines, names)
        if initial_state is None:
            initial_state = np.zeros(len(names), dtype=np.uint8)
        task = setup_sw_do(lines)
        task._names = names
        self._tasks['sw_do'] = task
        self.write_sw_do(initial_state)

    def configure_et(self, lines, clock, names=None):
        '''
        Setup change detection with high-precision timestamps

        Anytime a rising or falling edge is detected on one of the specified
        lines, a timestamp based on the specified clock will be captured. For
        example, if the clock is 'ao/SampleClock', then the timestamp will be
        the number of samples played at the point when the line changed state.

        Parameters
        ----------
        lines : string
            Digital lines (in NI-DAQmx syntax, e.g., 'Dev1/port0/line0:4') to
            monitor.
        clock : string
            Reference clock from which timestamps will be drawn.
        names : string (optional)
            Aliases for the lines. When aliases are provided, registered
            callbacks will receive the alias for the line instead of the
            NI-DAQmx notation.

        Notes
        -----
        Be aware of the limitations of your device. All X-series devices support
        change detection on all ports; however, only some M-series devices do
        (and then, only on port 0).
        '''
        # Find out which device the lines are from. Use this to configure the
        # event timer. Right now we don't want to deal with multi-device event
        # timers. If there's more than one device, then we should configure each
        # separately.
        devices = device_list(lines, 'digital')
        if len(devices) != 1:
            raise ValueError('Cannot configure multi-device event timer')

        trigger = '/{}/ChangeDetectionEvent'.format(devices[0])
        counter = '/{}/Ctr0'.format(devices[0])
        et_task = setup_event_timer(trigger, counter, clock)
        cd_task = setup_change_detect_callback(lines, self._et_fired, et_task,
                                               names)
        self._tasks['et_task'] = et_task
        self._tasks['cd_task'] = cd_task

    def register_ao_callback(self, callback):
        self._callbacks.setdefault('ao', []).append(callback)

    def register_ai_callback(self, callback):
        self._callbacks.setdefault('ai', []).append(callback)

    def register_di_callback(self, callback):
        self._callbacks.setdefault('di', []).append(callback)

    def register_et_callback(self, callback):
        self._callbacks.setdefault('et', []).append(callback)

    def register_ai_epoch_callback(self, callback, queue, buffer_size=60):
        '''
        Configure epoch-based acquisition. The queue will be used to provide the
        start time and duration of epochs (in seconds) that should be captured
        from the analog input signal. The analog input signal can be buffered,
        allowing you to retroactively request an epoch that was already
        acquired.  However, the buffer will be a fixed duration, so if you wait
        too long to notify the Engine that you want to capture a certain epoch,
        it may no longer be available in the buffer.

        Parameters
        ----------
        callback : callable
            Function to be called when the entire epoch is acquired. The
            function will receive the uuid and epoch data.
        queue : {threading, multiprocessing, Queue}.Queue class
            This queue will provide the epoch start time and duration
        buffer_size : float
            Buffer size in seconds

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
        buffer_samples = int(buffer_size*task._fs)
        generator = extract_epochs(task._names, queue, callback,
                                   buffer_samples)
        self._callbacks.setdefault('ai_epoch', []).append(generator)

    def register_di_change_callback(self, callback, debounce=1):
        '''
        Configure change detection on hardware-timed digital inputs.

        Parameters
        ----------
        callback : callable
            Function to be called when a change event is detected on one of the
            digital lines.
        debounce : int
            Number of samples to use for debouncing filter. The line must
            maintain the new state for the specified number of samples before
            the change is recorded. Set to 1 for no debouncing.
        '''
        task = self._tasks['hw_di']
        generator = extract_edges(task._names, task._initial_state, debounce,
                                  callback)
        self._callbacks.setdefault('di_edges', []).append(generator)

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
                               mx.DAQmx_Val_GroupByChannel,
                               data.astype(np.float64), self._int32, None)
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

    def _hw_ai_callback(self, samples):
        task = self._tasks['hw_ai']
        for cb in self._callbacks.get('ai', []):
            cb(task._names, samples)
        for generator in self._callbacks.get('ai_epoch', []):
            generator.send(samples)

    def _hw_di_callback(self, samples):
        task = self._tasks['hw_di']
        for cb in self._callbacks.get('di', []):
            cb(task._names, samples)
        for generator in self._callbacks.get('di_edges', []):
            generator.send(samples)

    def _hw_ao_callback(self, offset, samples):
        names = self._tasks['hw_ao']._names
        for cb in self._callbacks.get('ao', []):
            cb(names, offset, samples)

    def start(self):
        if 'hw_ao' in self._tasks:
            self._hw_ao_callback(0, self.hw_ao_buffer_samples)
        for task in self._tasks.values():
            mx.DAQmxStartTask(task)

    def stop(self):
        for task in self._tasks.values():
            mx.DAQmxStopTask(task)

    def ao_write_space_available(self, offset=None):
        try:
            task = self._tasks['hw_ao']
            mx.DAQmxGetWriteCurrWritePos(task, self._uint64)
            log.trace('Current write position %d', self._uint64.value)
            if offset is not None:
                mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_FirstSample)
                mx.DAQmxSetWriteOffset(task, offset)
            else:
                mx.DAQmxSetWriteRelativeTo(task, mx.DAQmx_Val_CurrWritePos)
                mx.DAQmxSetWriteOffset(task, -1000)
            mx.DAQmxGetWriteSpaceAvail(task, self._uint32)
            log.trace('Current write space available %d', self._uint32.value)
            return self._uint32.value
        except KeyError:
            raise SystemError('No hardware-timed AO task configured')

    def ao_sample_clock(self):
        task = self._tasks['hw_ao']
        mx.DAQmxGetWriteTotalSampPerChanGenerated(task, self._uint64)
        return self._uint64.value
