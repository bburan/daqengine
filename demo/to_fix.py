################################################################################
# demo
################################################################################
def demo_timer(device='Dev1'):
    '''
    Demonstrates accurate timestamp detection of rising and falling edges on
    PFI1 and PFI2.  To test this, use the Dev1 Test Panels (via the Measurement
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

    timebase = '/Dev1/10MHzRefClock'
    t1 = setup_event_timer('/Dev1/PFI1', '/Dev1/Ctr0', timebase,
                           callback=event_timer_callback)
    #t2 = setup_event_timer('/Dev1/PFI1', '/Dev1/Ctr1', timebase,
                           #callback=event_timer_callback, edge='falling')
    #t3 = setup_event_timer('/Dev1/PFI2', '/Dev1/Ctr2', timebase,
    #                       callback=event_timer_callback)
    mx.DAQmxStartTask(t1)
    #mx.DAQmxStartTask(t2)
    #mx.DAQmxStartTask(t3)
    raw_input('Demo running. Hit enter to exit.\n')



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
        self.update_delay = int(self.fs*0.25)

        # How often should we add new masker data to the buffer?
        self.update_interval = int(self.fs*1)

        # Place to save acquired data
        self._acquired = []
        self._event_times = []
        self._masker_offset = 0

        self.engine = Engine()
        self.engine.configure_hw_ai(self.fs, 'Dev1/ai0', (-10, 10))
        self.engine.configure_hw_ao(self.fs, 'Dev1/ao0', (-10, 10))
        self.engine.configure_et('/Dev1/port0/line0', 'ao/SampleClock',
                                 names=['trigger'])
        self.engine.write_hw_ao(self.masker)

        self.engine.register_ao_callback(self.samples_needed)
        self.engine.register_ai_callback(self.samples_acquired)
        self.engine.register_et_callback(self.et_fired)

        self.engine.start()

    def et_fired(self, edge, line, timestamp):
        if edge == 'rising' and line == 'trigger':
            log.debug('Detected trigger at %d', timestamp)
            self._event_times.append(timestamp)
            offset = int(timestamp) + self.update_delay
            log.debug('Inserting target at %d', offset)
            duration = self.engine.ao_write_space_available(offset)/2
            log.debug('Overwriting %d samples in buffer', duration)
            result = self._get_masker(offset, duration)
            result[:self.T01.shape[-1]] += self.T01*0.1
            self.engine.write_hw_ao(result, offset)
            self.engine.write_hw_ao(result)
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
    import time
    engine_demo = EngineDemo()
    raw_input('Demo running. Hit enter to exit.\n')
    engine_demo.engine.stop()
    acquired = np.concatenate(engine_demo._acquired, axis=-1)
    event_times = np.array(engine_demo._event_times)/engine_demo.fs
    t = np.arange(acquired.shape[-1], dtype=np.double)/engine_demo.fs
    pl.plot(t, acquired.T, 'k-')
    pl.plot(event_times, np.zeros_like(event_times), 'ro')
    pl.show()


def test_write():
    import pylab as pl

    class WriteTest(object):

        def __init__(self):
            self.ai_task = setup_hw_ai(25e3, '/Dev1/ai0', (-10, 10),
                                       self.ai_callback, 2500, sync_ao=True)
            self.ao_task = setup_hw_ao(25e3, '/Dev1/ao0', (-10, 10),
                                       self.ao_callback, 10000)
            self.acquired = []
            self.ao_callback(10000, 1)
            mx.DAQmxStartTask(self.ai_task)
            mx.DAQmxStartTask(self.ao_task)
            result = ctypes.c_int32()
            self.ao_callback(2500, 0.5)

            mx.DAQmxSetWriteRelativeTo(self.ao_task, mx.DAQmx_Val_FirstSample)
            mx.DAQmxSetWriteOffset(self.ao_task, 10000+2500-1250)
            self.ao_callback(2500, 0)

            result = ctypes.c_uint64()
            mx.DAQmxGetWriteCurrWritePos(self.ao_task, result)
            written = result.value

            result = ctypes.c_uint64()
            mx.DAQmxGetWriteTotalSampPerChanGenerated(self.ao_task, result)
            generated = result.value

            print('Current generate position', result.value)
            print('Current write position', written)

            mx.DAQmxSetWriteRelativeTo(self.ao_task, mx.DAQmx_Val_FirstSample)
            #mx.DAQmxSetWriteOffset(self.ao_task, written-5000)
            mx.DAQmxSetWriteOffset(self.ao_task, generated+8300)
            self.ao_callback(2500, -1)

        def ao_callback(self, samples, sf=2):
            try:
                result = ctypes.c_uint64()
                mx.DAQmxGetWriteCurrWritePos(self.ao_task, result)
                print('Current write position', result.value)
                mx.DAQmxGetWriteSpaceAvail(self.ao_task, result)
                print('Current write space available', result.value)
            except:
                pass

            result = ctypes.c_int32()
            data = np.ones(samples)*sf
            mx.DAQmxWriteAnalogF64(self.ao_task, data.shape[-1], False, 0,
                                   mx.DAQmx_Val_GroupByChannel, data, result,
                                   None)

        def ai_callback(self, samples):
            self.acquired.append(samples)

    test = WriteTest()
    raw_input('ready')
    mx.DAQmxStopTask(test.ao_task)
    acquired = np.concatenate(test.acquired, axis=-1)
    pl.plot(acquired.T)
    pl.show()


################################################################################
# Test cases
################################################################################

if __name__ == '__main__':
    pass
    #logging.basicConfig(level='TRACE')
    #demo_timer()
    #masker_engine_demo()
    #test_write()
