from functools import partial

import Queue
import numpy as np

from daqengine.ni import Engine

def main(device):
    '''
    This demonstrates the basic Engine interface that we will be using to
    communicate with the DAQ hardware. A subclass of the Engine will implement
    NI hardware-specific logic.  Another subclass will implement MCC
    (TBSI)-specific logic. This enables us to write code that does not care
    whether it's communicating with a NI or MCC device.
    '''
    def ao_callback(offset, samples):
        print('{} samples needed at {}'.format(samples, offset, name))
        engine.write_hw_ao(np.zeros(samples))

    def ai_callback(name, data):
        print('{} samples acquired from {}'.format(data.shape, name))

    def et_callback(name, change, event_time):
        print('{} edge on {} at {}'.format(change, name, event_time))
        if change == 'rising':
            if name == 'port0/line0':
                queue_0.put((event_time-100, 100))
            elif name == 'port0/line1':
                queue_1.put((event_time-100, 100))

    def ai_epoch_callback(name, start, duration, data):
        print('Epoch {} acquired with {} samples from {}' \
              .format(start, data.shape, name))

    queue_0 = Queue.Queue()
    queue_1 = Queue.Queue()
    engine = Engine()

    # Set the polling interval to a high value to minimize clutter on the scren.
    engine.hw_ao_monitor_period = 5
    engine.hw_ai_monitor_period = 5
    engine.configure_hw_ai(1e3, '/{}/ai0:2'.format(device), (-10, 10))
    engine.configure_hw_ao(1e3, '/{}/ao0'.format(device), (-10, 10))
    engine.configure_et('/{}/port0/line0:7'.format(device), 'ao/SampleClock')

    for name in ('ai0', 'ai1', 'ai2'):
        channel = '{}/{}'.format(device, name) 
        engine.register_ai_callback(partial(ai_callback, name), channel)

    # Here, we don't bother providing the name of the channel because this type
    # of callback only supports one task.
    engine.register_ao_callback(ao_callback, 'Dev1/ao0')
    engine.register_et_callback(partial(et_callback, 'port0/line0'),
                                '{}/port0/line0'.format(device))
    engine.register_et_callback(partial(et_callback, 'port0/line1'),
                                '{}/port0/line1'.format(device))
    engine.register_ai_epoch_callback(partial(ai_epoch_callback, 'ai0'),
                                      '{}/ai0'.format(device), queue_0)
    engine.register_ai_epoch_callback(partial(ai_epoch_callback, 'ai1'),
                                      '{}/ai1'.format(device), queue_1)

    queue_0.put((0, 100))
    queue_0.put((15, 100))
    queue_0.put((55, 100))
    queue_1.put((60, 150))

    engine.start()
    raw_input('Demo running. Hit enter to exit.\n')

if __name__ == '__main__':
    import sys
    main(sys.argv[1])
