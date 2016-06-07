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
    def ao_callback(names, offset, samples):
        print('{} samples needed at {} for {}'.format(samples, offset, names))
        engine.write_hw_ao(np.zeros(samples))

    def ai_callback(names, data):
        print('{} samples acquired from {}'.format(data.shape, names))

    def et_callback(change, line, event_time):
        print('{} edge on {} at {}'.format(change, line, event_time))
        queue.put((event_time-100, 100))

    def ai_epoch_callback(names, start, duration, data):
        print('Epoch {} acquired with {} samples from {}' \
              .format(start, data.shape, names))

    queue = Queue.Queue()
    engine = Engine()
    engine.configure_hw_ai(20e3, '/{}/ai0:4'.format(device), (-10, 10),
                           sync_ao=False)
    engine.configure_hw_ao(20e3, '/{}/ao0'.format(device), (-10, 10))
    engine.configure_et('/{}/port0/line0:7'.format(device), 'ao/SampleClock')

    engine.register_ao_callback(ao_callback)
    engine.register_ai_callback(ai_callback)
    engine.register_et_callback(et_callback)
    engine.register_ai_epoch_callback(ai_epoch_callback, queue)
    queue.put((0, 100))
    queue.put((15, 100))
    queue.put((55, 100))

    engine.start()
    raw_input('Demo running. Hit enter to exit.\n')

if __name__ == '__main__':
    import sys
    main(sys.argv[1])
