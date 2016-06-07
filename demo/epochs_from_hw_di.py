import pylab as pl
import numpy as np
from daqengine.ni import mx, setup_hw_di, setup_hw_ai, extract_edges


def main():
    '''
    Demonstrates detection of changes on lines configured for hardware-timed
    digital input.
    '''
    fs = 100.0
    accumulated = []

    def ai_callback(samples):
        pass

    def printer(edge, event_time):
        seconds = event_time/fs
        print('{} edge at {} samples (i.e., {:.2f} seconds)' \
              .format(edge, event_time, seconds))

    extractor = extract_edges(0, 1, printer)

    def di_callback(samples):
        accumulated.append(samples)
        extractor.send(samples)

    ai_task = setup_hw_ai(fs, '/Dev1/ai0', (-10, 10), ai_callback, 10, False)
    di_task = setup_hw_di(fs, '/Dev1/port0/line0', di_callback, 10, False)
    mx.DAQmxStartTask(di_task)
    mx.DAQmxStartTask(ai_task)
    raw_input('Demo running. Hit enter to exit.\n')
    mx.DAQmxStopTask(di_task)
    mx.DAQmxStopTask(ai_task)

    pl.plot(np.concatenate(accumulated, axis=-1))
    pl.show()


if __name__ == '__main__':
    main()
