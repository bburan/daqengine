import unittest

from functools import partial
import numpy as np

from daqengine.ni import analog_threshold


def test_analog_threshold():
    def accumulator(name, edge, ts):
        results.append((name, edge, ts))

    analog = np.zeros((2, 100), dtype=np.float)
    analog[0, :5] = 2.5
    analog[0, 5:10] = 5
    analog[0, 11] = 2.5
    analog[0, 12:25] = 3

    analog[1, 45:50] = 5
    analog[1, 50] = 10
    analog[1, 51:55] = 3

    expected = {
        (2.75, 3): [('a', 'rising', 5), ('a', 'falling', 25),
                    ('b', 'rising', 45), ('b', 'falling', 55)],
        (2.75, 2): [('a', 'rising', 5), ('a', 'falling', 10), 
                    ('a', 'rising', 12), ('a', 'falling', 25),
                    ('b', 'rising', 45), ('b', 'falling', 55)],
        (2.5, 2):  [('a', 'rising', 0), ('a', 'falling', 25),
                    ('b', 'rising', 45), ('b', 'falling', 55)],
        (2.5, 3):  [('a', 'rising', 0), ('a', 'falling', 25),
                    ('b', 'rising', 45), ('b', 'falling', 55)],
        (3.1, 2):  [('a', 'rising', 5), ('a', 'falling', 10),
                    ('b', 'rising', 45), ('b', 'falling', 51)],
        (3.1, 3):  [('a', 'rising', 5), ('a', 'falling', 10),
                    ('b', 'rising', 45), ('b', 'falling', 51)],
    }

    for threshold in (2.5, 2.75, 3.1):
        for debounce in (2, 3):
            results = []
            # This demonstrates how thresholding on each channel can be
            # configured independently (e.g., as done in psiexperiment), yet
            # share a common callback.
            a_accumulator = partial(accumulator, 'a')
            b_accumulator = partial(accumulator, 'b')
            extractor = analog_threshold(threshold, debounce, a_accumulator)
            extractor.send(analog[0])
            extractor = analog_threshold(threshold, debounce, b_accumulator)
            extractor.send(analog[1])

            assert set(expected[threshold, debounce]) == set(results)
