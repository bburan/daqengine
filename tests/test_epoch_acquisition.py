from Queue import Queue
from uuid import uuid4

import numpy as np
import numpy.testing

from daqengine.ni import extract_epochs


def test_epoch_extraction():

    def assert_equal(expected, accumulated):
        assert len(expected) == len(accumulated)
        for e, a in zip(expected, accumulated):
            assert e[:2] == a[:2]
            np.testing.assert_array_equal(e[-1],  a[-1])

    queue = Queue()
    accumulated = []
    def epoch_acquired_cb(names, start, duration, data):
        accumulated.append((start, duration, data))
        if data is not None:
            print (start, duration, data.shape)
        else:
            print (start, duration, None)
    extractor = extract_epochs(None, queue, epoch_acquired_cb, 100)

    queue.put((10, 10))
    queue.put((5, 15))
    queue.put((300, 25))
    extractor.send(np.arange(100))

    expected = [(10, 10, np.arange(10, 20)),
                (5, 15, np.arange(5, 20))]
    assert_equal(expected, accumulated)

    # Right now the queue.put operations don't actually cause the generator to
    # return existing samples. The generator only checks when new samples are
    # *sent*.
    queue.put((50, 25))
    queue.put((150, 13))
    assert_equal(expected, accumulated)

    # Now we send new samples and this should trigger the return of these
    # samples.
    expected.append((50, 25, np.arange(50, 75)))
    expected.append((150, 13, np.arange(150, 163)))
    extractor.send(np.arange(100, 200))
    assert_equal(expected, accumulated)

    extractor.send(np.arange(200, 400))
    expected.append((300, 25, np.arange(300, 325)))
    assert_equal(expected, accumulated)

    queue.put((0, 100))       # will be lost
    queue.put((300, 25))      # will be drawn from buffer
    queue.put((350, 100))     # will be drawn from buffer plus
                              # next acquisition
    queue.put((350, 125))     # will be drawn from buffer plus
                              # next two acquisitions
    queue.put((350, 150))     # will be drawn from buffer plus
                              # next three acquisitions
    assert_equal(expected, accumulated)

    extractor.send(np.arange(400, 450))
    expected.append((0, 100, None))
    expected.append((300, 25, np.arange(300, 325)))
    expected.append((350, 100, np.arange(350, 450)))
    assert_equal(expected, accumulated)

    extractor.send(np.arange(450, 475))
    expected.append((350, 125, np.arange(350, 475)))
    assert_equal(expected, accumulated)

    extractor.send(np.arange(475, 500))
    expected.append((350, 150, np.arange(350, 500)))
