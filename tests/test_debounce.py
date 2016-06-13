import unittest
import numpy as np

from daqengine.ni import extract_edges


def test_extract_edges():

    def accumulator(name, edge, ts):
        edges.append(edge)
        timestamps.append(ts)

    dio = np.array([
        [0, 1, 0, 0, 0, 0], # 5
        [1, 1, 0, 1, 1, 1], # 11
        [1, 1, 1, 1, 1, 1], # 17
        [0, 0, 0, 0, 1, 0], # 23
        [0, 0, 0, 0, 0, 1], # 29
        [1, 1, 1, 0, 0, 0], # 35
        [0, 0, 0, 0, 0, 0], # 41
    ])


    expected = {
        (0, 1): {
            'edges': ['rising',  'falling']*5,
            'timestamps':  [1, 2, 6, 8, 9, 18, 22, 23, 29, 33],
        },
        (0, 2): {
            'edges': ['rising', 'falling', 'rising', 'falling'],
            'timestamps':  [6, 18, 29, 33]
        },
        (0, 3): {
            'edges': ['rising', 'falling', 'rising', 'falling'],
            'timestamps':  [9, 18, 29, 33]
        },
        (1, 1): {
            'edges': ['falling', 'rising']*5 + ['falling'],
            'timestamps':  [0, 1, 2, 6, 8, 9, 18, 22, 23, 29, 33],
        },
        (1, 2): {
            'edges': ['falling', 'rising', 'falling', 'rising', 'falling'],
            'timestamps':  [2, 6, 18, 29, 33]
        },
        (1, 3): {
            'edges': ['falling', 'rising', 'falling', 'rising', 'falling'],
            'timestamps':  [2, 9, 18, 29, 33]
        },
    }

    for initial_state in (0, 1):
        for min_samples in (1, 2, 3):
            edges = []
            timestamps = []
            extractor = extract_edges(['a'],
                                      initial_states=[initial_state],
                                      min_samples=min_samples,
                                      target=accumulator)
            for line in dio:
                extractor.send([line])

            assert expected[initial_state, min_samples]['edges'] == edges
            assert expected[initial_state, min_samples]['timestamps'] \
                == timestamps
