import sys

from daqengine.ni import Engine


def main(device):
    '''
    Demonstrates thresholding of analog input lines. Also reads from a digital
    line (if you have an external comparator circuit).
    '''
    def ai_callback(names, data):
        print('{} samples from {}'.format(data.shape, names))

    def change_callback(name, change, event_time):
        m = '{} edge on {} at {}'.format(change, name, event_time)
        sys.stdout.write(m + '\n')

    engine = Engine()
    engine.hw_ai_monitor_period = 0.01
    engine.configure_hw_di(1000, '/{}/port0/line0'.format(device),
                           names=['poke_digital'],
                           clock='/{}/Ctr0'.format(device),
                           trigger='ai/StartTrigger')
    engine.configure_hw_ai(1000, '/{}/ai0'.format(device), (0, 5),
                           names=['poke_analog'])
    engine.register_ai_threshold_callback(change_callback, threshold=2.5,
                                          debounce=10)
    #engine.register_ai_callback(ai_callback)
    engine.register_di_change_callback(change_callback, debounce=10)
    engine.start()
    raw_input('Demo running. Hit enter to exit.\n')


if __name__ == '__main__':
    main(sys.argv[1])
