from daqengine.ni import setup_event_timer, setup_change_detect_callback, mx


def main(device):
    '''
    Demonstrates detection of changes digital state of lines, including an
    accurate event timer using the 10MHz timebase.  To convert the timestamp to
    seconds, divide by 10 MHz.
    '''
    def callback(edge, line, event_time):
        print(('{} edge on {} at {}'.format(edge, line, event_time/10e6)))

    timer_task = setup_event_timer('/{}/ChangeDetectionEvent'.format(device), 
                                   '/{}/Ctr0'.format(device),
                                   '/{}/10MhzRefClock'.format(device))
    cd_task = setup_change_detect_callback('/{}/port0/line0:2'.format(device), 
                                           callback,
                                           timer=timer_task,
                                           names=['spout', 'lick', 'poke'])
    mx.DAQmxStartTask(timer_task)
    mx.DAQmxStartTask(cd_task)
    input('Connect some lines from port1 to port0/line0:2. \n'
              'Toggle state of port 1 lines using MAX. \n'
              'Hit enter to exit.\n')


if __name__ == '__main__':
    import sys
    main(sys.argv[1])
