import time
from daqengine.ni import Engine


def main(device):
    '''
    Demonstrates the use of software-timed analog outputs and digital outputs.
    Software-timed outputs change the state (or value) of an output to the new
    level as soon as requested by the program.

    A common use-case for software-timed digital outputs is to generate a TTL
    (i.e., a square pulse) to trigger something (e.g., a water pump). I have
    included convenience functions (`fire_sw_do`) to facilitate this.

    I also have included name aliases. NI-DAQmx refers to the lines using the
    arcane syntax (e.g., 'Dev1/ao0' or 'Dev1/port0/line0'). However, it is
    probably easier (and generates more readable programs), if we can assign
    aliases to these lines. For example, if 'Dev1/ao0' is connected to a
    Coulbourn shock controller which uses the analog signal to control the
    intensity of the shock, we may want to call that output 'shock_level'.
    Likewise, if 'Dev1/port0/line1' is connected to the trigger port of a pellet
    dispenser, we probably want to call that line 'food_dispense'.
    '''
    engine = Engine()
    # Configure four digital outputs and give them the aliases 'a', 'b', 'c',
    # and 'd'.
    engine.configure_sw_do('{}/port0/line0:3'.format(device), 
                           ['a', 'b', 'c', 'd'],
                           initial_state=[1, 0, 1, 1])
    # Configure two analog otuputs and give them aliases (i.e., assume ao0
    # controls shock level and ao1 controls pump rate).
    engine.configure_sw_ao('{}/ao0:1'.format(device), (-10, 10),
                           ['shock_level', 'pump_rate'])
    time.sleep(1)

    # You can connect ao0 to ai0 and use the analog input panel on the MAX test
    # panels to observe the changes to the analog output.
    engine.set_sw_ao('shock_level', 5)
    engine.set_sw_ao('pump_rate', 4)

     # You can connect the digital lines for port0 to port1 and then monitor the
     # changes on port1 using the digital panel on the MAX test panels.
    time.sleep(1)
    engine.write_sw_do([0, 0, 0, 0])
    engine.start()
    engine.set_sw_do('a', 1)
    time.sleep(0.25)
    engine.set_sw_do('b', 1)
    time.sleep(0.25)
    engine.set_sw_do('c', 1)
    time.sleep(0.25)
    engine.set_sw_do('a', 0)
    time.sleep(0.25)
    engine.set_sw_do('b', 0)
    time.sleep(0.25)
    engine.set_sw_do('c', 0)
    time.sleep(0.25)

    # Generate a TTL pulse of varying duration (in sec).
    engine.fire_sw_do('a', 0.25)
    engine.fire_sw_do('b', 0.5)
    engine.fire_sw_do('c', 1)
    time.sleep(1)


if __name__ == '__main__':
    import sys
    main(sys.argv[1])
