import logging

# Set up a verbose debugger level for tracing execution of code. I generally use
# this inside read/write/poll loops that are executed several times a second.
# This results in quite a bit of logging information being generated. However,
# this logging level is very useful when you  need to debug issues related to
# the communication with the NI-DAQmx hardware
TRACE_LEVEL_NUM = 5
logging.addLevelName(TRACE_LEVEL_NUM, "TRACE")
def trace(self, message, *args, **kws):
    # Yes, logger takes its '*args' as 'args'.
    if self.isEnabledFor(TRACE_LEVEL_NUM):
        self._log(TRACE_LEVEL_NUM, message, args, **kws)
logging.Logger.trace = trace


logging.getLogger(__name__).addHandler(logging.NullHandler())
