import queue
import time
from threading import Thread


class TickerThread(Thread):
    """Puts a token on its queue every ``period`` seconds."""

    def __init__(self, q=None, period=1.0):
        Thread.__init__(self)
        self.daemon = True

        self.__Q = q if q is not None else queue.Queue(maxsize=0)
        self.__period = float(period)

    def run(self):
        # Scheduled against the monotonic clock so ticks don't drift over time
        next_tick = time.monotonic() + self.__period
        while True:
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            next_tick += self.__period
            self.__Q.put(0)

    def get_q(self):
        return self.__Q
