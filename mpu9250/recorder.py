"""CSV session recorder: samples interval averages and appends them to disk."""

import csv
import threading
import time

FIELDS = ['timestamp', 'g1', 'g2', 'g3', 'a1', 'a2', 'a3',
          'm1', 'm2', 'm3', 'temp', 'n', 'nm', 'dt_ms', 'dtm_ms', 'error']


class Recorder:
    """
    Periodically calls ``mpu.get_avg()`` and appends one CSV row per interval.

    ``get_avg()`` resets the driver's accumulators, so the recorder must be
    the only consumer calling it — the streamer reads instantaneous data and
    does not interfere.
    """

    def __init__(self, mpu, path, interval=0.2):
        self.__mpu = mpu
        self.__path = path
        self.__interval = float(interval)
        self.__stop = threading.Event()
        self.__thread = None
        self.rows = 0

    def __run(self):
        with open(self.__path, 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(FIELDS)
            next_tick = time.monotonic() + self.__interval
            while not self.__stop.is_set():
                delay = next_tick - time.monotonic()
                if delay > 0:
                    self.__stop.wait(delay)
                    if self.__stop.is_set():
                        break
                next_tick += self.__interval

                d = self.__mpu.get_avg()
                writer.writerow([
                    d.T.isoformat(),
                    f'{float(d.G1):.6f}', f'{float(d.G2):.6f}', f'{float(d.G3):.6f}',
                    f'{float(d.A1):.6f}', f'{float(d.A2):.6f}', f'{float(d.A3):.6f}',
                    f'{float(d.M1):.6f}', f'{float(d.M2):.6f}', f'{float(d.M3):.6f}',
                    f'{float(d.Temp):.3f}',
                    d.N, d.NM, f'{float(d.DT):.1f}', f'{float(d.DTM):.1f}',
                    d.MsgError or '',
                ])
                self.rows += 1
                fh.flush()

    def start(self):
        self.__thread = threading.Thread(target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()
        return self

    def stop(self):
        self.__stop.set()
        if self.__thread is not None:
            self.__thread.join(timeout=5)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
