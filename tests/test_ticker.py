import time

from mpu9250.ticker import TickerThread


class TestTickerThread:
    def test_ticks_at_the_requested_period(self):
        ticker = TickerThread(period=0.02)
        ticker.start()
        time.sleep(0.25)

        count = ticker.get_q().qsize()
        assert 5 <= count <= 20  # nominal ~12; generous margins for scheduler jitter

    def test_each_ticker_gets_its_own_queue(self):
        assert TickerThread().get_q() is not TickerThread().get_q()
