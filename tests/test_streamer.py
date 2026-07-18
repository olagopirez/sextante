import http.client
import json
import math
import time

import pytest

from mpu9250.demo import DemoMPU
from mpu9250.streamer import StreamHub, create_server


@pytest.fixture
def server():
    httpd, hub = create_server(DemoMPU(), host='127.0.0.1', port=0,
                               rate=100, stream_rate=50, source='demo')
    import threading
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield httpd
    httpd.shutdown()
    hub.stop()
    httpd.server_close()


class TestStreamHub:
    def test_produces_fused_payloads(self):
        hub = StreamHub(DemoMPU(), rate=100, source='demo')
        hub.start()
        time.sleep(0.3)
        hub.stop()

        payload = hub.snapshot()
        assert payload is not None
        assert payload['src'] == 'demo'
        assert len(payload['q']) == 4
        assert sum(c * c for c in payload['q']) == pytest.approx(1.0, abs=1e-3)
        assert len(payload['g']) == len(payload['a']) == len(payload['m']) == 3
        # demo gravity magnitude survives the pipeline
        assert math.sqrt(sum(v * v for v in payload['a'])) == pytest.approx(1.0, abs=0.1)


class TestHTTPServer:
    def test_serves_the_viewer_page(self, server):
        conn = http.client.HTTPConnection(*server.server_address, timeout=5)
        conn.request('GET', '/')
        resp = conn.getresponse()
        body = resp.read().decode()

        assert resp.status == 200
        assert 'SEXTANTE' in body
        assert 'EventSource' in body

    def test_status_endpoint(self, server):
        conn = http.client.HTTPConnection(*server.server_address, timeout=5)
        conn.request('GET', '/status')
        resp = conn.getresponse()

        assert resp.status == 200
        assert json.loads(resp.read()) == {'source': 'demo', 'rate': 100}

    def test_unknown_path_is_404(self, server):
        conn = http.client.HTTPConnection(*server.server_address, timeout=5)
        conn.request('GET', '/nope')
        assert conn.getresponse().status == 404

    def test_events_stream_emits_json_payloads(self, server):
        time.sleep(0.2)  # let the hub produce its first sample
        conn = http.client.HTTPConnection(*server.server_address, timeout=5)
        conn.request('GET', '/events')
        resp = conn.getresponse()
        assert resp.status == 200
        assert resp.getheader('Content-Type') == 'text/event-stream'

        line = b''
        deadline = time.time() + 5
        while time.time() < deadline:
            chunk = resp.fp.readline()
            if chunk.startswith(b'data: '):
                line = chunk
                break
        conn.close()

        assert line.startswith(b'data: ')
        payload = json.loads(line[len(b'data: '):])
        assert payload['src'] == 'demo'
        assert len(payload['e']) == 3


class TestStreamHubBaro:
    def test_payload_includes_barometer_when_attached(self):
        from mpu9250.demo import DemoBaro
        hub = StreamHub(DemoMPU(), rate=100, source='demo', baro=DemoBaro())
        hub.start()
        time.sleep(0.3)
        hub.stop()

        payload = hub.snapshot()
        assert 1000 < payload['press'] < 1020   # hPa
        assert 5 < payload['alt'] < 20
        assert 'btemp' in payload

    def test_payload_has_no_baro_keys_without_one(self):
        hub = StreamHub(DemoMPU(), rate=100, source='demo')
        hub.start()
        time.sleep(0.2)
        hub.stop()

        assert 'press' not in hub.snapshot()
