import pytest

from mpu9250.report import analyze, load_session, render_markdown

CSV = """timestamp,g1,g2,g3,a1,a2,a3,m1,m2,m3,temp,n,nm,dt_ms,dtm_ms,error
2026-07-14T12:00:00,10.0,0.0,0.0,0.0,0.0,1.0,22.0,0.0,-31.0,36.0,10,5,1000.0,1000.0,
2026-07-14T12:00:01,-10.0,0.0,0.0,0.0,0.0,1.0,22.0,0.0,-31.0,36.5,10,5,1000.0,1000.0,
2026-07-14T12:00:02,0.0,0.0,0.0,0.6,0.0,0.8,22.0,0.0,-31.0,37.0,10,5,1000.0,1000.0,
2026-07-14T12:00:03,0.0,0.0,0.0,0.0,0.0,1.0,22.0,0.0,-31.0,37.5,10,5,1000.0,1000.0,MPU9250 Error: No new magnetometer values
"""


@pytest.fixture
def session(tmp_path):
    path = tmp_path / 'session.csv'
    path.write_text(CSV)
    return load_session(path)


class TestAnalyze:
    def test_session_metrics(self, session):
        a = analyze(session)

        assert a['rows'] == 4
        assert a['duration_s'] == pytest.approx(3.0)
        assert a['row_rate_hz'] == pytest.approx(1.0)
        assert a['samples'] == 40
        assert a['errors'] == 1

    def test_channel_stats(self, session):
        a = analyze(session)

        g1 = a['channels']['g1']
        assert g1['mean'] == pytest.approx(0.0)
        assert g1['min'] == -10.0
        assert g1['max'] == 10.0
        assert g1['rms'] == pytest.approx(7.071, abs=0.001)

    def test_motion_metrics(self, session):
        a = analyze(session)

        # two rows at |10|°/s for 1 s each
        assert a['rotation_g1_deg'] == pytest.approx(20.0)
        assert a['still_pct'] == pytest.approx(50.0)
        assert a['peak_accel_g'] == pytest.approx(1.0, abs=0.01)
        assert a['mag_plausible'] is True
        assert a['mag_mean_ut'] == pytest.approx(38.0, abs=0.1)

    def test_empty_file_raises(self, tmp_path):
        path = tmp_path / 'empty.csv'
        path.write_text('timestamp,g1\n')
        with pytest.raises(ValueError):
            load_session(path)


class TestRenderMarkdown:
    def test_report_contains_the_key_numbers(self, session, tmp_path):
        md = render_markdown('session.csv', session, analyze(session))

        assert '# sextante session report' in md
        assert '| Duration | 3.0 s |' in md
        assert '| Gyro X | °/s |' in md
        assert 'plausible Earth field' in md
        assert '| Still' in md


CSV_BARO = """timestamp,g1,g2,g3,a1,a2,a3,m1,m2,m3,temp,n,nm,dt_ms,dtm_ms,press_pa,baro_temp,alt_m,error
2026-07-14T12:00:00,0.0,0.0,0.0,0.0,0.0,1.0,22.0,0.0,-31.0,36.0,10,5,1000.0,1000.0,101325.0,24.0,0.00,
2026-07-14T12:00:01,0.0,0.0,0.0,0.0,0.0,1.0,22.0,0.0,-31.0,36.0,10,5,1000.0,1000.0,100653.0,24.1,56.10,
2026-07-14T12:00:02,0.0,0.0,0.0,0.0,0.0,1.0,22.0,0.0,-31.0,36.0,10,5,1000.0,1000.0,101000.0,24.2,27.20,
"""


class TestBaroSession:
    def test_baro_channels_and_altitude_range(self, tmp_path):
        path = tmp_path / 'baro.csv'
        path.write_text(CSV_BARO)
        session = load_session(path)
        a = analyze(session)

        assert a['channels']['press_pa']['mean'] == pytest.approx(1009.93, abs=0.1)  # hPa
        assert a['alt_min_m'] == pytest.approx(0.0)
        assert a['alt_max_m'] == pytest.approx(56.1)

        md = render_markdown(path, session, a)
        assert '| Pressure | hPa |' in md
        assert 'Altitude range | 0.0 – 56.1 m (Δ 56.1 m)' in md

    def test_sessions_without_baro_have_no_baro_rows(self, session):
        a = analyze(session)

        assert 'press_pa' not in a['channels']
        assert 'alt_min_m' not in a

        md = render_markdown('s.csv', session, a)
        assert 'Pressure' not in md
        assert 'Altitude range' not in md
