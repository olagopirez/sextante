import numpy as nu
from datetime import datetime as dtime


class MPUData:
    def __init__(self, g1=nu.float64(0.0), g2=nu.float64(0.0), g3=nu.float64(0.0),
                 a1=nu.float64(0.0), a2=nu.float64(0.0), a3=nu.float64(0.0),
                 m1=nu.float64(0.0), m2=nu.float64(0.0), m3=nu.float64(0.0),
                 temp=nu.float64(0.0), n=1, nm=1, t=None, tm=None, dt=0, dtm=0,
                 msg_error=None):
        self.G1 = g1
        self.G2 = g2
        self.G3 = g3
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.M1 = m1
        self.M2 = m2
        self.M3 = m3
        self.Temp = temp
        self.T = t if t is not None else dtime.now()
        self.TM = tm if tm is not None else dtime.now()
        self.DT = dt
        self.DTM = dtm
        self.N = n
        self.NM = nm
        self.MsgError = msg_error

    def get_json(self):
        return {
            'G1': self.G1,
            'G2': self.G2,
            'G3': self.G3,
            'A1': self.A1,
            'A2': self.A2,
            'A3': self.A3,
            'M1': self.M1,
            'M2': self.M2,
            'M3': self.M3,
            'Temp': self.Temp,
            'T': self.T,
            'TM': self.TM,
            'DT': self.DT,
            'DTM': self.DTM,
            'N': self.N,
            'NM': self.NM,
            'MsgError': self.MsgError
        }


class MPUCalData:
    def __init__(self,
                 g01=nu.float64(0.0), g02=nu.float64(0.0), g03=nu.float64(0.0),  # Gyro hardware bias
                 a01=nu.float64(0.0), a02=nu.float64(0.0), a03=nu.float64(0.0),  # Accelerometer hardware bias
                 m01=nu.float64(0.0), m02=nu.float64(0.0), m03=nu.float64(0.0),  # Magnetometer hardware bias
                 ms11=nu.float64(1.0), ms12=nu.float64(0.0), ms13=nu.float64(0.0),  # Magnetometer rescaling matrix
                 ms21=nu.float64(0.0), ms22=nu.float64(1.0), ms23=nu.float64(0.0),  # (Only diagonal is used currently)
                 ms31=nu.float64(0.0), ms32=nu.float64(0.0), ms33=nu.float64(1.0)):
        self.G01 = g01
        self.G02 = g02
        self.G03 = g03
        self.A01 = a01
        self.A02 = a02
        self.A03 = a03
        self.M01 = m01
        self.M02 = m02
        self.M03 = m03
        self.Ms11 = ms11
        self.Ms12 = ms12
        self.Ms13 = ms13
        self.Ms21 = ms21
        self.Ms22 = ms22
        self.Ms23 = ms23
        self.Ms31 = ms31
        self.Ms32 = ms32
        self.Ms33 = ms33

    def get_json(self):
        return {
            'G01': self.G01,
            'G02': self.G02,
            'G03': self.G03,
            'A01': self.A01,
            'A02': self.A02,
            'A03': self.A03,
            'M01': self.M01,
            'M02': self.M02,
            'M03': self.M03,
            'Ms11': self.Ms11,
            'Ms12': self.Ms12,
            'Ms13': self.Ms13,
            'Ms21': self.Ms21,
            'Ms22': self.Ms22,
            'Ms23': self.Ms23,
            'Ms31': self.Ms31,
            'Ms32': self.Ms32,
            'Ms33': self.Ms33
        }


class BaroData:
    """One barometer reading: pressure (Pa), temperature (°C), altitude (m)."""

    def __init__(self, pressure=0.0, temp=0.0, altitude=0.0, t=None):
        self.Pressure = pressure
        self.Temp = temp
        self.Altitude = altitude
        self.T = t if t is not None else dtime.now()

    def get_json(self):
        return {
            'Pressure': self.Pressure,
            'Temp': self.Temp,
            'Altitude': self.Altitude,
            'T': self.T,
        }


class GPSData:
    """One GPS state: position (°), speed (km/h), course (°), fix info."""

    def __init__(self, lat=None, lon=None, speed_kmh=0.0, course=0.0,
                 sats=0, hdop=0.0, altitude=0.0, fix=False, t=None):
        self.Lat = lat
        self.Lon = lon
        self.SpeedKmh = speed_kmh
        self.Course = course
        self.Sats = sats
        self.Hdop = hdop
        self.Altitude = altitude
        self.Fix = fix
        self.T = t if t is not None else dtime.now()

    def get_json(self):
        return {
            'Lat': self.Lat,
            'Lon': self.Lon,
            'SpeedKmh': self.SpeedKmh,
            'Course': self.Course,
            'Sats': self.Sats,
            'Hdop': self.Hdop,
            'Altitude': self.Altitude,
            'Fix': self.Fix,
            'T': self.T,
        }
