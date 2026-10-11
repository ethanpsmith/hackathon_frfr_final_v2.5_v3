"""NOAA-based reference environment and optional timestamped NWS observations.

Wind direction is meteorological FROM degrees; vectors are ENU TO m/s.
No silent fallback when a requested live observation is unavailable or invalid.
"""
from dataclasses import dataclass, asdict
import calendar
from datetime import datetime, timezone, timedelta
from pathlib import Path
import json
import math
import urllib.request
import numpy as np


@dataclass
class Weather:
    temperature_C: float
    relative_humidity: float
    pressure_Pa: float
    wind_speed_m_s: float
    wind_from_deg: float
    gust_sigma_m_s: float
    visibility_m: float
    cloud_base_agl_m: float
    horizontal_cloud_clearance_m: float
    rain_mm_hr: float
    daylight: bool
    source: str
    timestamp: str = ''
    severe_alert: bool = False
    measured_gust_m_s: float = 0.0
    notes: str = ''


def from_direction(speed, degrees):
    a = math.radians(degrees)
    return np.array([-speed * math.sin(a), -speed * math.cos(a), 0.0])


def density(temperature_C, rh, pressure_Pa, height_m=0):
    """Moist-air ideal-gas density with simple hydrostatic/lapse-rate adjustment."""
    T = temperature_C + 273.15 - 0.0065 * max(0., height_m)
    p = pressure_Pa * math.exp(-9.80665 * max(0., height_m) / (287.05 * T))
    tc = T - 273.15
    vapor = min(p * 0.1, np.clip(rh, 0, 1) * 611.2 * math.exp(17.67 * tc / (tc + 243.5)))
    return (p - vapor) / (287.05 * T) + vapor / (461.5 * T)


def climate_weather(config, base):
    c, w = config['weather'], json.loads((Path(base) / 'data/columbia_climate.json').read_text())
    return Weather((w['monthly_mean_temperature_F'][c['month'] - 1] - 32) * 5/9,
                   w['wind_reference']['mean_relative_humidity_percent'] / 100,
                   c['surface_pressure_Pa'], w['wind_reference']['mean_speed_mph'] * 0.44704,
                   c['wind_from_deg'], c['gust_sigma_m_s'], c['visibility_m'],
                   c['cloud_base_agl_m'], c['horizontal_cloud_clearance_m'], c['rain_mm_hr'],
                   c['daylight'], 'CLIMATE SCENARIO (not live)', notes=w['interpretation'])


def quantity(obj, field, target):
    q = obj.get(field) or {}
    value, unit = q.get('value'), q.get('unitCode', '')
    if value is None:
        return None
    value = float(value)
    suffix = unit.split(':')[-1]
    if suffix == target:
        return value
    conversions = {('km_h-1','m_s-1'):1/3.6, ('kn','m_s-1'):0.514444,
                   ('mi_h-1','m_s-1'):0.44704, ('hPa','Pa'):100,
                   ('km','m'):1000, ('ft','m'):0.3048, ('m','mm'):1000}
    if (suffix, target) in conversions:
        return value * conversions[(suffix,target)]
    if suffix == 'degF' and target == 'degC':
        return (value - 32) * 5/9
    raise ValueError(f'Unknown weather unit {unit!r} for {field}; expected {target}')


def observation_time(p):
    return datetime.fromisoformat(p['timestamp'].replace('Z', '+00:00')).astimezone(timezone.utc)


def recent_statistics(features):
    """One available report per UTC hour; equal-hour sample mean, not climatology."""
    bins = {}
    for f in sorted(features, key=lambda f:f['properties']['timestamp']):
        p = f['properties']; stamp = observation_time(p)
        bins[stamp.replace(minute=0, second=0, microsecond=0)] = p
    values = {'temperature_C':[], 'wind_speed_m_s':[], 'relative_humidity_percent':[]}
    vectors = []
    for p in bins.values():
        for key, field, unit in [('temperature_C','temperature','degC'),
                                 ('wind_speed_m_s','windSpeed','m_s-1'),
                                 ('relative_humidity_percent','relativeHumidity','percent')]:
            x = quantity(p, field, unit)
            if x is not None:
                values[key].append(x)
        speed, direction = quantity(p,'windSpeed','m_s-1'), quantity(p,'windDirection','degree_(angle)')
        if speed == 0:
            vectors.append(np.zeros(3))
        elif speed is not None and direction is not None:
            vectors.append(from_direction(speed,direction))
    result = {k:{'mean':float(np.mean(v)) if v else None, 'sample_count':len(v)} for k,v in values.items()}
    result['period_start'] = min(bins).isoformat() if bins else None
    result['period_end'] = max(bins).isoformat() if bins else None
    if vectors:
        vec = np.mean(vectors,axis=0)
        result['resultant_wind_from_deg'] = float(math.degrees(math.atan2(-vec[0],-vec[1])) % 360)
    result['method'] = 'One available observation per UTC hour. Missing hours excluded; NOT a long-term normal.'
    return result


def solar_elevation(stamp, lat, lon):
    """Approximate solar position, only for a conservative SIMULATED daylight gate."""
    day = stamp.timetuple().tm_yday
    hour = stamp.hour + stamp.minute/60 + stamp.second/3600
    gamma = 2*math.pi/(366 if calendar.isleap(stamp.year) else 365) * (day-1+(hour-12)/24)
    eq = 229.18*(0.000075+0.001868*math.cos(gamma)-0.032077*math.sin(gamma)
                 -0.014615*math.cos(2*gamma)-0.040849*math.sin(2*gamma))
    decl = (0.006918-0.399912*math.cos(gamma)+0.070257*math.sin(gamma)
            -0.006758*math.cos(2*gamma)+0.000907*math.sin(2*gamma)
            -0.002697*math.cos(3*gamma)+0.00148*math.sin(3*gamma))
    ha = math.radians(((hour*60 + eq + 4*lon) % 1440)/4-180)
    la = math.radians(lat)
    return math.degrees(math.asin(math.sin(la)*math.sin(decl)+math.cos(la)*math.cos(decl)*math.cos(ha)))


def parse_latest(features, config, now=None):
    if not features:
        raise ValueError('NWS returned no observations')
    p = max(features,key=lambda f:f['properties']['timestamp'])['properties']
    now = now or datetime.now(timezone.utc)
    stamp = observation_time(p); age = (now-stamp).total_seconds()/60
    if age < -5 or age > config['weather']['max_observation_age_minutes']:
        raise ValueError(f'Observation age {age:.1f} min is outside allowed freshness bounds')
    required = {'temperature_C':('temperature','degC'), 'relative_humidity':('relativeHumidity','percent'),
                'pressure_Pa':('barometricPressure','Pa'), 'wind_speed_m_s':('windSpeed','m_s-1'),
                'visibility_m':('visibility','m')}
    vals = {k:quantity(p,*v) for k,v in required.items()}
    missing = [k for k,v in vals.items() if v is None]
    if missing:
        raise ValueError('Missing required observation fields: '+', '.join(missing))
    vals['relative_humidity'] /= 100
    direction = quantity(p,'windDirection','degree_(angle)')
    if direction is None and vals['wind_speed_m_s'] > 0:
        raise ValueError('Wind direction unknown or variable; choose an explicit scenario instead')
    gust = quantity(p,'windGust','m_s-1') or vals['wind_speed_m_s']
    layers = p.get('cloudLayers')
    if layers is None:
        raise ValueError('Cloud observations missing')
    bases = [quantity(layer,'base','m') for layer in layers if layer.get('amount') not in ('CLR','SKC')]
    if any(v is None for v in bases):
        raise ValueError('Cloud base unknown')
    # ASOS base is above station, not hub. Adjust station elevation when available.
    elevation = quantity(p,'elevation','m')
    station_elev = elevation if elevation is not None else config['weather']['surface_elevation_m']
    cloud_base = (min(bases)+station_elev-config['weather']['surface_elevation_m']) if bases else 10000.
    desc = p.get('textDescription','').lower()
    present = p.get('presentWeather')
    if present is None:
        raise ValueError('Present-weather information missing')
    weather_words = desc + ' ' + ' '.join(str(item.get('weather','')) for item in present)
    wet = any(word in weather_words.lower() for word in ('rain','drizzle','snow','sleet','ice','freezing','hail','precipitation'))
    precip = quantity(p,'precipitationLastHour','mm')
    # Nonzero rain is sufficient to reject our dry-only envelope. Missing rate is
    # never interpreted as dry when present-weather reports precipitation.
    rain = max(0.01,precip or 0) if wet else (precip or 0)
    w = config['weather']
    return Weather(**vals, wind_from_deg=direction or 0, gust_sigma_m_s=w['gust_sigma_m_s'],
                   cloud_base_agl_m=cloud_base, horizontal_cloud_clearance_m=w['horizontal_cloud_clearance_m'],
                   rain_mm_hr=rain, daylight=solar_elevation(now,w['latitude'],w['longitude']) > 10,
                   source=f'NWS {w["station"]} OBSERVATION SNAPSHOT', timestamp=stamp.isoformat(),
                   measured_gust_m_s=gust, severe_alert=('thunder' in desc or 'tornado' in desc),
                   notes='Station snapshot, not route weather. Gust turbulence and horizontal cloud separation remain scenario assumptions. No forecast/radar/airspace clearance.')


def fetch_live(config, base):
    w = config['weather']; now = datetime.now(timezone.utc)
    def get(url):
        req = urllib.request.Request(url,headers={'User-Agent':w['user_agent'],'Accept':'application/geo+json'})
        with urllib.request.urlopen(req,timeout=15) as response:
            return json.load(response)
    start = (now-timedelta(hours=48)).isoformat().replace('+00:00','Z')
    url = f'https://api.weather.gov/stations/{w["station"]}/observations?start={start}&limit=500'
    observations = get(url)
    features = observations.get('features',[])
    weather = parse_latest(features,config,now)
    alerts = get(f'https://api.weather.gov/alerts/active?point={w["latitude"]},{w["longitude"]}')
    events = [f['properties'].get('event','') for f in alerts.get('features',[])]
    weather.severe_alert |= any(any(s in e.lower() for s in ['thunderstorm','tornado','hurricane','tropical storm','high wind','winter storm']) for e in events)
    cache = {'retrieved_utc':now.isoformat(),'observation_url':url,'weather':asdict(weather),
             'recent_statistics':recent_statistics(features),'active_alerts':events,'raw':observations}
    out = Path(base)/'outputs'; out.mkdir(exist_ok=True)
    (out/'weather_snapshot.json').write_text(json.dumps(cache,indent=2))
    return weather, cache['recent_statistics']


class WindField:
    def __init__(self, weather, config):
        self.weather, self.config = weather, config
        self.rng = np.random.default_rng(config['simulation']['random_seed'])
        self.gust = np.zeros(3)
        self.event_vector = np.zeros(3)

    def mean(self, z):
        w = self.config['weather']
        shear = (max(z,1)/w['reference_height_m']) ** w['wind_shear_exponent']
        return from_direction(self.weather.wind_speed_m_s,self.weather.wind_from_deg)*shear + self.event_vector

    def step(self,z,dt):
        a = math.exp(-dt/self.config['weather']['turbulence_correlation_s'])
        sigma = self.weather.gust_sigma_m_s*np.array([1,1,0.25])
        self.gust = a*self.gust + math.sqrt(1-a*a)*sigma*self.rng.normal(size=3)
        return self.mean(z)+self.gust
