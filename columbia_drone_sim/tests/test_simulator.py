import copy
import json
import math
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone,timedelta
import numpy as np

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE))
from mesh import load_stl,prepare_mesh
from physics import Aircraft,rotation,battery_current,G
from weather import climate_weather,density,from_direction,quantity,parse_latest,recent_statistics,solar_elevation
from simulator import Simulator


class FlightTests(unittest.TestCase):
    def setUp(self):
        self.c=json.loads((BASE/'config.json').read_text()); self.w=climate_weather(self.c,BASE)

    def short_sim(self,scenario='nominal'):
        self.c['mission']['site_a_offset_east_north_m']=[150,0]
        s=Simulator(self.c,self.w,scenario)
        return s

    def run_sim(self,s):
        while not s.done: s.step()
        return s

    def test_stl_bounds_and_scale(self):
        tri=load_stl(BASE/'assets/quad_delivery_vtol.stl')
        self.assertEqual(len(tri),35496)
        np.testing.assert_allclose(np.ptp(tri.reshape(-1,3),axis=0)*.001,[.8660194,.8660194,.468],atol=1e-5)
        self.assertTrue(len(prepare_mesh(self.c,BASE))<len(tri))

    def test_wind_from_convention(self):
        np.testing.assert_allclose(from_direction(5,0),[0,-5,0],atol=1e-10)
        np.testing.assert_allclose(from_direction(5,90),[-5,0,0],atol=1e-10)

    def test_hot_and_humid_density(self):
        self.assertLess(density(35,.5,101325),density(15,.5,101325))
        self.assertLess(density(25,1,101325),density(25,0,101325))
        self.assertLess(density(25,.5,101325,100),density(25,.5,101325,0))

    def test_battery_equation(self):
        i,v,limited=battery_current(1000,24,.035)
        self.assertAlmostEqual(i*v,1000,places=7); self.assertFalse(limited); self.assertLess(v,24)
        self.assertTrue(battery_current(10000,20,.1)[2])

    def test_free_fall_initial_acceleration(self):
        d=Aircraft(self.c,self.w); d.pos[2]=30
        d.step(.01,np.zeros(3),np.zeros(3),0,False)
        self.assertAlmostEqual(d.vel[2],-G*.01,places=8)

    def test_hover_quaternion_and_charge(self):
        d=Aircraft(self.c,self.w); d.pos[2]=20; d.motor_thrust[:]=d.mass*G/4
        for _ in range(200): d.step(.02,np.zeros(3),np.zeros(3),0,True)
        self.assertAlmostEqual(np.linalg.norm(d.q),1,places=10)
        np.testing.assert_allclose(rotation(d.q).T@rotation(d.q),np.eye(3),atol=1e-10)
        self.assertLess(abs(d.pos[2]-20),.2); self.assertLess(d.soc,1); self.assertGreater(d.energy_Wh,0)

    def test_release_interlock_and_mass(self):
        d=Aircraft(self.c,self.w); original=d.mass; inertia=d.inertia.copy()
        d.on_ground=False
        with self.assertRaises(RuntimeError): d.release()
        d.on_ground=True; d.motor_thrust[:]=0; d.release()
        self.assertAlmostEqual(original-d.mass,self.c['aircraft']['payload_kg'])
        self.assertLess(d.inertia[0,0],inertia[0,0])

    def test_complete_delivery_and_return(self):
        s=self.run_sim(self.short_sim())
        self.assertEqual(s.state,'COMPLETE'); self.assertTrue(s.delivered); self.assertFalse(s.aborted)
        self.assertLess(np.linalg.norm(s.drone.pos[:2]),1); self.assertEqual(s.drone.payload,0)
        states=[e['state'] for e in s.events]
        self.assertLess(states.index('SHUTDOWN_A'),states.index('RELEASE'))

    def test_fog_no_go(self):
        s=self.short_sim('fog'); self.assertEqual(s.state,'NO_GO'); self.assertEqual(s.t,0)

    def test_excessive_altitude_rejected(self):
        self.c['mission']['cruise_altitude_agl_m']=130
        self.assertEqual(Simulator(self.c,self.w).state,'NO_GO')

    def test_five_mile_range_not_forced(self):
        self.c['mission']['site_a_offset_east_north_m']=[5*1609.344,0]
        self.assertEqual(Simulator(self.c,self.w).state,'NO_GO')

    def test_rain_abort_retains_package(self):
        s=self.short_sim()
        while s.t<36: s.step()
        s.inject('rain'); self.run_sim(s)
        self.assertTrue(s.aborted); self.assertFalse(s.delivered); self.assertGreater(s.drone.payload,0)
        self.assertEqual(s.state,'COMPLETE')

    def test_blocked_pad_no_release(self):
        s=self.short_sim(); s.pad_clear=False; self.run_sim(s)
        self.assertFalse(s.delivered); self.assertTrue(s.aborted)

    def test_motor_failure_not_claimed_as_success(self):
        s=self.short_sim()
        while s.t<35: s.step()
        s.inject('motor_failure'); self.run_sim(s)
        self.assertTrue(s.aborted); self.assertFalse(s.delivered)
        self.assertEqual(s.state,'CRASHED')

    def test_invalid_dt_rejected(self):
        self.c['simulation']['dt_s']=.5
        with self.assertRaises(ValueError): Simulator(self.c,self.w)

    def sample_observation(self,stamp):
        def q(v,u): return {'value':v,'unitCode':'wmoUnit:'+u}
        p={'timestamp':stamp.isoformat(),'temperature':q(20,'degC'),'relativeHumidity':q(60,'percent'),
           'barometricPressure':q(100500,'Pa'),'windSpeed':q(18,'km_h-1'),'windDirection':q(180,'degree_(angle)'),
           'visibility':q(16000,'m'),'windGust':q(25,'km_h-1'),'cloudLayers':[],
           'textDescription':'Clear','presentWeather':[],'precipitationLastHour':q(0,'mm')}
        return {'properties':p}

    def test_live_parse_units_and_freshness(self):
        now=datetime(2026,10,10,17,tzinfo=timezone.utc); f=self.sample_observation(now-timedelta(minutes=10))
        w=parse_latest([f],self.c,now); self.assertAlmostEqual(w.wind_speed_m_s,5)
        self.assertTrue(w.daylight); self.assertEqual(w.relative_humidity,.6)
        with self.assertRaises(ValueError): parse_latest([f],self.c,now+timedelta(hours=4))

    def test_missing_weather_not_silently_zero(self):
        now=datetime(2026,10,10,17,tzinfo=timezone.utc); f=self.sample_observation(now)
        f['properties']['temperature']['value']=None
        with self.assertRaises(ValueError): parse_latest([f],self.c,now)

    def test_recent_mean_deduplicates_hours(self):
        now=datetime(2026,10,10,17,tzinfo=timezone.utc)
        samples=[self.sample_observation(now+timedelta(minutes=i)) for i in [0,10,20,60]]
        stats=recent_statistics(samples)
        self.assertEqual(stats['wind_speed_m_s']['sample_count'],2)
        self.assertAlmostEqual(stats['wind_speed_m_s']['mean'],5)

    def test_haze_is_not_invented_rain(self):
        now=datetime(2026,10,10,17,tzinfo=timezone.utc); f=self.sample_observation(now)
        f['properties']['textDescription']='Haze'; f['properties']['presentWeather']=[{'weather':'haze'}]
        self.assertEqual(parse_latest([f],self.c,now).rain_mm_hr,0)

    def test_daylight_calculation(self):
        noon=datetime(2026,10,10,17,tzinfo=timezone.utc); midnight=noon+timedelta(hours=12)
        self.assertGreater(solar_elevation(noon,34,-81),30)
        self.assertLess(solar_elevation(midnight,34,-81),0)

    def test_repeatable_seed(self):
        s1=self.short_sim(); s2=self.short_sim()
        for _ in range(100): s1.step(); s2.step()
        np.testing.assert_array_equal(s1.drone.pos,s2.drone.pos)


if __name__=='__main__': unittest.main()
