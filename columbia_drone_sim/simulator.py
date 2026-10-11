"""Mission state machine, explicit operating gates, events and audit telemetry."""
from dataclasses import asdict
from pathlib import Path
import csv
import json
import math
import copy
import numpy as np
from physics import Aircraft, rotation, rpy
from weather import WindField, density

SCENARIOS = {
    'nominal': [],
    'gust': [(150,'gust'),(195,'clear_wind')],
    'headwind': [(150,'headwind'),(220,'clear_wind')],
    'rain': [(150,'rain')],
    'battery': [(150,'battery')],
    'motor': [(150,'motor')],
    'motor_failure': [(150,'motor_failure')],
    'lost_link': [(150,'lost_link')],
    'navigation_loss': [(150,'navigation_loss')],
    'blocked_pad': [(150,'blocked_pad')],
    'hot': [],
    'fog': []
}


def validate_config(c):
    a,m,w,l,s = (c[k] for k in ['aircraft','mission','weather','limits','simulation'])
    positive = [(a,k) for k in ['dry_mass_including_battery_kg','motor_arm_m','rotor_radius_m',
        'max_thrust_per_motor_N','motor_time_constant_s','battery_capacity_Ah','battery_cells',
        'battery_thermal_capacity_J_K','ground_clearance_body_origin_m']]
    positive += [(m,k) for k in ['cruise_altitude_agl_m','approach_altitude_agl_m','cruise_groundspeed_m_s',
        'max_airspeed_m_s','climb_m_s','descent_m_s','final_descent_m_s','max_tilt_deg','geofence_margin_m']]
    positive += [(w,k) for k in ['reference_height_m','surface_pressure_Pa','turbulence_correlation_s']]
    for obj,k in positive:
        if not np.isfinite(obj[k]) or obj[k]<=0:
            raise ValueError(f'{k} must be positive and finite')
    if not 0<s['dt_s']<=.02: raise ValueError('Use fixed physics dt <= 0.02 s')
    if not 0<=a['payload_kg']: raise ValueError('Payload cannot be negative')
    if not 0<a['battery_initial_soc']<=1: raise ValueError('Initial SOC must be in (0,1]')
    if not 0<a['battery_state_of_health']<=1: raise ValueError('Battery SOH must be in (0,1]')
    if not 0<a['propulsive_efficiency']<=1: raise ValueError('Efficiency must be in (0,1]')
    if not 0<m['reserve_fraction']<1: raise ValueError('Reserve fraction must be in (0,1)')
    if not 1<=w['month']<=12: raise ValueError('Month must be 1..12')
    if m['approach_altitude_agl_m']>=m['cruise_altitude_agl_m']: raise ValueError('Approach altitude must be below cruise')
    if len(m['site_a_offset_east_north_m'])!=2 or np.linalg.norm(m['site_a_offset_east_north_m'])<20: raise ValueError('Site A must be at least 20 m from hub')
    if min(a['inertia_diagonal_kg_m2'])<=0: raise ValueError('Inertia must be positive')
    if a['battery_internal_resistance_ohm']<0: raise ValueError('Battery resistance cannot be negative')
    if not 0<a['battery_mass_included_in_dry_mass_kg']<a['dry_mass_including_battery_kg']: raise ValueError('Battery mass must be included within dry mass')


class Simulator:
    def __init__(self,config,weather,scenario='nominal'):
        validate_config(config)
        self.config=copy.deepcopy(config); self.weather=copy.deepcopy(weather); self.scenario=scenario
        if scenario=='hot': self.weather.temperature_C=36
        if scenario=='fog': self.weather.visibility_m=800
        self.drone=Aircraft(self.config,self.weather); self.field=WindField(self.weather,self.config)
        self.site=np.array(config['mission']['site_a_offset_east_north_m'],dtype=float)
        self.home=np.zeros(2); self.t=0.; self.state='LOADING'; self.state_since=0.
        self.events=[]; self.rows=[]; self.next_log=0.; self.next_safety=0.
        self.scheduled=list(SCENARIOS[scenario]); self.pad_clear=True; self.link_ok=True; self.navigation_ok=True
        self.delivered=False; self.delivery_position=None; self.aborted=False; self.landed_safe=False
        self.reason='Warehouse worker loads and confirms the order-linked destination.'
        self.wind=np.zeros(3); self.v_ref=np.zeros(3); self.yaw=0.; self.target=self.home.copy()
        self.return_energy_Wh=0.; self.peak_altitude=0.; self.peak_airspeed=0.; self.max_cross_track=0.
        self.reserve_Wh=self.drone.capacity_Ah*self.drone.c['battery_cells']*3.7*config['mission']['reserve_fraction']
        self.preflight_issues=self.preflight()
        self.event('INFO','Simulation only: actual flight authorization NOT VERIFIED.')
        if self.preflight_issues:
            self.transition('NO_GO','; '.join(self.preflight_issues))

    @property
    def done(self): return self.state in ('COMPLETE','NO_GO','EMERGENCY_LANDED','CRASHED','TIMEOUT')

    @property
    def slow_phase(self):
        return self.state in ('LOADING','DESCEND_A','LAND_HOME','SHUTDOWN_A','RELEASE','DEPART_A','EMERGENCY_DESCEND')

    def event(self,level,message):
        self.events.append({'t_s':round(self.t,3),'state':self.state,'level':level,'message':message})

    def transition(self,state,reason):
        self.state=state; self.state_since=self.t; self.reason=reason; self.event('STATE',reason)

    def weather_issues(self):
        w,l,m=self.weather,self.config['limits'],self.config['mission']; errors=[]
        if w.rain_mm_hr>l['max_rain_mm_hr']: errors.append('precipitation outside dry-only envelope')
        if not l['min_temperature_C']<=w.temperature_C<=l['max_temperature_C']: errors.append('ambient temperature outside assumed envelope')
        if w.visibility_m<l['min_visibility_m']: errors.append('visibility below 3 statute miles')
        if w.cloud_base_agl_m-m['cruise_altitude_agl_m']<l['min_vertical_cloud_clearance_m']: errors.append('insufficient vertical cloud clearance')
        if w.horizontal_cloud_clearance_m<l['min_horizontal_cloud_clearance_m']: errors.append('insufficient horizontal cloud clearance')
        if not w.daylight: errors.append('outside project daylight-only envelope')
        if w.severe_alert: errors.append('severe weather alert/condition')
        if w.measured_gust_m_s>l['max_instant_wind_m_s']: errors.append('observed gust exceeds assumed limit')
        return errors

    def leg_energy(self,delta,mass,extra_seconds=0):
        m=self.config['mission']; distance=np.linalg.norm(delta)
        direction=delta/max(distance,1e-9); wind=self.field.mean(m['cruise_altitude_agl_m'])[:2]
        speed=m['cruise_groundspeed_m_s']
        airspeed=np.linalg.norm(direction*speed-wind)
        p=self.drone.estimate_power(mass,airspeed)
        # Loss allowance (10%) is an explicit uncalibrated planning margin.
        return 1.10*p*(distance/speed+extra_seconds)/3600

    def preflight(self):
        a,m,l=self.config['aircraft'],self.config['mission'],self.config['limits']; errors=self.weather_issues()
        if m['cruise_altitude_agl_m']>l['max_altitude_agl_m']: errors.append('planned altitude exceeds configured AGL ceiling')
        if m['cruise_groundspeed_m_s']>l['max_groundspeed_m_s']: errors.append('groundspeed exceeds configured ceiling')
        if a['payload_kg']>l['max_payload_kg']: errors.append('payload exceeds project limit')
        if self.drone.mass>=55*.45359237: errors.append('mass is not below the Part 107 55 lb threshold')
        if np.linalg.norm(self.field.mean(m['cruise_altitude_agl_m']))>l['max_mean_wind_at_aircraft_m_s']: errors.append('mean wind at cruise altitude exceeds assumed limit')
        roundtrip=self.leg_energy(self.site,self.drone.mass,60)+self.leg_energy(-self.site,a['dry_mass_including_battery_kg'],60)
        self.planned_energy_Wh=roundtrip
        abort_case=self.leg_energy(self.site,self.drone.mass,60)+self.leg_energy(-self.site,self.drone.mass,60)
        self.loaded_abort_energy_Wh=abort_case
        if roundtrip+self.reserve_Wh>self.drone.available_energy_Wh(): errors.append('estimated round trip plus reserve exceeds available energy')
        elif abort_case+self.reserve_Wh>self.drone.available_energy_Wh(): errors.append('cannot retain reserve for a loaded return if delivery is denied')
        if 4*a['max_thrust_per_motor_N']<1.5*self.drone.mass*9.80665: errors.append('insufficient assumed static thrust margin')
        return errors

    def inject(self,kind):
        if self.done: return
        if kind=='gust':
            self.field.event_vector=np.array([0.,4.5,0.]); self.weather.gust_sigma_m_s=1.1
            msg='Crosswind step and turbulence: track error drives tilt and thrust correction.'
        elif kind=='headwind':
            self.field.event_vector=np.array([-6.,0.,0.]); msg='Headwind step: airspeed-aware speed command and return-energy check.'
        elif kind=='clear_wind':
            self.field.event_vector[:]=0; self.weather.gust_sigma_m_s=self.config['weather']['gust_sigma_m_s']; msg='Wind disturbance removed.'
        elif kind=='rain': self.weather.rain_mm_hr=3; msg='Synthetic rain onset: dry-only mission must abort.'
        elif kind=='battery': self.drone.soc=min(self.drone.soc,.20); msg='Injected battery-state fault: remaining charge set to 20%; not normal aging.'
        elif kind=='motor': self.drone.motor_health[0]=.80; msg='Motor 1 maximum-thrust capability reduced by 20%; mission returns conservatively.'
        elif kind=='motor_failure': self.drone.motor_health[0]=0; msg='Motor 1 failed. Ordinary quadrotor motor-out recovery is NOT guaranteed.'
        elif kind=='lost_link': self.link_ok=False; msg='Command link lost: preprogrammed onboard return while navigation is available.'
        elif kind=='navigation_loss': self.navigation_ok=False; msg='Navigation-confidence fault: terminate mission using idealized local descent, not a real GNSS-denied solution.'
        elif kind=='blocked_pad': self.pad_clear=False; msg='Site A pad occupied: approach clearance will be denied.'
        elif kind=='return': self.abort('Operator-requested simulated return'); return
        else: raise ValueError('Unknown event '+kind)
        self.event('DISTURBANCE',msg); self.reason=msg

    def emergency(self,reason):
        if self.state=='EMERGENCY_DESCEND': return
        self.aborted=True; self.target=self.drone.pos[:2].copy()
        self.transition('EMERGENCY_DESCEND',reason+' Emergency surface is an assumed clear virtual area, not a surveyed real landing site.')

    def abort(self,reason):
        if self.aborted or self.done: return
        self.aborted=True; self.target=self.home.copy()
        self.transition('RETURN_CLIMB',reason+' Package retained unless already delivered; return to hub.')

    def safety(self):
        d,m,l=self.drone,self.config['mission'],self.config['limits']
        self.return_energy_Wh=self.leg_energy(-d.pos[:2],d.mass,75)
        if d.crashed:
            self.transition('CRASHED','Unsafe ground contact / loss of control. This scenario did not recover safely.'); return
        if self.state=='EMERGENCY_DESCEND': return
        if min(d.motor_health)<.1:
            self.emergency('Critical motor failure; attempting descent with possible loss of control.'); return
        if not self.navigation_ok:
            self.emergency('Navigation lost; normal delivery and return navigation stopped.'); return
        if d.soc<.08 or d.voltage<d.c['battery_cutoff_V'] or d.power_limited:
            self.emergency('Critical battery voltage/charge/power.'); return
        if d.pos[2]>l['max_altitude_agl_m'] or np.linalg.norm(d.vel)>l['max_groundspeed_m_s']:
            self.emergency('Configured flight envelope exceeded.'); return
        lo=np.minimum(self.home,self.site)-m['geofence_margin_m']; hi=np.maximum(self.home,self.site)+m['geofence_margin_m']
        if np.any(d.pos[:2]<lo) or np.any(d.pos[:2]>hi):
            self.emergency('Virtual geofence excursion.'); return
        if self.state in ('LOADING','SHUTDOWN_A','RELEASE','DEPART_A'): return
        issues=self.weather_issues()
        if np.linalg.norm(self.field.mean(d.pos[2]))>l['max_mean_wind_at_aircraft_m_s'] or np.linalg.norm(self.wind)>l['max_instant_wind_m_s']:
            issues.append('wind exceeds assumed flight envelope')
        if d.battery_temperature>l['max_battery_temperature_C']: issues.append('battery overheated')
        if np.linalg.norm(d.vel-self.wind)>m['max_airspeed_m_s']+.5: issues.append('airspeed exceeds project envelope')
        if min(d.motor_health)<.99: issues.append('motor-health degradation')
        if not self.link_ok: issues.append('command-link loss')
        if d.available_energy_Wh()<self.return_energy_Wh+self.reserve_Wh: issues.append('projected home energy would consume reserve')
        if issues: self.abort('; '.join(issues))

    def speed_to(self,target,slow=False):
        d,m=self.drone,self.config['mission']; delta=target-d.pos[:2]; dist=np.linalg.norm(delta)
        direction=delta/max(dist,1e-9)
        speed=min(m['cruise_groundspeed_m_s'],math.sqrt(2*1.5*dist),.65*dist)
        if slow: speed=min(speed,2.)
        # Exact horizontal airspeed constraint along intended ground-track direction.
        w=self.field.mean(d.pos[2])[:2]; along=np.dot(w,direction); perpendicular=w-along*direction
        allowable=along+math.sqrt(max(0.,m['max_airspeed_m_s']**2-np.dot(perpendicular,perpendicular)))
        speed=min(speed,max(0.,allowable))
        return direction*speed

    def guidance(self):
        d,m=self.drone,self.config['mission']; state=self.state
        age=self.t-self.state_since; floor=d.c['ground_clearance_body_origin_m']
        enabled=True; z=m['cruise_altitude_agl_m']; vxy=np.zeros(2)
        self.target=self.home if state in ('RETURN_CLIMB','INBOUND','APPROACH_HOME','LAND_HOME') else self.site
        if state=='LOADING':
            enabled=False
            if age>=4: self.transition('CLIMB','Order confirmed; checks passed for virtual mission; climb above hub.')
        elif state in ('CLIMB','RETURN_CLIMB'):
            anchor=self.home if state=='CLIMB' else (self.site if self.delivered and np.linalg.norm(d.pos[:2]-self.site)<5 else d.pos[:2])
            vxy=self.speed_to(anchor,True)
            if d.pos[2]>z-.7:
                self.transition('OUTBOUND' if state=='CLIMB' else 'INBOUND','Cruise altitude reached; follow planned ground track.')
        elif state in ('OUTBOUND','INBOUND','APPROACH_A','APPROACH_HOME'):
            dist=np.linalg.norm(self.target-d.pos[:2]); vxy=self.speed_to(self.target)
            if dist<450:
                z=m['approach_altitude_agl_m']+(m['cruise_altitude_agl_m']-m['approach_altitude_agl_m'])*dist/450
                if state in ('OUTBOUND','INBOUND'):
                    self.transition('APPROACH_A' if state=='OUTBOUND' else 'APPROACH_HOME','Descending approach: reducing altitude and braking toward the pad.')
            if dist<2 and abs(d.vel[2])<1.2 and d.pos[2]<m['approach_altitude_agl_m']+1:
                if self.target is self.site and not self.pad_clear: self.abort('Landing pad occupied; delivery denied')
                else: self.transition('DESCEND_A' if state in ('OUTBOUND','APPROACH_A') else 'LAND_HOME','Pad alignment confirmed; slow vertical descent. Playback switches to 1x.')
        elif state in ('DESCEND_A','LAND_HOME','EMERGENCY_DESCEND'):
            if state=='EMERGENCY_DESCEND':
                self.target=getattr(self,'emergency_target',d.pos[:2].copy())
                self.emergency_target=self.target
            if state=='DESCEND_A' and not self.pad_clear:
                self.abort('Pad became occupied during descent'); return np.zeros(3),True
            vxy=self.speed_to(self.target,True); z=floor-.1
            if d.on_ground and age>1:
                if state=='DESCEND_A': self.transition('SHUTDOWN_A','Touchdown confirmed; stop rotors and hold before package release.')
                elif state=='LAND_HOME':
                    self.landed_safe=True; self.transition('COMPLETE','Returned to hub. '+('Delivery completed.' if self.delivered else 'Mission aborted; package retained.'))
                else:
                    self.landed_safe=True; self.transition('EMERGENCY_LANDED','Landed on assumed virtual emergency surface; recovery required.')
                enabled=False
        elif state=='SHUTDOWN_A':
            enabled=False
            if age>=m['landing_dwell_s'] and max(d.motor_thrust)<.2:
                if not self.pad_clear: self.transition('EMERGENCY_LANDED','Pad occupied after landing; keep motors stopped, retain package and require recovery.')
                else: self.transition('RELEASE','Motors stopped and weight on ground: unlocking landed cargo. No cable.')
        elif state=='RELEASE':
            enabled=False
            if age>=3:
                if not self.pad_clear: self.transition('EMERGENCY_LANDED','Release denied: pad occupied. Remain stopped on ground with package retained.')
                else:
                    d.release(); self.delivered=True; self.delivery_position=d.pos.copy()
                    self.transition('DEPART_A','Package released; confirm separation and clear departure area.')
        elif state=='DEPART_A':
            enabled=False
            if age>=m['release_confirmation_s']:
                if not self.pad_clear: self.reason='Waiting on ground for departure area to clear.'
                elif self.weather_issues(): self.transition('EMERGENCY_LANDED','Weather changed while landed; remain at Site A for recovery.')
                else: self.transition('RETURN_CLIMB','Separation verified; lift off with reduced mass and return to hub.')
        else: enabled=False
        vz=float(np.clip(1.0*(z-d.pos[2]),-m['descent_m_s'],m['climb_m_s']))
        if state in ('DESCEND_A','LAND_HOME','EMERGENCY_DESCEND'):
            vz=max(vz,-m['final_descent_m_s'])
        if enabled and np.linalg.norm(vxy)>.5:
            target_yaw=math.atan2(vxy[1],vxy[0])
            change=(target_yaw-self.yaw+math.pi)%(2*math.pi)-math.pi
            self.yaw+=float(np.clip(change,-.6*self.config['simulation']['dt_s'],.6*self.config['simulation']['dt_s']))
        return np.r_[vxy,vz],enabled

    def step(self):
        if self.done: return
        dt=self.config['simulation']['dt_s']
        while self.scheduled and self.t>=self.scheduled[0][0]:
            _,kind=self.scheduled.pop(0); self.inject(kind)
        self.wind=self.field.step(self.drone.pos[2],dt)
        if self.t>=self.next_safety:
            self.safety(); self.next_safety=self.t+.5
        if self.done: self.log(); return
        self.v_ref,enabled=self.guidance()
        self.drone.step(dt,self.wind,self.v_ref,self.yaw,enabled and not self.done)
        self.t+=dt
        # Contact classification is immediate. Never let the mission state machine
        # label a hard impact as a successful landing before the 2 Hz safety check.
        if self.drone.crashed:
            self.transition('CRASHED','Unsafe impact / loss of control. Delivery failed; no safe recovery demonstrated.')
        self.peak_altitude=max(self.peak_altitude,self.drone.pos[2])
        self.peak_airspeed=max(self.peak_airspeed,np.linalg.norm(self.drone.vel-self.wind))
        u=self.site/np.linalg.norm(self.site)
        cross_track=abs(u[0]*self.drone.pos[1]-u[1]*self.drone.pos[0]); self.max_cross_track=max(self.max_cross_track,cross_track)
        if self.t>=self.next_log: self.log(); self.next_log=self.t+self.config['simulation']['log_interval_s']
        if not self.done and self.t>=self.config['mission']['simulation_timeout_s']: self.transition('TIMEOUT','Simulation time limit reached; mission not complete.')

    def log(self):
        d=self.drone; angles=rpy(rotation(d.q)); u=self.site/np.linalg.norm(self.site)
        row={'time_s':self.t,'state':self.state,'east_m':d.pos[0],'north_m':d.pos[1],'altitude_agl_m':d.pos[2],
             'east_velocity_m_s':d.vel[0],'north_velocity_m_s':d.vel[1],'vertical_velocity_m_s':d.vel[2],
             'groundspeed_m_s':np.linalg.norm(d.vel[:2]),'airspeed_m_s':np.linalg.norm(d.vel-self.wind),
             'wind_east_m_s':self.wind[0],'wind_north_m_s':self.wind[1],'wind_up_m_s':self.wind[2],
             'roll_deg':angles[0],'pitch_deg':angles[1],'yaw_deg':angles[2],
             'soc_percent':100*d.soc,'voltage_V':d.voltage,'current_A':d.current,'power_W':d.power,
             'energy_Wh':d.energy_Wh,'battery_temperature_C':d.battery_temperature,'density_kg_m3':d.rho,
             'payload_kg':d.payload,'return_energy_Wh':self.return_energy_Wh,
             'cross_track_m':u[0]*d.pos[1]-u[1]*d.pos[0],'motor_saturation_fraction':d.saturation}
        row.update({f'motor_{i+1}_thrust_N':v for i,v in enumerate(d.motor_thrust)})
        self.rows.append(row)

    def summary(self):
        checks=self.config['regulatory_review']
        return {'simulation_only':True,'real_flight_authorization':'NOT VERIFIED; no hardware interface exists',
                'scenario':self.scenario,'final_state':self.state,'delivered':self.delivered,'returned_to_hub':self.state=='COMPLETE',
                'aborted':self.aborted,'duration_s':self.t,'energy_used_Wh':self.drone.energy_Wh,
                'remaining_soc_percent':self.drone.soc*100,'max_altitude_agl_m':self.peak_altitude,
                'max_airspeed_m_s':self.peak_airspeed,'max_cross_track_m':self.max_cross_track,
                'planning_roundtrip_Wh':self.planned_energy_Wh,'planning_loaded_abort_Wh':self.loaded_abort_energy_Wh,'reserve_Wh':self.reserve_Wh,
                'preflight_issues':self.preflight_issues,'weather':asdict(self.weather),
                'unverified_regulatory_items':[k for k,v in checks.items() if isinstance(v,bool) and not v],
                'model_status':'Illustrative unvalidated parameters; STL used for visualization only'}

    def save(self,directory):
        directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
        if not self.rows or self.rows[-1]['time_s']!=self.t or self.rows[-1]['state']!=self.state: self.log()
        with (directory/'telemetry.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(self.rows[0])); writer.writeheader(); writer.writerows(self.rows)
        (directory/'events.json').write_text(json.dumps(self.events,indent=2))
        (directory/'summary.json').write_text(json.dumps(self.summary(),indent=2))
        (directory/'config_used.json').write_text(json.dumps(self.config,indent=2))
