"""Transparent 6-DOF quadrotor model. Coefficients are assumptions, not CFD.
World = east/north/up; body = forward/left/up; quaternion = [w,x,y,z].
"""
import math
import numpy as np
from weather import density

G = 9.80665


def cross(a,b):
    return np.array([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]])


def rotation(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def q_derivative(q,omega):
    w,x,y,z=q; a,b,c=omega
    return .5*np.array([-x*a-y*b-z*c,w*a+y*c-z*b,w*b+z*a-x*c,w*c+x*b-y*a])


def rpy(R):
    return np.rad2deg([math.atan2(R[2,1],R[2,2]),math.asin(np.clip(-R[2,0],-1,1)),math.atan2(R[1,0],R[0,0])])


def battery_current(power_W,ocv_V,resistance):
    if resistance <= 0:
        return power_W / ocv_V, ocv_V, False
    disc=ocv_V**2-4*resistance*power_W
    if disc < 0:
        return ocv_V/(2*resistance),ocv_V/2,True
    # Numerically stable low-current root of P=(Voc-I*R)*I.
    current=2*power_W/(ocv_V+math.sqrt(disc))
    return current,ocv_V-current*resistance,False


class Aircraft:
    def __init__(self,config,weather):
        self.c=config['aircraft']; self.config=config; self.weather=weather
        self.pos=np.array([0.,0.,self.c['ground_clearance_body_origin_m']])
        self.vel=np.zeros(3); self.q=np.array([1.,0.,0.,0.]); self.omega=np.zeros(3)
        self.motor_thrust=np.zeros(4); self.motor_health=np.ones(4)
        self.payload=self.c['payload_kg']; self.update_mass()
        arm=self.c['motor_arm_m']/math.sqrt(2)
        self.motor_positions=np.array([[arm,arm,0],[-arm,arm,0],[-arm,-arm,0],[arm,-arm,0]])
        k=self.c['yaw_torque_per_thrust_m']
        self.mixer=np.vstack([np.ones(4),self.motor_positions[:,1]-self.cg[1],
                              -(self.motor_positions[:,0]-self.cg[0]),k*np.array([1,-1,1,-1])])
        self.mixer_inv=np.linalg.inv(self.mixer)
        self.soc=self.c['battery_initial_soc']; self.battery_temperature=weather.temperature_C
        cold_factor=np.clip(1-.008*max(0.,20-weather.temperature_C),.65,1)
        self.capacity_Ah=self.c['battery_capacity_Ah']*self.c['battery_state_of_health']*cold_factor
        self.voltage=self.c['battery_cells']*(3.3+.9*self.soc)
        self.current=0.; self.power=0.; self.energy_Wh=0.; self.integral=np.zeros(3)
        self.crashed=False; self.on_ground=True; self.power_limited=False; self.saturation=0.
        self.rho=density(weather.temperature_C,weather.relative_humidity,weather.pressure_Pa)
        self.wind=np.zeros(3); self.last_acceleration=np.zeros(3)

    def update_mass(self):
        c=self.c; m0=c['dry_mass_including_battery_kg']; r=np.array(c['payload_offset_body_m'])
        self.mass=m0+self.payload; self.cg=self.payload*r/self.mass
        def parallel(m,x):
            return m*(np.dot(x,x)*np.eye(3)-np.outer(x,x))
        self.inertia=np.diag(c['inertia_diagonal_kg_m2'])+parallel(m0,self.cg)+parallel(self.payload,r-self.cg)
        self.inertia_inv=np.linalg.inv(self.inertia)

    def release(self):
        if not self.on_ground or max(self.motor_thrust) > .2:
            raise RuntimeError('Payload release interlock: require landed and motors stopped')
        self.payload=0.; self.update_mass(); self.integral[:]=0
        self.mixer[1]=self.motor_positions[:,1]-self.cg[1]
        self.mixer[2]=-(self.motor_positions[:,0]-self.cg[0]); self.mixer_inv=np.linalg.inv(self.mixer)

    def available_energy_Wh(self):
        # Conservative nominal-voltage equivalent, not a calibrated battery discharge curve.
        return self.soc*self.capacity_Ah*self.c['battery_cells']*3.7

    def estimate_power(self,mass,airspeed,rho=None,climb=0):
        rho=self.rho if rho is None else rho
        disk=4*math.pi*self.c['rotor_radius_m']**2
        drag=.5*rho*self.c['drag_area_body_m2'][0]*airspeed**2
        T=math.hypot(mass*G,drag)
        induced=T**1.5/math.sqrt(2*rho*disk)
        # Conservative hover-induced-power approximation in cruise (no translational lift).
        return (induced+drag*airspeed+mass*G*max(climb,0))/self.c['propulsive_efficiency']+self.c['rotor_profile_power_W']+self.c['avionics_power_W']

    def controller(self,v_ref,yaw,dt):
        R=rotation(self.q); error=v_ref-self.vel
        self.integral=np.clip(self.integral+error*dt,-3,3)
        acceleration=np.array([1.4,1.4,2.8])*error+.25*self.integral
        acceleration[:2]=np.clip(acceleration[:2],-4,4); acceleration[2]=np.clip(acceleration[2],-3,4)
        desired=acceleration+np.array([0,0,G])
        lateral=np.linalg.norm(desired[:2]); max_lat=desired[2]*math.tan(math.radians(self.config['mission']['max_tilt_deg']))
        if lateral > max_lat:
            desired[:2]*=max_lat/lateral
        z=desired/np.linalg.norm(desired); heading=np.array([math.cos(yaw),math.sin(yaw),0])
        y=cross(z,heading); y/=np.linalg.norm(y); x=cross(y,z)
        Rd=np.column_stack((x,y,z))
        skew=Rd.T@R-R.T@Rd
        eR=.5*np.array([skew[2,1],skew[0,2],skew[1,0]])
        torque=-np.array([6.,6.,2.5])*eR-np.array([2.3,2.3,1.6])*self.omega+cross(self.omega,self.inertia@self.omega)
        total=max(0.,self.mass*np.dot(desired,R[:,2]))
        return self.mixer_inv@np.r_[total,torque]

    def step(self,dt,wind,v_ref,yaw,motors_enabled=True):
        c=self.c; self.wind=np.asarray(wind)
        self.rho=density(self.weather.temperature_C,self.weather.relative_humidity,self.weather.pressure_Pa,self.pos[2])
        R=rotation(self.q)
        demand=self.controller(v_ref,yaw,dt) if motors_enabled else np.zeros(4)
        if not motors_enabled:
            self.integral[:]=0
        voltage_ratio=np.clip(self.voltage/(c['battery_cells']*4.2),0,1.05)
        cap=c['max_thrust_per_motor_N']*(self.rho/1.225)*voltage_ratio**2*self.motor_health
        if self.soc<=0:
            cap[:]=0
        self.saturation=float(np.mean((demand>cap)|(demand<0)))
        target=np.clip(demand,0,cap)
        lag=1-math.exp(-dt/c['motor_time_constant_s'])
        self.motor_thrust+=lag*(target-self.motor_thrust)
        air_body=R.T@(self.vel-self.wind)
        drag_body=-.5*self.rho*np.array(c['drag_area_body_m2'])*air_body*np.abs(air_body)
        F=R@(drag_body+np.array([0,0,self.motor_thrust.sum()]))+np.array([0,0,-self.mass*G])
        torque=(self.mixer@self.motor_thrust)[1:]+cross(-self.cg,drag_body)-np.array(c['angular_damping_Nm_per_rad_s'])*self.omega
        omega_dot=self.inertia_inv@(torque-cross(self.omega,self.inertia@self.omega))
        # Semi-implicit fixed-step integration. Quaternion normalization prevents drift.
        self.omega+=omega_dot*dt
        self.q+=q_derivative(self.q,self.omega)*dt; self.q/=np.linalg.norm(self.q)
        self.last_acceleration=F/self.mass
        self.vel+=self.last_acceleration*dt; self.pos+=self.vel*dt
        floor=c['ground_clearance_body_origin_m']
        self.on_ground=self.pos[2]<=floor
        if self.on_ground:
            if self.vel[2]<-.85 or np.linalg.norm(self.vel[:2])>1.0 or rotation(self.q)[2,2]<math.cos(math.radians(15)):
                self.crashed=True
            self.pos[2]=floor; self.vel[:]=0; self.omega[:]=0
            # Rigid, level pad constraint at safe contact; no compliant landing gear model.
            yaw_now=math.atan2(rotation(self.q)[1,0],rotation(self.q)[0,0])
            self.q=np.array([math.cos(yaw_now/2),0,0,math.sin(yaw_now/2)])
        area=math.pi*c['rotor_radius_m']**2
        induced=float(np.sum(np.maximum(self.motor_thrust,0)**1.5)/math.sqrt(2*self.rho*area))
        drag_power=max(0.,-float(np.dot(drag_body,air_body)))
        climb_power=max(0.,self.mass*G*self.vel[2])
        spinning=float(self.motor_thrust.sum()>.5)
        self.power=(induced+drag_power+climb_power)/c['propulsive_efficiency']+spinning*c['rotor_profile_power_W']+c['avionics_power_W']
        ocv=c['battery_cells']*(3.3+.9*self.soc)
        resistance=c['battery_internal_resistance_ohm']*(1+.02*max(0,20-self.battery_temperature))
        self.current,self.voltage,self.power_limited=battery_current(self.power,ocv,resistance)
        self.soc=max(0.,self.soc-self.current*dt/(self.capacity_Ah*3600))
        chemical_power=self.current*ocv
        self.energy_Wh+=chemical_power*dt/3600
        heat=self.current**2*resistance-c['battery_cooling_W_K']*(self.battery_temperature-self.weather.temperature_C)
        self.battery_temperature+=heat*dt/c['battery_thermal_capacity_J_K']
        if not np.isfinite(np.r_[self.pos,self.vel,self.q,self.omega,self.soc]).all():
            raise FloatingPointError('Non-finite state; simulation stopped')
