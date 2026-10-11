"""Desktop 3D dashboard; NumPy + Matplotlib only, works from VS Code."""
from pathlib import Path
import textwrap
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from mesh import prepare_mesh
from physics import rotation, rpy

BG='#09121f'; PANEL='#101e2e'; FG='#deebf7'; MUTED='#88a4bd'
CYAN='#54d7e9'; GREEN='#92e6b0'; ORANGE='#ffb55a'; RED='#ff707a'


def box_faces(center,size):
    c=np.asarray(center); sx,sy,sz=np.asarray(size)/2
    v=c+np.array([[-sx,-sy,-sz],[sx,-sy,-sz],[sx,sy,-sz],[-sx,sy,-sz],
                  [-sx,-sy,sz],[sx,-sy,sz],[sx,sy,sz],[-sx,sy,sz]])
    return v[np.array([[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])]


class Viewer:
    def __init__(self,sim,base,output,interactive=True):
        self.sim=sim; self.base=Path(base); self.output=Path(output); self.interactive=interactive
        self.mesh=prepare_mesh(sim.config,base); self.paused=False
        self.rate=sim.config['simulation']['playback_speed']; self.last_clock=time.perf_counter(); self.accumulator=0
        self.last_save_status=''; self.buttons=[]; self.anim=None
        plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'text.color':FG,
                             'axes.labelcolor':MUTED,'xtick.color':MUTED,'ytick.color':MUTED,
                             'axes.edgecolor':'#2a4258','savefig.facecolor':BG})
        self.fig=plt.figure(figsize=(16,9),facecolor=BG)
        if self.fig.canvas.manager is not None:
            self.fig.canvas.manager.set_window_title('Columbia | Autonomous Delivery Flight Lab — SIMULATION ONLY')
        self.fig.text(.035,.954,'COLUMBIA  /  AUTONOMOUS DELIVERY FLIGHT LAB',fontsize=19,weight='bold')
        self.header=self.fig.text(.035,.916,'',fontsize=10,color=CYAN)
        self.fig.text(.965,.955,'ENGINEERING PROTOTYPE',ha='right',fontsize=9,color=MUTED)
        self.fig.text(.965,.918,'REAL FLIGHT: NOT AUTHORIZED BY THIS TOOL',ha='right',fontsize=8,color=ORANGE)
        self.route=self.fig.add_axes([.025,.405,.455,.475],projection='3d',facecolor=BG)
        self.close=self.fig.add_axes([.49,.405,.285,.475],projection='3d',facecolor=BG)
        # Matplotlib uses painter sorting, not a depth buffer. Explicit ordering
        # keeps the landing pad behind the craft and the symbolic cargo visible.
        self.close.computed_zorder=False
        for ax in (self.route,self.close):
            ax.tick_params(labelsize=7,pad=0)
            for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
                axis.set_pane_color((.055,.10,.16,1)); axis._axinfo['grid']['color']=(.16,.24,.32,.45)
            ax.view_init(elev=22,azim=-67)
        self.route.set_title('MISSION CORRIDOR  |  virtual flat terrain',loc='left',pad=12,fontsize=10)
        self.route.set_xlabel('East (km)',labelpad=0); self.route.set_ylabel('North (km)',labelpad=0); self.route.set_zlabel('AGL (m)',labelpad=1)
        site=sim.site; length=np.linalg.norm(site); m=sim.config['mission']; cruise=m['cruise_altitude_agl_m']
        xs=np.array([0,0,site[0]*.90,site[0],site[0]])/1000
        ys=np.array([0,0,site[1]*.90,site[1],site[1]])/1000
        self.route.plot(xs,ys,[.35,cruise,cruise,m['approach_altitude_agl_m'],.35],ls='--',color=MUTED,lw=1,label='planned')
        self.path,=self.route.plot([],[],[],color=CYAN,lw=1.8,label='flown')
        self.marker,=self.route.plot([],[],[],marker='o',color=ORANGE,ms=7,ls='')
        self.route.scatter([0,site[0]/1000],[0,site[1]/1000],[0,0],marker='s',s=50,c=[GREEN,ORANGE])
        self.route.text(0,0,6,'HUB',color=GREEN,size=8); self.route.text(site[0]/1000,site[1]/1000,6,'SITE A',color=ORANGE,size=8)
        margin=m['geofence_margin_m']/1000
        xl=(min(0,site[0]/1000)-margin,max(0,site[0]/1000)+margin)
        yl=(min(0,site[1]/1000)-margin,max(0,site[1]/1000)+margin)
        self.route.set_xlim(*xl); self.route.set_ylim(*yl); self.route.set_zlim(0,max(90,cruise*1.3))
        self.route.set_box_aspect((3.5,1,1))
        self.route.plot([xl[0],xl[1],xl[1],xl[0],xl[0]],[yl[0],yl[0],yl[1],yl[1],yl[0]],[0]*5,color='#39566d',lw=.8)
        self.route.text2D(.02,.0,'Horizontal/vertical scales differ • markers enlarged • no mapped airspace',transform=self.route.transAxes,color=MUTED,size=7)
        self.close.set_title('YOUR STL  |  attitude + landed delivery',loc='left',pad=12,fontsize=10)
        self.close.set_xlabel(''); self.close.set_ylabel(''); self.close.set_zlabel('AGL (m)',labelpad=0)
        self.close.set_xticks([-1,0,1]); self.close.set_yticks([-1,0,1])
        self.close.set_xlim(-1.1,1.1); self.close.set_ylim(-1.1,1.1); self.close.set_zlim(0,2.0)
        self.close.set_box_aspect((1,1,.9)); self.close.view_init(elev=23,azim=-48)
        normals=np.cross(self.mesh[:,1]-self.mesh[:,0],self.mesh[:,2]-self.mesh[:,0]); normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-9)
        lighting=.45+.55*np.abs(normals@np.array([.3,-.4,.866]))
        colors=np.column_stack([lighting*.5,lighting*.72,lighting*.85,np.ones(len(lighting))])
        self.body=Poly3DCollection(self.mesh,facecolors=colors,edgecolor='none',zsort='average',zorder=2)
        self.close.add_collection3d(self.body)
        self.cargo=Poly3DCollection(box_faces([0,0,0],[.22,.15,.14]),facecolors=ORANGE,edgecolors='#7e522a',linewidths=.5,zorder=3)
        self.close.add_collection3d(self.cargo)
        self.pad=Poly3DCollection([np.zeros((4,3))],facecolors='#315b50',edgecolors=GREEN,alpha=.25,zorder=0)
        self.close.add_collection3d(self.pad)
        self.wind_lines=Line3DCollection([[[0,0,0],[0,0,0]]],colors=CYAN,linewidths=1.1,alpha=.65,zorder=1); self.close.add_collection3d(self.wind_lines)
        self.rain_lines=Line3DCollection([[[0,0,0],[0,0,0]]],colors='#739ebc',linewidths=.8,alpha=.8,zorder=4); self.close.add_collection3d(self.rain_lines)
        self.vector_artists=[]
        self.close.text2D(.02,.0,'Cyan = wind TO  |  gold = thrust  |  red = weight\nOrange box = symbolic cargo; mesh is a rigid STL.',transform=self.close.transAxes,size=7,color=MUTED)
        self.info=self.fig.add_axes([.80,.39,.18,.485],facecolor=PANEL); self.info.axis('off')
        self.info_title=self.info.text(.04,.96,'LIVE TELEMETRY',fontsize=11,weight='bold',va='top')
        self.telemetry=self.info.text(.04,.90,'',fontsize=9,va='top',family='DejaVu Sans Mono',linespacing=1.65)
        self.diagnostic=self.fig.text(.035,.373,'',fontsize=10,color=ORANGE,va='top')
        self.ax_alt=self.fig.add_axes([.055,.18,.225,.125],facecolor=PANEL)
        self.ax_track=self.fig.add_axes([.365,.18,.225,.125],facecolor=PANEL)
        self.ax_energy=self.fig.add_axes([.675,.18,.28,.125],facecolor=PANEL)
        for ax in (self.ax_alt,self.ax_track,self.ax_energy):
            ax.grid(alpha=.12); ax.tick_params(labelsize=7); ax.set_xlabel('Simulation time (s)',fontsize=8,labelpad=1)
        self.ax_alt.set_title('Altitude / flight phase',fontsize=9,loc='left'); self.ax_alt.set_ylabel('m',fontsize=8)
        self.ax_track.set_title('Cross-track error / wind response',fontsize=9,loc='left'); self.ax_track.set_ylabel('m',fontsize=8)
        self.ax_energy.set_title('Battery remaining / reserve target',fontsize=9,loc='left'); self.ax_energy.set_ylabel('%',fontsize=8)
        self.alt_line,=self.ax_alt.plot([],[],color=CYAN,lw=1.3)
        self.track_line,=self.ax_track.plot([],[],color=ORANGE,lw=1.3)
        self.soc_line,=self.ax_energy.plot([],[],color=GREEN,lw=1.3)
        self.ax_energy.axhline(m['reserve_fraction']*100,color=ORANGE,ls='--',lw=.7); self.ax_energy.set_ylim(0,105)
        self.footer=self.fig.text(.035,.020,'',fontsize=8,color=MUTED)
        if interactive:
            actions=[('Pause',self.pause),('Crosswind',lambda e:sim.inject('gust')),('Headwind',lambda e:sim.inject('headwind')),
                     ('Rain',lambda e:sim.inject('rain')),('Battery fault',lambda e:sim.inject('battery')),
                     ('Return',lambda e:sim.inject('return')),('Save',lambda e:self.save())]
            for i,(label,fn) in enumerate(actions):
                ax=self.fig.add_axes([.035+i*.096,.068,.085,.037])
                button=Button(ax,label,color='#183148',hovercolor='#27536b'); button.label.set_color(FG); button.label.set_fontsize(8); button.on_clicked(fn); self.buttons.append(button)
            ax=self.fig.add_axes([.79,.080,.15,.013],facecolor=PANEL)
            self.speed_slider=Slider(ax,'Speed',1,32,valinit=self.rate,valstep=1,color=CYAN)
            self.speed_slider.label.set_fontsize(8); self.speed_slider.valtext.set_fontsize(8)
            self.speed_slider.on_changed(lambda v:setattr(self,'rate',float(v)))
            self.fig.canvas.mpl_connect('key_press_event',self.key)

    def pause(self,event=None):
        self.paused=not self.paused
        if self.buttons: self.buttons[0].label.set_text('Resume' if self.paused else 'Pause')

    def key(self,event):
        if event.key==' ': self.pause()
        elif event.key=='s': self.save()
        elif event.key=='g': self.sim.inject('gust')
        elif event.key=='r': self.sim.inject('return')

    def save(self):
        self.sim.save(self.output)
        self.fig.savefig(self.output/'dashboard.png',dpi=135,facecolor=BG)
        self.last_save_status='Saved telemetry, event log, configuration and dashboard.'

    def draw(self):
        s=self.sim; d=s.drone; R=rotation(d.q); angles=rpy(R)
        slow=1 if s.slow_phase else self.rate
        self.header.set_text(f'{s.scenario.upper()}   |   {s.state.replace("_"," ")}   |   T+{s.t:06.1f} s   |   {slow:g}x playback'+('   |   PAUSED' if self.paused else ''))
        if s.rows:
            a=s.rows[::max(1,len(s.rows)//1800)]
            self.path.set_data([r['east_m']/1000 for r in a],[r['north_m']/1000 for r in a]); self.path.set_3d_properties([r['altitude_agl_m'] for r in a])
            t=np.array([r['time_s'] for r in a]); self.alt_line.set_data(t,[r['altitude_agl_m'] for r in a]); self.track_line.set_data(t,[r['cross_track_m'] for r in a]); self.soc_line.set_data(t,[r['soc_percent'] for r in a])
            for ax in (self.ax_alt,self.ax_track): ax.relim(); ax.autoscale_view()
            for ax in (self.ax_alt,self.ax_track,self.ax_energy): ax.set_xlim(0,max(30,s.t))
        self.marker.set_data([d.pos[0]/1000],[d.pos[1]/1000]); self.marker.set_3d_properties([d.pos[2]])
        offset=np.array([0,0,d.pos[2]])
        self.body.set_verts(self.mesh@R.T+offset)
        zlo=max(0,d.pos[2]-1.0); self.close.set_zlim(zlo,zlo+2.0)
        nearest=s.site if np.linalg.norm(d.pos[:2]-s.site)<np.linalg.norm(d.pos[:2]) else np.zeros(2)
        delta=nearest-d.pos[:2]
        self.pad.set_visible(np.linalg.norm(delta)<3 and d.pos[2]<2.7)
        self.pad.set_verts([np.array([[-.75,-.75,0],[.75,-.75,0],[.75,.75,0],[-.75,.75,0]])+np.r_[delta,0]])
        if not s.delivered:
            cargo_center=np.array([0.,0.,-.18])
            if s.state=='RELEASE': cargo_center[2]-=.10*min(1,(s.t-s.state_since)/3)
            self.cargo.set_verts(box_faces(cargo_center,[.22,.15,.14])@R.T+offset); self.cargo.set_visible(True)
        else:
            cargo_pos=np.r_[s.delivery_position[:2]-d.pos[:2],.07]
            self.cargo.set_verts(box_faces(cargo_pos,[.22,.15,.14])); self.cargo.set_visible(np.linalg.norm(cargo_pos[:2])<2 and d.pos[2]<3)
        for artist in self.vector_artists: artist.remove()
        self.vector_artists=[]
        for vec,color in [(s.wind*.07,CYAN),(R[:,2]*(d.motor_thrust.sum()/max(d.mass*9.80665,1))*.55,ORANGE),(np.array([0,0,-.55]),RED)]:
            self.vector_artists.append(self.close.quiver(0,0,d.pos[2],*vec,color=color,arrow_length_ratio=.2,linewidth=2,zorder=4))
        w=s.wind; direction=w/max(np.linalg.norm(w),1e-9)
        seg=[]
        for k in range(14):
            p=np.array([((k*.57+s.t*.13*w[0])%2.2)-1.1,((k*.93+s.t*.13*w[1])%2.2)-1.1,zlo+.15+(k*.31%1.7)])
            seg.append([p,p+direction*.23])
        self.wind_lines.set_segments(seg)
        self.rain_lines.set_segments([[np.array([((k*.43)%2.2)-1.1,((k*.71)%2.2)-1.1,zlo+((k*.27-s.t*1.2)%2)]),
                                       np.array([((k*.43)%2.2)-1.05,((k*.71)%2.2)-1.1,zlo+((k*.27-s.t*1.2)%2)-.15])] for k in range(40)] if s.weather.rain_mm_hr>0 else [])
        dist=np.linalg.norm(s.site-d.pos[:2])
        self.telemetry.set_text(f'Altitude    {d.pos[2]:7.1f} m\nGroundspeed {np.linalg.norm(d.vel[:2]):7.2f} m/s\nAirspeed    {np.linalg.norm(d.vel-s.wind):7.2f} m/s\nWind @ UAV  {np.linalg.norm(s.wind):7.2f} m/s\nRoll/pitch {angles[0]:5.1f}/{angles[1]:5.1f} deg\nBattery     {100*d.soc:7.1f} %\nVoltage     {d.voltage:7.2f} V\nPower       {d.power:7.0f} W\nPack temp   {d.battery_temperature:7.1f} C\nAir temp    {s.weather.temperature_C:7.1f} C\nAir density {d.rho:7.3f} kg/m3\nPayload     {d.payload:7.2f} kg\nTo Site A   {dist/1000:7.2f} km\nHome est.   {s.return_energy_Wh:7.1f} Wh\nReserve     {s.reserve_Wh:7.1f} Wh\nDelivered       {"YES" if s.delivered else "NO"}')
        explanation=s.reason
        if s.state in ('OUTBOUND','INBOUND') and not s.aborted:
            explanation=f'Controller corrects wind-induced velocity error using tilt and individual motor thrust. Wind {np.linalg.norm(s.wind):.1f} m/s; requested groundspeed {np.linalg.norm(s.v_ref[:2]):.1f} m/s. Return energy is checked every 0.5 s.'
        self.diagnostic.set_text('\n'.join(textwrap.wrap('AUTONOMY  /  '+explanation,145)[:2]))
        weather_label=s.weather.source+((' | '+s.weather.timestamp) if s.weather.timestamp else '')
        self.footer.set_text(weather_label+'  |  Aircraft parameters UNVALIDATED  |  '+(self.last_save_status or 'Drag 3D views to rotate. Space: pause | G: gust | R: return | S: save'))
        return []

    def frame(self,unused):
        now=time.perf_counter(); elapsed=min(.25,now-self.last_clock); self.last_clock=now
        if not self.paused and not self.sim.done:
            self.accumulator+=elapsed*(1 if self.sim.slow_phase else self.rate)
            dt=self.sim.config['simulation']['dt_s']; steps=0
            while self.accumulator>=dt and not self.sim.done and steps<250:
                was_slow=self.sim.slow_phase; self.sim.step(); self.accumulator-=dt; steps+=1
                if self.sim.slow_phase and not was_slow:
                    self.accumulator=min(self.accumulator,dt); break
        return self.draw()

    def show(self):
        self.draw()
        self.anim=FuncAnimation(self.fig,self.frame,interval=1000/self.sim.config['simulation']['viewer_fps'],cache_frame_data=False)
        plt.show()
