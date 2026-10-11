"""Run from VS Code terminal: python main.py (no drone hardware connections)."""
from pathlib import Path
import argparse
import json
import sys
from datetime import datetime
from simulator import Simulator, SCENARIOS
from weather import climate_weather, fetch_live

BASE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description='Columbia quadrotor delivery simulation — no real-flight authorization')
    parser.add_argument('--config',default=str(BASE/'config.json'))
    parser.add_argument('--scenario',choices=list(SCENARIOS),default='nominal')
    parser.add_argument('--weather',choices=['climate','live'],default='climate')
    parser.add_argument('--month',type=int,choices=range(1,13))
    parser.add_argument('--distance-miles',type=float,help='Virtual one-way distance east; 3 miles by default')
    parser.add_argument('--headless',action='store_true',help='Run without 3D window; export CSV/JSON and dashboard PNG')
    parser.add_argument('--snapshot-at',type=float,help='Stop at this simulated second; useful for inspection')
    parser.add_argument('--output',help='Output directory (default outputs/<time>_<scenario>)')
    args=parser.parse_args()
    config=json.loads(Path(args.config).read_text())
    if args.month: config['weather']['month']=args.month
    if args.distance_miles is not None: config['mission']['site_a_offset_east_north_m']=[args.distance_miles*1609.344,0]
    config['weather']['mode']=args.weather
    try:
        if args.weather=='live':
            weather,stats=fetch_live(config,BASE)
            print('Recent hourly sample statistics (NOT climate normals):',json.dumps(stats,indent=2))
        else: weather=climate_weather(config,BASE)
        sim=Simulator(config,weather,args.scenario)
    except Exception as exc:
        print(f'Cannot initialize requested simulation: {exc}',file=sys.stderr)
        print('No automatic weather fallback. Use --weather climate for an explicitly offline reference scenario.',file=sys.stderr)
        return 2
    print('SIMULATION ONLY. Real flight authorization: NOT VERIFIED. STL is geometry, not performance data.')
    print(f'Weather: {weather.source}; {weather.temperature_C:.1f} C; mean surface wind {weather.wind_speed_m_s:.2f} m/s')
    print(f'Round-trip planning energy {sim.planned_energy_Wh:.1f} Wh + reserve {sim.reserve_Wh:.1f} Wh')
    out=Path(args.output) if args.output else BASE/'outputs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.scenario)
    if args.headless:
        import matplotlib
        matplotlib.use('Agg')
        while not sim.done and (args.snapshot_at is None or sim.t<args.snapshot_at): sim.step()
        sim.save(out)
        from viewer import Viewer
        view=Viewer(sim,BASE,out,interactive=False)
        view.draw(); view.fig.savefig(out/'dashboard.png',dpi=135,facecolor=view.fig.get_facecolor())
    else:
        from viewer import Viewer
        view=Viewer(sim,BASE,out)
        view.show()
        sim.save(out)
    print(json.dumps(sim.summary(),indent=2))
    print('Outputs:',out.resolve())
    return 0


if __name__=='__main__':
    raise SystemExit(main())
