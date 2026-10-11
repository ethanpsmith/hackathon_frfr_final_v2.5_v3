"""Offline scenario sweep. Each case gets independent logs. No hardware/network."""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse
import csv
import json
from simulator import Simulator,SCENARIOS
from weather import climate_weather

BASE=Path(__file__).resolve().parent


def run(case):
    scenario,distance,dt=case
    c=json.loads((BASE/'config.json').read_text())
    c['simulation']['dt_s']=dt
    if distance is not None: c['mission']['site_a_offset_east_north_m']=[distance*1609.344,0]
    s=Simulator(c,climate_weather(c,BASE),scenario)
    while not s.done: s.step()
    label=f'{scenario}_{distance or 3:g}mi_dt{dt:g}'
    s.save(BASE/'outputs'/'scenario_suite'/label)
    r=s.summary()
    keys=['final_state','delivered','aborted','duration_s','energy_used_Wh','remaining_soc_percent','max_altitude_agl_m','max_cross_track_m']
    return dict(case=label,**{k:r[k] for k in keys})


def main():
    p=argparse.ArgumentParser(); p.add_argument('--workers',type=int,default=2); args=p.parse_args()
    cases=[(name,None,.02) for name in SCENARIOS]+[('nominal',2,.02),('nominal',5,.02),('nominal',None,.01)]
    directory=BASE/'outputs'/'scenario_suite'; directory.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(max_workers=max(1,args.workers)) as pool:
        results=[]
        for result in pool.map(run,cases): results.append(result); print(result,flush=True)
    with (directory/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=results[0].keys()); writer.writeheader(); writer.writerows(results)


if __name__=='__main__': main()
