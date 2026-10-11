"""Regenerate presentation screenshots with the actual supplied STL (no GUI)."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from simulator import Simulator
from weather import climate_weather
from viewer import Viewer

BASE=Path(__file__).resolve().parent


def main():
    out=BASE/'outputs'/'previews'; out.mkdir(parents=True,exist_ok=True)
    c=json.loads((BASE/'config.json').read_text())
    for name,times in [('gust',[190]),('nominal',[499,503])]:
        sim=Simulator(c,climate_weather(c,BASE),name)
        view=Viewer(sim,BASE,out,interactive=True)
        for target in times:
            while sim.t<target and not sim.done: sim.step()
            view.draw(); view.fig.savefig(out/f'{name}_{target}s.png',dpi=135,facecolor=view.fig.get_facecolor())
        plt.close(view.fig)
    print('Saved previews:',out)


if __name__=='__main__': main()
