#!/usr/bin/env python3
"""Plot thrust and weather telemetry from quadcopter_sim.py.

Examples:
    python3 plot_flight_telemetry.py flight_telemetry.csv
    python3 plot_flight_telemetry.py flight_telemetry.csv --show
    python3 plot_flight_telemetry.py flight_telemetry.csv --output weather_test.png
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_column(rows, name, default=0.0):
    """Read one numeric CSV column, accepting older telemetry files too."""
    values = []
    for row in rows:
        value = row.get(name, default)
        try:
            values.append(float(value))
        except (TypeError, ValueError):
            values.append(default)
    return np.asarray(values, dtype=float)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', nargs='?', default='flight_telemetry.csv',
                        help='Telemetry CSV from quadcopter_sim.py')
    parser.add_argument('--output', help='PNG output path (default: <csv>_plots.png)')
    parser.add_argument('--show', action='store_true',
                        help='Open the figure after saving it')
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.is_file():
        raise SystemExit(f'CSV file not found: {csv_path}')

    with csv_path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit(f'No telemetry rows found in: {csv_path}')

    required = {'time_s', 'T0_N', 'T1_N', 'T2_N', 'T3_N'}
    missing = required - set(rows[0])
    if missing:
        raise SystemExit('This is not a quadcopter telemetry CSV. Missing: '
                         + ', '.join(sorted(missing)))

    time_s = read_column(rows, 'time_s')
    thrusts = np.column_stack([read_column(rows, f'T{i}_N') for i in range(4)])
    total_thrust = (read_column(rows, 'total_thrust_N')
                    if 'total_thrust_N' in rows[0] else thrusts.sum(axis=1))

    weather = np.column_stack([
        read_column(rows, 'weather_Fx'),
        read_column(rows, 'weather_Fy'),
        read_column(rows, 'weather_Fz'),
    ])
    weather_magnitude = (read_column(rows, 'weather_force_N')
                         if 'weather_force_N' in rows[0]
                         else np.linalg.norm(weather, axis=1))
    estimated = np.column_stack([
        read_column(rows, 'estimated_Fx'),
        read_column(rows, 'estimated_Fy'),
        read_column(rows, 'estimated_Fz'),
    ])
    estimated_magnitude = np.linalg.norm(estimated, axis=1)

    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True,
                                constrained_layout=True)
    figure.suptitle(f'Quadcopter telemetry — {csv_path.name}', fontsize=15, fontweight='bold')

    motor_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    for motor, color in enumerate(motor_colors):
        axes[0].plot(time_s, thrusts[:, motor], color=color, linewidth=1.3,
                     label=f'Rotor {motor + 1}')
    axes[0].plot(time_s, total_thrust, color='black', linewidth=2.2,
                 label='Total thrust')
    axes[0].set_ylabel('Thrust (N)')
    axes[0].set_title('Motor thrust commands')
    axes[0].grid(alpha=0.25)
    axes[0].legend(ncol=5, fontsize=9, loc='upper right')

    axes[1].plot(time_s, weather_magnitude, color='#6a3d9a', linewidth=2,
                 label='Applied weather force')
    axes[1].plot(time_s, estimated_magnitude, color='#e31a1c', linewidth=1.8,
                 linestyle='--', label='Controller-estimated disturbance')
    axes[1].set_ylabel('Force magnitude (N)')
    axes[1].set_title('Weather impact: actual versus inferred')
    axes[1].grid(alpha=0.25)
    axes[1].legend()

    component_colors = ['#377eb8', '#4daf4a', '#e41a1c']
    component_names = ['X', 'Y', 'Z']
    for index, (name, color) in enumerate(zip(component_names, component_colors)):
        axes[2].plot(time_s, weather[:, index], color=color, linewidth=1.6,
                     label=f'Weather $F_{name}$')
    axes[2].axhline(0.0, color='black', linewidth=0.8)
    axes[2].set_xlabel('Simulation time (s)')
    axes[2].set_ylabel('Force component (N)')
    axes[2].set_title('Applied weather-force components')
    axes[2].grid(alpha=0.25)
    axes[2].legend(ncol=3)

    output_path = Path(args.output) if args.output else csv_path.with_name(
        f'{csv_path.stem}_plots.png')
    figure.savefig(output_path, dpi=180)
    print(f'Saved plot: {output_path}')

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()

