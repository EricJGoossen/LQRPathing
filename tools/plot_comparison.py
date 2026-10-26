#!/usr/bin/env python3
"""Animate compare_pure_pursuit: pure pursuit, LQR and LQR's reference driving the same path.

  tools/plot_comparison.py 0.4 0.5 3.2 0.9 0.8 1.9 3.4 2.9 0.6 3.9   # runs ./compare_pure_pursuit
  ./compare_pure_pursuit 0 0 2 0 2 2 | tools/plot_comparison.py -     # or reads its CSV on stdin
  tools/plot_comparison.py --save build/run.gif 0 0 2 0 2 2           # write a GIF instead

Space pauses, left/right step while paused.
"""
import argparse
import os
import subprocess
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse(text):
    """The CSV columns as lists, and the '# name finish ..., rms ...' summary lines."""
    cols, data, summary = None, {}, {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith('#'):
            name, rest = line[1:].split(None, 1)
            summary[name] = rest
        elif cols is None:
            cols = line.split(',')
            data = {col: [] for col in cols}
        else:
            for col, val in zip(cols, line.split(',')):
                data[col].append(float(val))
    if not cols or not data['t']:
        sys.exit('no CSV rows in the comparison output')
    return data, summary


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('waypoints', nargs='+', help="x0 y0 x1 y1 ... in meters, or '-' for CSV on stdin")
    ap.add_argument('--speed', type=float, default=1.0, help='playback speed, 1 = real time')
    ap.add_argument('--save', metavar='PATH', help='write a GIF instead of opening a window')
    args = ap.parse_args()

    waypoints = []
    if args.waypoints == ['-']:
        text = sys.stdin.read()
    else:
        nums = [float(v) for v in args.waypoints]
        if len(nums) < 4 or len(nums) % 2:
            ap.error('need an even number of coordinates, at least two waypoints')
        exe = os.path.join(ROOT, 'compare_pure_pursuit')
        if not os.path.exists(exe):
            sys.exit(f'{exe} not found; run ./build.sh first')
        text = subprocess.run([exe, *args.waypoints], check=True, capture_output=True, text=True).stdout
        waypoints = [nums[i:i + 2] for i in range(0, len(nums), 2)]
    d, summary = parse(text)
    t = d['t']
    n = len(t)

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    # The reference over the whole run traces the planned path.
    ax.plot(d['ref_x'], d['ref_y'], color='0.8', lw=6, solid_capstyle='round', label='path', zorder=1)
    if waypoints:
        ax.plot(*zip(*waypoints), 'o', color='0.5', ms=5, label='waypoints', zorder=2)
    pp_line, = ax.plot([], [], color='tab:orange', lw=2, label='pure pursuit (aruw-mcb)', zorder=3)
    lqr_line, = ax.plot([], [], color='tab:blue', lw=2, label='LQR', zorder=3)
    pp_dot, = ax.plot([], [], 'o', color='tab:orange', ms=9, zorder=4)
    lqr_dot, = ax.plot([], [], 'o', color='tab:blue', ms=9, zorder=4)
    ref_dot, = ax.plot([], [], 'o', color='tab:green', ms=7, label='LQR reference', zorder=5)
    clock = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', family='monospace')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title('\n'.join(f'{name}: {rest}' for name, rest in summary.items()), fontsize=9)
    fig.tight_layout()

    # Frames are the CSV rows; the interval plays them in (scaled) real time.
    dt = (t[-1] - t[0]) / max(1, n - 1)
    interval_ms = max(1, round(1000 * dt / args.speed))
    state = {'k': 0, 'paused': False}

    def draw(k):
        pp_line.set_data(d['pp_x'][:k + 1], d['pp_y'][:k + 1])
        lqr_line.set_data(d['lqr_x'][:k + 1], d['lqr_y'][:k + 1])
        pp_dot.set_data([d['pp_x'][k]], [d['pp_y'][k]])
        lqr_dot.set_data([d['lqr_x'][k]], [d['lqr_y'][k]])
        ref_dot.set_data([d['ref_x'][k]], [d['ref_y'][k]])
        clock.set_text(f't = {t[k]:5.2f} s')
        return pp_line, lqr_line, pp_dot, lqr_dot, ref_dot, clock

    if args.save:
        # A GIF holds at most ~50 fps, so drop rows to stay under it.
        step = max(1, round(0.02 / dt))
        anim = FuncAnimation(fig, draw, frames=range(0, n, step), blit=True)
        anim.save(args.save, writer=PillowWriter(fps=min(50, round(args.speed / (dt * step)))))
        print(args.save)
        return

    def frames():
        while True:
            if not state['paused'] and state['k'] < n - 1:
                state['k'] += 1
            yield state['k']

    def on_key(event):
        if event.key == ' ':
            state['paused'] = not state['paused']
            if state['k'] >= n - 1:
                state['k'], state['paused'] = 0, False
        elif state['paused'] and event.key in ('left', 'right'):
            state['k'] = min(n - 1, max(0, state['k'] + (1 if event.key == 'right' else -1)))

    # Left/right default to the toolbar's view history; take them for stepping.
    for key, binding in (('left', 'keymap.back'), ('right', 'keymap.forward')):
        if key in plt.rcParams[binding]:
            plt.rcParams[binding].remove(key)
    fig.canvas.mpl_connect('key_press_event', on_key)
    anim = FuncAnimation(fig, draw, frames=frames, interval=interval_ms, blit=True,
                         cache_frame_data=False)
    plt.show()
    del anim


if __name__ == '__main__':
    main()
