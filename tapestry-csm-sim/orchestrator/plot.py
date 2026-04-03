"""
plot.py — Tapestry CSM simulation telemetry visualiser.

Reads one or two telemetry CSV files and produces a 4-panel figure
comparing L4 algorithm efficacy across four metrics:

  Panel 1 — fresh_ratio          (accuracy: is the world model current?)
  Panel 2 — fraction_degraded    (consistency: what fraction of the fleet lost quorum?)
  Panel 3 — mean_position_error  (accuracy: how far off are beliefs?)
  Panel 4 — mean_age_ms          (accuracy: how old is the data being used?)
  Panel 5 — min_separation       (safety: how close did elements get?)

Partition events are inferred from drops in fresh_ratio and marked as
shaded regions.  Each panel plots the per-element mean ± std band so
individual element variance is visible without per-element clutter.

Usage
─────
    # Single run
    python plot.py telemetry.csv

    # Compare two algorithms
    python plot.py baseline.csv experimental.csv --labels Baseline Experimental

    # Save without displaying
    python plot.py telemetry.csv --out result.png
"""

import argparse
import sys

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# ── Colour palette ───────────────────────────────────────────────────────────

COLOURS = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0']


# ── Data loading ──────────────────────────────────────────────────────────────

def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df['wall_time_s'] = df['wall_time_s'].astype(float)
    # min_separation may be NaN where no peers existed (sentinel decoded as None)
    df['min_separation'] = pd.to_numeric(df['min_separation'], errors='coerce')
    return df


def per_cycle_stats(df: pd.DataFrame, col: str,
                    smooth_window_s: float = 0.0) -> pd.DataFrame:
    """
    Aggregate per-cycle across all elements: mean, std, min, max.
    Groups by wall_time_s rounded to the nearest 100 ms cycle boundary.

    smooth_window_s: if > 0, apply a rolling mean over this many seconds
    after aggregation.  Useful for metrics that oscillate at the gossip
    interval (500 ms) — e.g. mean_position_error.
    """
    df = df.copy()
    df['t'] = (df['wall_time_s'] * 10).round() / 10   # round to 0.1 s
    grp = df.groupby('t')[col]
    result = pd.DataFrame({
        'mean': grp.mean(),
        'std':  grp.std().fillna(0),
        'min':  grp.min(),
        'max':  grp.max(),
    })
    if smooth_window_s > 0:
        # Convert seconds to number of 0.1 s buckets
        w = max(1, int(smooth_window_s / 0.1))
        result['mean'] = result['mean'].rolling(w, center=True,
                                                min_periods=1).mean()
        result['std']  = result['std'].rolling(w,  center=True,
                                               min_periods=1).mean()
    return result


# ── Partition region detection ────────────────────────────────────────────────

def partition_regions(df: pd.DataFrame) -> list[tuple[float, float]]:
    """
    Return list of (start_t, end_t) where fresh_ratio < 1.0 for the
    fleet mean.  Used to shade partition windows on every panel.
    """
    stats = per_cycle_stats(df, 'fresh_ratio')
    in_partition = stats['mean'] < 0.99
    regions = []
    start = None
    for t, flag in in_partition.items():
        if flag and start is None:
            start = t
        elif not flag and start is not None:
            regions.append((start, t))
            start = None
    if start is not None:
        regions.append((start, stats.index[-1]))
    return regions


# ── Panel drawing ─────────────────────────────────────────────────────────────

def _draw_panel(ax, stats: pd.DataFrame, colour: str, label: str,
                regions: list, ylabel: str, ylim=None, hline=None):
    t = stats.index.values
    m = stats['mean'].values
    s = stats['std'].values

    ax.plot(t, m, color=colour, linewidth=1.8, label=label)
    ax.fill_between(t, m - s, m + s, color=colour, alpha=0.15)

    for r_start, r_end in regions:
        ax.axvspan(r_start, r_end, color='#FFC107', alpha=0.15)

    if hline is not None:
        ax.axhline(hline, color='grey', linewidth=0.8, linestyle='--')

    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_xlim(left=0)
    if ylim:
        ax.set_ylim(*ylim)
    ax.grid(axis='y', linewidth=0.4, alpha=0.5)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


# ── Main ──────────────────────────────────────────────────────────────────────

def plot(paths: list[str], labels: list[str], out: str | None):
    datasets = [load(p) for p in paths]

    fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    fig.subplots_adjust(hspace=0.08, top=0.93, bottom=0.07)

    # Use the first dataset's partition regions as the reference timeline
    regions = partition_regions(datasets[0])

    panels = [
        ('fresh_ratio',         'Fresh ratio [0–1]',      (0, 1.05), 0.5),
        ('degraded',            'Fraction degraded [0–1]', (0, 1.05), None),
        ('mean_position_error', 'Mean pos error (u)',      None,       None),
        ('mean_age_ms',         'Mean age (ms)',            None,       None),
        ('min_separation',      'Min separation (u)',       (0, None),  3.0),
    ]

    legend_handles = []

    for ds_idx, (df, label) in enumerate(zip(datasets, labels)):
        colour = COLOURS[ds_idx % len(COLOURS)]
        handle = mpatches.Patch(color=colour, label=label)
        legend_handles.append(handle)

        for ax, (col, ylabel, ylim, hline) in zip(axes, panels):
            # mean_position_error oscillates at the gossip interval (500 ms)
            # due to intra-island belief refreshes; smooth over 1.5 s.
            smooth = 1.5 if col == 'mean_position_error' else 0.0
            stats = per_cycle_stats(df, col, smooth_window_s=smooth)
            _draw_panel(ax, stats, colour, label, regions,
                        ylabel, ylim=ylim, hline=hline)

    # Partition shading legend entry
    if regions:
        legend_handles.append(
            mpatches.Patch(color='#FFC107', alpha=0.4, label='Partition window')
        )

    axes[-1].set_xlabel('Wall time (s)', fontsize=9)

    # Metric labels on panels
    for ax, (_, ylabel, _, _) in zip(axes, panels):
        ax.set_ylabel(ylabel, fontsize=9)

    fig.legend(handles=legend_handles, loc='upper right',
               fontsize=9, framealpha=0.9)
    fig.suptitle('Tapestry L4 CSM — Algorithm Efficacy', fontsize=12,
                 fontweight='bold')

    if out:
        fig.savefig(out, dpi=150, bbox_inches='tight')
        print(f"saved: {out}")
    else:
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description='Plot Tapestry CSM telemetry',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('csvs', nargs='+', metavar='CSV',
                   help='One or two telemetry CSV files')
    p.add_argument('--labels', nargs='+', metavar='LABEL',
                   help='Legend labels (one per CSV)')
    p.add_argument('--out', metavar='FILE',
                   help='Save figure to file instead of displaying')
    args = p.parse_args()

    if len(args.csvs) > 2:
        p.error('at most two CSV files supported')

    labels = args.labels or [f'Run {i+1}' for i in range(len(args.csvs))]
    if len(labels) < len(args.csvs):
        labels += [f'Run {i+1}' for i in range(len(labels), len(args.csvs))]

    try:
        plot(args.csvs, labels, args.out)
    except FileNotFoundError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
