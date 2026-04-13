"""
plot.py — Tapestry L5 SCR simulation telemetry visualiser.

Reads one or two combined L4+L5 telemetry CSV files (produced by
tapestry-scr-sim/orchestrator/main.py) and produces a 5-panel figure
focused on election and quorum dynamics:

  Panel 1 — quorum_state          Fleet-mean quorum level (0=LOST … 2=HEALTHY)
  Panel 2 — leader_agreement      Fraction of elements agreeing on same leader
  Panel 3 — is_leader_count       Number of elements with role=LEADER (should be 1)
  Panel 4 — fresh_count           Fleet-mean non-self fresh peer count
  Panel 5 — min_separation        Closest peer distance (safety proxy from L4)

Partition windows are inferred from quorum_state < 2 and shaded amber.

Usage
─────
    # Single run
    python plot.py telemetry.csv

    # Compare two scenarios
    python plot.py leader_loss.csv cascade.csv --labels "Leader loss" Cascade

    # Save without displaying
    python plot.py telemetry.csv --out result.png
"""

import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

COLOURS = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0']

ELEMENT_ID_INVALID = 255

# ── Data loading ──────────────────────────���───────────────────────────────────

def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df['wall_time_s']    = df['wall_time_s'].astype(float)
    df['min_separation'] = pd.to_numeric(df['min_separation'], errors='coerce')
    return df


# ── Derived metrics ───────────────────────────���─────────────────────────────��─

def _time_bucket(df: pd.DataFrame) -> pd.Series:
    """Round wall_time_s to the nearest 100 ms cycle boundary."""
    return (df['wall_time_s'] * 10).round() / 10


def per_cycle_stats(df: pd.DataFrame, col: str,
                    smooth_window_s: float = 0.0) -> pd.DataFrame:
    """Fleet mean ± std per 100 ms time bucket."""
    df = df.copy()
    df['t'] = _time_bucket(df)
    grp = df.groupby('t')[col]
    result = pd.DataFrame({
        'mean': grp.mean(),
        'std':  grp.std().fillna(0),
        'min':  grp.min(),
        'max':  grp.max(),
    })
    if smooth_window_s > 0:
        w = max(1, int(smooth_window_s / 0.1))
        result['mean'] = result['mean'].rolling(w, center=True, min_periods=1).mean()
        result['std']  = result['std'].rolling(w,  center=True, min_periods=1).mean()
    return result


def leader_agreement_series(df: pd.DataFrame) -> pd.DataFrame:
    """
    At each time bucket, compute the fraction of elements that agree on
    the same leader_id.

    Agreement = count(most_common leader_id) / n_elements.
    Elements reporting leader_id=255 (ELEMENT_ID_INVALID) are counted as
    a distinct 'no leader' group — if all elements have no leader that
    counts as full agreement on 'no leader', not as consensus on a leader.
    """
    df = df.copy()
    df['t'] = _time_bucket(df)
    n_elements = df['element_id'].nunique()

    def _agreement(group):
        counts = group['leader_id'].value_counts()
        return counts.iloc[0] / n_elements if len(counts) > 0 else 0.0

    result = df.groupby('t').apply(_agreement, include_groups=False)
    return pd.DataFrame({'mean': result, 'std': np.zeros(len(result))})


def is_leader_count_series(df: pd.DataFrame) -> pd.DataFrame:
    """Number of elements with role=LEADER (2) per time bucket."""
    df = df.copy()
    df['t'] = _time_bucket(df)
    df['is_leader'] = (df['role'] == 2).astype(int)
    grp = df.groupby('t')['is_leader']
    return pd.DataFrame({
        'mean': grp.sum().astype(float),
        'std':  grp.std().fillna(0),
        'min':  grp.min(),
        'max':  grp.max(),
    })


# ── Partition region detection ───────────────────────────���────────────────────

def partition_regions(df: pd.DataFrame) -> list[tuple[float, float]]:
    """
    Shaded windows where fleet-mean quorum_state < 2 (not HEALTHY).
    Uses quorum_state rather than fresh_ratio so it captures DEGRADED too.
    """
    stats = per_cycle_stats(df, 'quorum_state')
    not_healthy = stats['mean'] < 1.95   # below HEALTHY (2.0) with small margin
    regions = []
    start = None
    for t, flag in not_healthy.items():
        if flag and start is None:
            start = t
        elif not flag and start is not None:
            regions.append((start, t))
            start = None
    if start is not None:
        regions.append((start, stats.index[-1]))
    return regions


# ── Panel drawing ─────────────────────��────────────────────────────────────���──

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


# ── Main ──────────────────────────────���─────────────────────────────��─────────

def plot(paths: list[str], labels: list[str], out: str | None):
    datasets = [load(p) for p in paths]

    fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    fig.subplots_adjust(hspace=0.08, top=0.93, bottom=0.07)

    regions = partition_regions(datasets[0])
    legend_handles = []

    for ds_idx, (df, label) in enumerate(zip(datasets, labels)):
        colour = COLOURS[ds_idx % len(COLOURS)]
        legend_handles.append(mpatches.Patch(color=colour, label=label))

        # Panel 1 — quorum_state (0=LOST, 1=DEGRADED, 2=HEALTHY)
        _draw_panel(axes[0],
                    per_cycle_stats(df, 'quorum_state'),
                    colour, label, regions,
                    ylabel='Quorum state [0=LOST … 2=HEALTHY]',
                    ylim=(-0.1, 2.3),
                    hline=2.0)

        # Panel 2 — leader agreement
        _draw_panel(axes[1],
                    leader_agreement_series(df),
                    colour, label, regions,
                    ylabel='Leader agreement [0–1]',
                    ylim=(0, 1.05),
                    hline=1.0)

        # Panel 3 — is_leader count (should be exactly 1 in healthy state)
        _draw_panel(axes[2],
                    is_leader_count_series(df),
                    colour, label, regions,
                    ylabel='# elements as LEADER',
                    ylim=(-0.1, None),
                    hline=1.0)

        # Panel 4 — fresh_count
        _draw_panel(axes[3],
                    per_cycle_stats(df, 'fresh_count'),
                    colour, label, regions,
                    ylabel='Fresh peer count',
                    ylim=(0, None))

        # Panel 5 — min_separation (safety proxy from L4)
        _draw_panel(axes[4],
                    per_cycle_stats(df, 'min_separation'),
                    colour, label, regions,
                    ylabel='Min separation (u)',
                    ylim=(0, None),
                    hline=3.0)

    if regions:
        legend_handles.append(
            mpatches.Patch(color='#FFC107', alpha=0.4, label='Quorum degraded')
        )

    # Quorum state tick labels
    axes[0].set_yticks([0, 1, 2])
    axes[0].set_yticklabels(['LOST', 'DEGRADED', 'HEALTHY'], fontsize=7)

    axes[-1].set_xlabel('Wall time (s)', fontsize=9)
    fig.legend(handles=legend_handles, loc='upper right',
               fontsize=9, framealpha=0.9)
    fig.suptitle('Tapestry L5 SCR — Election & Quorum Dynamics',
                 fontsize=12, fontweight='bold')

    if out:
        fig.savefig(out, dpi=150, bbox_inches='tight')
        print(f"saved: {out}")
    else:
        plt.show()


# ── CLI ──────────────────────────────────────────────────────────��────────────

def main():
    p = argparse.ArgumentParser(
        description='Plot Tapestry SCR telemetry',
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
