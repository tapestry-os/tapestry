"""
plot.py — visualise Phase 2 hardware telemetry

Reads one or more CSVs produced by collect.py and generates a 5-panel
figure showing L4 + L5 behaviour across both physical elements over time.

The key proof point — moving the ESP32 out of WiFi range — appears as:
  • fresh_ratio drops to 0 on the RA8D1 (peer no longer heard)
  • quorum_state drops to LOST
  • a new leader election fires on the surviving element
  • on return: gossip resumes, quorum recovers, election counter increments

Usage:
    python plot.py hw_run.csv                 # opens interactive window
    python plot.py hw_run.csv --out hw_run.png
    python plot.py run1.csv run2.csv --labels "bias=0" "bias=1" --out compare.png
"""

import argparse
import sys

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

QUORUM_NAMES = {0: 'LOST', 1: 'DEGRADED', 2: 'HEALTHY'}
ROLE_NAMES   = {0: 'NONE', 1: 'FOLLOWER', 2: 'LEADER'}
QUORUM_COLS  = ['#d62728', '#ff7f0e', '#2ca02c']   # red / orange / green
ROLE_COLS    = ['#9467bd', '#1f77b4', '#e377c2']   # purple / blue / pink

ELEMENT_NAMES = {0: 'ESP32 (elem 0)', 1: 'RA8D1 (elem 1)'}


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    return df


def element_label(eid: int) -> str:
    return ELEMENT_NAMES.get(eid, f'elem {eid}')


def plot_run(axes, df: pd.DataFrame, label_prefix: str = ''):
    """Fill five axes for a single CSV run."""
    ax_fresh, ax_quorum, ax_role, ax_leader, ax_age = axes
    prefix = f'{label_prefix} ' if label_prefix else ''

    for eid, grp in df.groupby('element_id'):
        t   = grp['wall_time_s'].values
        lbl = f'{prefix}{element_label(eid)}'

        # Panel 1 — fresh ratio
        ax_fresh.plot(t, grp['fresh_ratio'], label=lbl)

        # Panel 2 — quorum state (0/1/2)
        ax_quorum.step(t, grp['quorum_state'], where='post', label=lbl)

        # Panel 3 — role (0=NONE 1=FOLLOWER 2=LEADER)
        ax_role.step(t, grp['role'], where='post', label=lbl)

        # Panel 4 — elected leader ID (0xFF = no leader → NaN for gap)
        leader = grp['leader_id'].where(grp['leader_id'] != 255)
        ax_leader.step(t, leader, where='post', label=lbl)

        # Panel 5 — mean peer age (ms)
        ax_age.plot(t, grp['mean_age_ms'], label=lbl)


def annotate_thresholds(ax_age):
    """Mark L4 staleness thresholds on the age panel."""
    # WM_STALE_THRESHOLD_MS = 750, WM_INACTIVE_THRESHOLD_MS = 1500
    ax_age.axhline(750,  color='orange', linestyle='--', linewidth=0.8,
                   label='stale (750 ms)')
    ax_age.axhline(1500, color='red',    linestyle='--', linewidth=0.8,
                   label='inactive (1500 ms)')


def make_figure(csvs: list[str], labels: list[str], out: str | None):
    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle('Tapestry Phase 2 — hardware gossip (ESP32 + RA8D1)',
                 fontsize=13, fontweight='bold')

    ax_fresh, ax_quorum, ax_role, ax_leader, ax_age = axes

    for path, lbl in zip(csvs, labels):
        df = load(path)
        plot_run(axes, df, label_prefix=lbl if len(csvs) > 1 else '')

    # ── Axis formatting ───────────────────────────────────────────────────────

    ax_fresh.set_ylabel('Fresh ratio')
    ax_fresh.set_ylim(-0.05, 1.05)
    ax_fresh.axhline(1.0, color='grey', linestyle=':', linewidth=0.6)

    ax_quorum.set_ylabel('Quorum state')
    ax_quorum.set_yticks([0, 1, 2])
    ax_quorum.set_yticklabels(['LOST', 'DEGRADED', 'HEALTHY'])

    ax_role.set_ylabel('Role')
    ax_role.set_yticks([0, 1, 2])
    ax_role.set_yticklabels(['NONE', 'FOLLOWER', 'LEADER'])

    ax_leader.set_ylabel('Leader ID')
    ax_leader.set_yticks([0, 1])
    ax_leader.set_yticklabels([element_label(0), element_label(1)])

    ax_age.set_ylabel('Mean peer age (ms)')
    annotate_thresholds(ax_age)

    axes[-1].set_xlabel('Wall time (s)')

    for ax in axes:
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if out:
        plt.savefig(out, dpi=150)
        print(f'saved: {out}')
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Plot Tapestry Phase 2 hardware telemetry')
    parser.add_argument('csvs', nargs='+',
                        help='CSV file(s) from collect.py')
    parser.add_argument('--labels', nargs='*',
                        help='per-file labels (default: filenames)')
    parser.add_argument('--out', default=None,
                        help='output PNG path (omit to show interactive window)')
    args = parser.parse_args()

    labels = args.labels or [p.replace('.csv', '') for p in args.csvs]
    if len(labels) < len(args.csvs):
        labels += args.csvs[len(labels):]

    make_figure(args.csvs, labels, args.out)


if __name__ == '__main__':
    main()
