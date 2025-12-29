#!/usr/bin/env python3
"""
ISAC Trajectory Benchmark Plotting Script

Generates publication-quality figures for ISAC trajectory optimization results.

Usage:
    python scripts/plot_isac.py --input benchmarks/isac-trajectory/results/ --output figures/
"""

import argparse
import json
import os
import glob
from pathlib import Path
from typing import Dict, List, Any, Tuple

import numpy as np

# Try to import matplotlib
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.lines import Line2D
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    print("Error: matplotlib is required. Install with: pip install matplotlib")
    exit(1)

# Try to import pandas for CSV reading
try:
    import pandas as pd
except ImportError:
    pd = None
    print("Warning: pandas not found. CSV trajectory loading will use basic parser.")

# ==================== Style Configuration ====================

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.figsize': (3.5, 2.8),
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.5,
    'lines.markersize': 5,
})

COLORS = {
    'hover': '#1f77b4',
    'random': '#ff7f0e',
    'rate-only': '#2ca02c',
    'energy-efficient': '#d62728',
    'proposed': '#9467bd',
}

LINE_STYLES = {
    'hover': '--',
    'random': ':',
    'rate-only': '-.',
    'energy-efficient': '-',
    'proposed': '-',
}

# ==================== Data Loading ====================

def load_report(filepath: str) -> Dict:
    """Load a single report JSON file."""
    with open(filepath, 'r') as f:
        return json.load(f)


def load_trajectory_csv(filepath: str) -> Dict[str, np.ndarray]:
    """Load trajectory CSV file."""
    if pd is not None:
        df = pd.read_csv(filepath)
        return {col: df[col].values for col in df.columns}
    else:
        # Basic CSV parsing
        with open(filepath, 'r') as f:
            lines = f.readlines()
        headers = lines[0].strip().split(',')
        data = {h: [] for h in headers}
        for line in lines[1:]:
            values = line.strip().split(',')
            for h, v in zip(headers, values):
                data[h].append(float(v))
        return {h: np.array(v) for h, v in data.items()}


def find_reports_in_dir(directory: str) -> List[str]:
    """Find all report JSON files in directory."""
    return glob.glob(os.path.join(directory, '*_report.json'))


# ==================== Plotting Functions ====================

def plot_3d_trajectory(trajectories: Dict[str, Dict], output_dir: str):
    """Plot 3D trajectory comparison."""
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    for policy, traj in trajectories.items():
        color = COLORS.get(policy, '#333333')
        ax.plot(traj['x'], traj['y'], traj['z'],
                color=color, label=policy, linewidth=2)
        
        # Mark start and end
        ax.scatter([traj['x'][0]], [traj['y'][0]], [traj['z'][0]],
                   color=color, marker='o', s=50)
        ax.scatter([traj['x'][-1]], [traj['y'][-1]], [traj['z'][-1]],
                   color=color, marker='X', s=50)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('UAV Trajectory Comparison')
    ax.legend(loc='upper left')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'isac_3d_trajectory.pdf'))
    plt.savefig(os.path.join(output_dir, 'isac_3d_trajectory.png'))
    plt.close()
    print(f"  ✓ Saved 3d_trajectory plot")


def plot_2d_trajectory_with_users(trajectories: Dict[str, Dict], 
                                   users: List[Dict], 
                                   obstacles: List[Dict],
                                   output_dir: str):
    """Plot 2D top-down view with users and obstacles."""
    fig, ax = plt.subplots(figsize=(6, 6))
    
    # Plot obstacles
    for obs in obstacles:
        x, y = obs['position']['x'], obs['position']['y']
        w, d = obs['dimensions'][0], obs['dimensions'][1]
        rect = plt.Rectangle((x - w/2, y - d/2), w, d,
                              color='gray', alpha=0.5, label='Building')
        ax.add_patch(rect)
    
    # Plot users
    for user in users:
        x, y = user['position']['x'], user['position']['y']
        color = 'red' if user.get('isUrllc', False) else 'blue'
        marker = '^' if user.get('isUrllc', False) else 'o'
        ax.scatter(x, y, c=color, marker=marker, s=100, zorder=5)
        ax.annotate(user['id'], (x, y), textcoords="offset points",
                    xytext=(5, 5), fontsize=8)
    
    # Plot trajectories
    for policy, traj in trajectories.items():
        color = COLORS.get(policy, '#333333')
        style = LINE_STYLES.get(policy, '-')
        ax.plot(traj['x'], traj['y'], color=color, linestyle=style,
                label=policy, linewidth=2)
        ax.scatter([traj['x'][0]], [traj['y'][0]], color=color, marker='o', s=80)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('UAV Trajectory (Top View)')
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # Custom legend
    legend_elements = [
        mpatches.Patch(facecolor='gray', alpha=0.5, label='Building'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='red',
               markersize=10, label='URLLC User'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
               markersize=10, label='eMBB User'),
    ]
    ax.legend(handles=legend_elements, loc='lower left')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'isac_2d_trajectory.pdf'))
    plt.savefig(os.path.join(output_dir, 'isac_2d_trajectory.png'))
    plt.close()
    print(f"  ✓ Saved 2d_trajectory plot")


def plot_metrics_over_time(trajectories: Dict[str, Dict], output_dir: str):
    """Plot metrics vs time step."""
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    
    for policy, traj in trajectories.items():
        color = COLORS.get(policy, '#333333')
        steps = traj['step']
        
        # LoS percentage
        axes[0, 0].plot(steps, traj['losPercentage'] * 100, color=color, label=policy)
        axes[0, 0].set_ylabel('LoS Percentage (%)')
        axes[0, 0].set_title('Line-of-Sight over Time')
        
        # Throughput
        axes[0, 1].plot(steps, traj['totalThroughput'] / 1e6, color=color, label=policy)
        axes[0, 1].set_ylabel('Throughput (Mbps)')
        axes[0, 1].set_title('Total Throughput over Time')
        
        # Speed
        axes[1, 0].plot(steps, traj['speed'], color=color, label=policy)
        axes[1, 0].set_ylabel('Speed (m/s)')
        axes[1, 0].set_xlabel('Step')
        axes[1, 0].set_title('UAV Speed over Time')
        
        # Energy
        axes[1, 1].plot(steps, traj['energyUsed'], color=color, label=policy)
        axes[1, 1].set_ylabel('Cumulative Energy (J)')
        axes[1, 1].set_xlabel('Step')
        axes[1, 1].set_title('Energy Consumption')
    
    axes[0, 0].legend(loc='best', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'isac_metrics_time.pdf'))
    plt.savefig(os.path.join(output_dir, 'isac_metrics_time.png'))
    plt.close()
    print(f"  ✓ Saved metrics_over_time plot")


def plot_policy_comparison_radar(reports: List[Dict], output_dir: str):
    """Plot radar chart comparing policies."""
    metrics = ['Throughput', 'LoS', 'Energy Eff.', 'URLLC', 'Success']
    
    fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(projection='polar'))
    
    angles = np.linspace(0, 2 * np.pi, len(metrics), endpoint=False).tolist()
    angles += angles[:1]  # Complete the loop
    
    for report in reports:
        policy = report['policyName']
        a = report['aggregate']
        
        # Normalize values to 0-1 scale
        values = [
            a['avgThroughput']['mean'] / 50e6,  # Normalize to 50 Mbps
            a['avgLosPersistence']['mean'],
            1 - a['totalEnergy']['mean'] / 10000,  # Invert energy
            1 - a['urllcViolationRate']['mean'],
            a['successRate'],
        ]
        values += values[:1]  # Complete the loop
        
        color = COLORS.get(policy, '#333333')
        ax.plot(angles, values, color=color, label=policy, linewidth=2)
        ax.fill(angles, values, color=color, alpha=0.1)
    
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metrics)
    ax.set_ylim(0, 1)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
    ax.set_title('Policy Comparison')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'isac_radar_comparison.pdf'))
    plt.savefig(os.path.join(output_dir, 'isac_radar_comparison.png'))
    plt.close()
    print(f"  ✓ Saved radar_comparison plot")


def plot_aggregate_bar(reports: List[Dict], output_dir: str):
    """Plot aggregate metrics bar chart."""
    fig, axes = plt.subplots(1, 4, figsize=(12, 3))
    
    policies = [r['policyName'] for r in reports]
    x = np.arange(len(policies))
    width = 0.6
    colors = [COLORS.get(p, '#333333') for p in policies]
    
    # Throughput
    values = [r['aggregate']['avgThroughput']['mean'] / 1e6 for r in reports]
    stds = [r['aggregate']['avgThroughput']['std'] / 1e6 for r in reports]
    axes[0].bar(x, values, width, yerr=stds, capsize=3, color=colors)
    axes[0].set_ylabel('Throughput (Mbps)')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    
    # LoS
    values = [r['aggregate']['avgLosPersistence']['mean'] * 100 for r in reports]
    stds = [r['aggregate']['avgLosPersistence']['std'] * 100 for r in reports]
    axes[1].bar(x, values, width, yerr=stds, capsize=3, color=colors)
    axes[1].set_ylabel('LoS (%)')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    axes[1].axhline(y=70, color='red', linestyle='--', alpha=0.5)
    
    # URLLC Violation
    values = [r['aggregate']['urllcViolationRate']['mean'] * 100 for r in reports]
    stds = [r['aggregate']['urllcViolationRate']['std'] * 100 for r in reports]
    axes[2].bar(x, values, width, yerr=stds, capsize=3, color=colors)
    axes[2].set_ylabel('URLLC Violation (%)')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    axes[2].axhline(y=5, color='red', linestyle='--', alpha=0.5)
    
    # Success Rate
    values = [r['aggregate']['successRate'] * 100 for r in reports]
    axes[3].bar(x, values, width, color=colors)
    axes[3].set_ylabel('Success Rate (%)')
    axes[3].set_xticks(x)
    axes[3].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'isac_aggregate_comparison.pdf'))
    plt.savefig(os.path.join(output_dir, 'isac_aggregate_comparison.png'))
    plt.close()
    print(f"  ✓ Saved aggregate_comparison plot")


# ==================== Main ====================

def main():
    parser = argparse.ArgumentParser(description='Generate ISAC trajectory plots')
    parser.add_argument('--input', '-i', required=True, help='Directory containing results')
    parser.add_argument('--output', '-o', default='figures', help='Output directory')
    args = parser.parse_args()
    
    os.makedirs(args.output, exist_ok=True)
    
    print(f"\n📊 ISAC Trajectory Plotting Script")
    print(f"   Input: {args.input}")
    print(f"   Output: {args.output}")
    
    # Find and load reports
    report_files = find_reports_in_dir(args.input)
    print(f"   Found {len(report_files)} report files\n")
    
    reports = [load_report(f) for f in report_files]
    
    if not reports:
        print("No reports found. Run benchmarks first.")
        return
    
    # Load trajectories
    trajectories = {}
    for report_file in report_files:
        policy = Path(report_file).stem.split('_')[0]
        traj_file = report_file.replace('_report.json', '_trajectory.csv')
        if os.path.exists(traj_file):
            trajectories[policy] = load_trajectory_csv(traj_file)
    
    print("Generating plots...")
    
    # Generate plots
    if trajectories:
        plot_3d_trajectory(trajectories, args.output)
        plot_metrics_over_time(trajectories, args.output)
        
        # Get users and obstacles from first report config
        if reports:
            config = reports[0].get('config', {})
            users = config.get('users', [])
            obstacles = config.get('obstacles', [])
            if users:
                plot_2d_trajectory_with_users(trajectories, users, obstacles, args.output)
    
    if len(reports) > 1:
        plot_policy_comparison_radar(reports, args.output)
        plot_aggregate_bar(reports, args.output)
    
    print(f"\n✅ All plots saved to {args.output}/")


if __name__ == '__main__':
    main()
