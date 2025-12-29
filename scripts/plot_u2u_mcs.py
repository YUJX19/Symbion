#!/usr/bin/env python3
"""
U2U-MCS Benchmark Plotting Script

Generates publication-quality figures for U2U-MCS task results.

Usage:
    python scripts/plot_u2u_mcs.py --input benchmarks/u2u-mcs/results/comparison_*.json --output figures/
"""

import argparse
import json
import os
from pathlib import Path
from typing import Dict, List, Any

import numpy as np

# Try to import matplotlib, provide helpful message if not available
try:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    from matplotlib.patches import Patch
except ImportError:
    print("Error: matplotlib is required. Install with: pip install matplotlib")
    exit(1)

# ==================== Style Configuration ====================

# IEEE-style settings
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.figsize': (3.5, 2.8),  # Single column width
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.5,
    'lines.markersize': 5,
})

# Color palette (colorblind-friendly)
COLORS = {
    'fixed-0': '#1f77b4',
    'fixed-5': '#ff7f0e',
    'fixed-10': '#2ca02c',
    'fixed-15': '#d62728',
    'fixed-20': '#9467bd',
    'random': '#8c564b',
    'greedy': '#e377c2',
    'bler-adaptive': '#7f7f7f',
    'olla': '#bcbd22',
    'proposed': '#17becf',
}

MARKERS = {
    'fixed-0': 'o',
    'fixed-5': 's',
    'fixed-10': '^',
    'fixed-15': 'D',
    'fixed-20': 'v',
    'random': 'X',
    'greedy': 'P',
    'bler-adaptive': '*',
    'olla': 'h',
    'proposed': 'p',
}

# ==================== Data Loading ====================

def load_comparison_data(filepath: str) -> Dict[str, List[Dict]]:
    """Load comparison JSON file."""
    with open(filepath, 'r') as f:
        return json.load(f)


def aggregate_metrics(episodes: List[Dict]) -> Dict[str, Dict[str, float]]:
    """Calculate mean and std for each metric."""
    metrics = ['avgThroughput', 'avgBler', 'urllcSuccessRate', 'mcsSwitchRate', 'totalReward']
    result = {}
    
    for metric in metrics:
        values = [ep.get(metric, 0) for ep in episodes]
        result[metric] = {
            'mean': np.mean(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
        }
    
    return result


# ==================== Plotting Functions ====================

def plot_throughput_vs_bler(data: Dict[str, List[Dict]], output_dir: str):
    """Plot throughput vs BLER trade-off scatter plot."""
    fig, ax = plt.subplots()
    
    for policy, episodes in data.items():
        agg = aggregate_metrics(episodes)
        
        throughput = agg['avgThroughput']['mean'] / 1e6  # Convert to Mbps
        bler = agg['avgBler']['mean']
        throughput_std = agg['avgThroughput']['std'] / 1e6
        bler_std = agg['avgBler']['std']
        
        color = COLORS.get(policy, '#333333')
        marker = MARKERS.get(policy, 'o')
        
        ax.errorbar(bler, throughput,
                    xerr=bler_std, yerr=throughput_std,
                    fmt=marker, color=color, label=policy,
                    capsize=2, markersize=8)
    
    ax.set_xlabel('Average BLER')
    ax.set_ylabel('Average Throughput (Mbps)')
    ax.set_xscale('log')
    ax.set_title('Throughput vs BLER Trade-off')
    ax.legend(loc='best', ncol=2, fontsize=8)
    
    # Add URLLC target line
    ax.axvline(x=1e-5, color='red', linestyle='--', alpha=0.5, label='URLLC Target')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'u2u_throughput_vs_bler.pdf'))
    plt.savefig(os.path.join(output_dir, 'u2u_throughput_vs_bler.png'))
    plt.close()
    print(f"  ✓ Saved throughput_vs_bler plot")


def plot_policy_comparison_bar(data: Dict[str, List[Dict]], output_dir: str):
    """Plot bar chart comparing all policies."""
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3))
    
    policies = list(data.keys())
    x = np.arange(len(policies))
    width = 0.6
    
    # Metric 1: Throughput
    throughputs = []
    throughput_stds = []
    for policy in policies:
        agg = aggregate_metrics(data[policy])
        throughputs.append(agg['avgThroughput']['mean'] / 1e6)
        throughput_stds.append(agg['avgThroughput']['std'] / 1e6)
    
    colors = [COLORS.get(p, '#333333') for p in policies]
    axes[0].bar(x, throughputs, width, yerr=throughput_stds, capsize=3, color=colors)
    axes[0].set_ylabel('Throughput (Mbps)')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    axes[0].set_title('Average Throughput')
    
    # Metric 2: BLER
    blers = []
    bler_stds = []
    for policy in policies:
        agg = aggregate_metrics(data[policy])
        blers.append(agg['avgBler']['mean'])
        bler_stds.append(agg['avgBler']['std'])
    
    axes[1].bar(x, blers, width, yerr=bler_stds, capsize=3, color=colors)
    axes[1].set_ylabel('BLER')
    axes[1].set_yscale('log')
    axes[1].axhline(y=1e-5, color='red', linestyle='--', alpha=0.5, label='Target')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    axes[1].set_title('Average BLER')
    axes[1].legend(fontsize=7)
    
    # Metric 3: URLLC Success Rate
    urllc_rates = []
    urllc_stds = []
    for policy in policies:
        agg = aggregate_metrics(data[policy])
        urllc_rates.append(agg['urllcSuccessRate']['mean'] * 100)
        urllc_stds.append(agg['urllcSuccessRate']['std'] * 100)
    
    axes[2].bar(x, urllc_rates, width, yerr=urllc_stds, capsize=3, color=colors)
    axes[2].set_ylabel('URLLC Success Rate (%)')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(policies, rotation=45, ha='right', fontsize=8)
    axes[2].set_title('URLLC Reliability')
    axes[2].axhline(y=99.999, color='red', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'u2u_policy_comparison.pdf'))
    plt.savefig(os.path.join(output_dir, 'u2u_policy_comparison.png'))
    plt.close()
    print(f"  ✓ Saved policy_comparison plot")


def plot_reward_boxplot(data: Dict[str, List[Dict]], output_dir: str):
    """Plot box plot of rewards across episodes."""
    fig, ax = plt.subplots(figsize=(6, 4))
    
    policies = list(data.keys())
    rewards_list = []
    
    for policy in policies:
        rewards = [ep['totalReward'] for ep in data[policy]]
        rewards_list.append(rewards)
    
    bp = ax.boxplot(rewards_list, labels=policies, patch_artist=True)
    
    # Color boxes
    for patch, policy in zip(bp['boxes'], policies):
        patch.set_facecolor(COLORS.get(policy, '#cccccc'))
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Total Episode Reward')
    ax.set_title('Reward Distribution by Policy')
    plt.xticks(rotation=45, ha='right')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'u2u_reward_boxplot.pdf'))
    plt.savefig(os.path.join(output_dir, 'u2u_reward_boxplot.png'))
    plt.close()
    print(f"  ✓ Saved reward_boxplot plot")


# ==================== Main ====================

def main():
    parser = argparse.ArgumentParser(description='Generate U2U-MCS benchmark plots')
    parser.add_argument('--input', '-i', required=True, help='Path to comparison JSON file')
    parser.add_argument('--output', '-o', default='figures', help='Output directory for figures')
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    print(f"\n📊 U2U-MCS Plotting Script")
    print(f"   Input: {args.input}")
    print(f"   Output: {args.output}")
    
    # Load data
    data = load_comparison_data(args.input)
    print(f"   Loaded {len(data)} policies\n")
    
    # Generate plots
    print("Generating plots...")
    plot_throughput_vs_bler(data, args.output)
    plot_policy_comparison_bar(data, args.output)
    plot_reward_boxplot(data, args.output)
    
    print(f"\n✅ All plots saved to {args.output}/")


if __name__ == '__main__':
    main()
