# U2U-MCS Benchmark

This directory contains benchmark configurations for the U2U Sidelink Dynamic MCS Selection task.

> **Full Documentation**: See [Tasks User Guide](../../docs/tasks-user-guide.md) for comprehensive API reference, tutorials, and examples.

## Configurations

| Config | Description | Speed Range | Distance Range |
|--------|-------------|-------------|----------------|
| `baseline-high-speed.json` | Urban highway scenario | 20-40 m/s | 50-300 m |
| `baseline-low-speed.json` | Pedestrian scenario | 0-5 m/s | 10-100 m |

## Running Benchmarks

```bash
# Run with a specific baseline policy
npx ts-node benchmarks/u2u-mcs/run-baseline.ts --config=baseline-high-speed.json --policy=greedy

# Run all baselines
npx ts-node benchmarks/u2u-mcs/run-baseline.ts --config=baseline-high-speed.json --all
```

## Output

Results are saved to `benchmarks/u2u-mcs/results/` in JSONL and CSV formats.

### Step-level Log Schema (JSONL)

```json
{
  "task": "u2u-mcs",
  "seed": 42,
  "episode": 0,
  "step": 100,
  "sinrDb": 15.3,
  "mcs": 12,
  "ack": true,
  "throughput": 30000000,
  "bler": 0.001,
  "reward": 0.85
}
```

### Episode Summary Schema

```json
{
  "avgThroughput": 25000000,
  "p5Throughput": 10000000,
  "avgBler": 0.005,
  "urllcSuccessRate": 0.95,
  "mcsHistogram": [0, 0, 0, 5, 10, 20, ...],
  "mcsSwitchRate": 0.15
}
```

## Baseline Policies

1. **Fixed-MCS**: Always uses the same MCS (0, 5, 10, 15, 20)
2. **Random**: Randomly selects MCS each step
3. **Greedy**: Selects highest MCS that achieves target BLER
4. **BLER-Adaptive**: Adjusts MCS based on recent BLER
5. **OLLA**: Outer Loop Link Adaptation with SINR offset adjustment
