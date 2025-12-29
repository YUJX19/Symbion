# ISAC Trajectory Benchmark

This directory contains benchmark configurations for the ISAC LoS-aware URLLC Trajectory Optimization task.

> **Full Documentation**: See [Tasks User Guide](../../docs/tasks-user-guide.md) for comprehensive API reference, tutorials, and examples.

## Configurations

| Config | Description | Users | Obstacles |
|--------|-------------|-------|-----------|
| `baseline-suburban.json` | Suburban scenario | 3 (2 URLLC, 1 eMBB) | 2 buildings |

## Running Benchmarks

```bash
# Run with a specific planner
npx ts-node benchmarks/isac-trajectory/run-baseline.ts --config=baseline-suburban.json --policy=proposed

# Run all baseline planners
npx ts-node benchmarks/isac-trajectory/run-baseline.ts --config=baseline-suburban.json --all
```

## Output

Results are saved to `benchmarks/isac-trajectory/results/` in JSONL, CSV, and trajectory formats.

### Episode Metrics Schema

```json
{
  "avgThroughput": 25000000,
  "avgLosPersistence": 0.85,
  "totalEnergy": 5000,
  "urllcViolationRate": 0.02,
  "avgSensingCoverage": 0.45,
  "pathLength": 350,
  "missionSuccess": true
}
```

### Trajectory Format (for plotting)

```csv
step,x,y,z,speed,losPercentage,totalThroughput,sensingCoverage,energyUsed
0,0.00,0.00,100.00,0.00,0.6667,45000000.00,0.3500,0.00
1,10.50,5.20,105.00,12.30,0.8000,52000000.00,0.4200,125.50
...
```

## Baseline Planners

1. **Hover**: Stay at initial position
2. **Random**: Random movement direction
3. **Rate-Only**: Move toward user with lowest SINR
4. **Energy-Efficient**: Minimize energy while maintaining coverage
5. **Proposed**: Balance LoS, throughput, energy, and URLLC

## Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| Throughput | Average achievable rate (bps) | Max |
| LoS Persistence | Fraction of users with LoS | > 70% |
| Energy | Total energy consumed (J) | Min |
| URLLC Violation | Rate of URLLC constraint violation | < 5% |
| Sensing Coverage | Fraction of area covered | > 50% |
| Success Rate | Missions meeting all constraints | Max |
