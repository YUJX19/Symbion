/**
 * @module src/isac/metrics
 * @description ISAC Performance Metrics - Physical quantity calculations
 * 
 * This module provides calculation functions for ISAC-specific metrics
 * (LoS probability, throughput, URLLC evaluation). These are distinct from
 * core.MetricSpec which defines optimization objectives.
 * 
 * Use cases:
 * - Calculate LoS probability for a UAV-user link
 * - Compute aggregate throughput for users
 * - Evaluate URLLC constraint satisfaction
 */

export * from './los';
export * from './throughput';
export * from './urllc';
