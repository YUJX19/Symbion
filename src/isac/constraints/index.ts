/**
 * @module src/isac/constraints
 * @description ISAC Constraints
 * 
 * Contains two types of constraint utilities:
 * 
 * 1. Simple check functions (flight.ts, communication.ts):
 *    - Boolean validation functions
 *    - Used for quick constraint checking
 * 
 * 2. ConstraintSpec factories (flightSpec.ts, communicationSpec.ts):
 *    - Produce ConstraintSpec objects compatible with core.Runner
 *    - Used for experiment evaluation and logging
 */

// Simple check functions
export * from './flight';
export * from './communication';

// ConstraintSpec factories (for integration with core.Runner)
export * from './flightSpec';
export * from './communicationSpec';
