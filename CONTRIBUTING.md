# Contributing to Symbion

Thank you for your interest in contributing to Symbion! This document provides guidelines and instructions for contributing.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Contribution Workflow](#contribution-workflow)
- [Code Standards](#code-standards)
- [Testing Requirements](#testing-requirements)
- [Documentation](#documentation)
- [Pull Request Guidelines](#pull-request-guidelines)
- [Project Structure](#project-structure)

---

## Code of Conduct

Please be respectful and considerate in all interactions. We aim to maintain a welcoming and inclusive community.

---

## Getting Started

### Prerequisites

- Node.js 18+ 
- npm or pnpm
- TypeScript 5+
- Git

### Quick Setup

```bash
# Clone the repository
git clone https://github.com/YUJX19/Symbion.git
cd Symbion

# Install dependencies
npm install

# Run all tests to verify setup
npm run test:all
```

---

## Development Setup

### 1. Fork and Clone

```bash
# Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/Symbion.git
cd Symbion
git remote add upstream https://github.com/YUJX19/Symbion.git
```

### 2. Install Dependencies

```bash
npm install
```

### 3. Verify Setup

```bash
# Run full test suite
npm run test:all

# Or quick test (skips benchmarks)
npm run test:quick
```

### 4. Create a Branch

```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

---

## Contribution Workflow

### 1. Make Your Changes

- Write code following our [Code Standards](#code-standards)
- Add tests for new functionality
- Update documentation as needed

### 2. Run Tests

All tests must pass before submitting a PR:

```bash
# Option 1: Full test suite (recommended)
npm run test:all

# Option 2: Individual steps
npm run typecheck    # TypeScript type checking
npm run lint         # ESLint code linting
npm run test         # Unit tests (Vitest)
npm run build        # Build verification
```

### 3. Commit Your Changes

Use clear, descriptive commit messages:

```bash
git commit -m "feat: add new MCS selection algorithm"
git commit -m "fix: resolve SINR calculation issue"
git commit -m "docs: update API reference for channel module"
```

#### Commit Message Format

- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation only
- `test:` - Adding or updating tests
- `refactor:` - Code refactoring (no functionality change)
- `style:` - Code style/formatting changes
- `chore:` - Build process or tooling changes

### 4. Push and Create PR

```bash
git push origin feature/your-feature-name
```

Then create a Pull Request on GitHub.

---

## Code Standards

### TypeScript

- Use TypeScript for all source files
- Enable strict mode (`strict: true` in tsconfig)
- Avoid `any` type when possible
- Use interfaces for object types

### File Structure

```typescript
/**
 * @module path/to/module
 * @description Short description of the module
 * 
 * Detailed explanation if needed.
 * 
 * @example
 * ```typescript
 * import { someFunction } from 'symbion/path';
 * const result = someFunction(params);
 * ```
 */

// Imports
import { ... } from '../utils';

// Constants
export const CONSTANT_NAME = value;

// Types/Interfaces
export interface Params { ... }
export interface Result { ... }

// Main functions
export function mainFunction(params: Params): Result { ... }

// Helper functions (private or exported)
export function helperFunction(...): ... { ... }
```

### Naming Conventions

| Type | Convention | Example |
|------|------------|---------|
| Files | `kebab-case.ts` | `two-ray-ground.ts` |
| Functions | `camelCase` | `calculatePathLoss` |
| Classes | `PascalCase` | `MincoTrajectory` |
| Interfaces | `PascalCase` | `ChannelParams` |
| Constants | `SCREAMING_SNAKE_CASE` | `SPEED_OF_LIGHT` |
| Type aliases | `PascalCase` | `Vector3Array` |

### Documentation (JSDoc)

All public functions must have JSDoc comments:

```typescript
/**
 * Calculates Shannon channel capacity.
 * 
 * @param snrDb - Signal-to-noise ratio in dB
 * @param bandwidth - Channel bandwidth in Hz
 * @returns Channel capacity in bits per second
 * 
 * @example
 * ```typescript
 * const capacity = shannonCapacity(10, 20e6);
 * // Returns ~69.23 Mbps
 * ```
 */
export function shannonCapacity(snrDb: number, bandwidth: number): number {
    // ...
}
```

---

## Testing Requirements

### Test Structure

Tests are located in `__tests__/` directory:

```typescript
// __tests__/channel.test.ts
import { describe, it, expect } from 'vitest';
import { shannonCapacity } from '../src/models/phy/channel/awgn';

describe('AWGN Channel', () => {
    describe('shannonCapacity', () => {
        it('should calculate correct capacity for 10dB SNR, 20MHz', () => {
            const capacity = shannonCapacity(10, 20e6);
            expect(capacity).toBeCloseTo(69.23e6, -4);
        });

        it('should handle edge cases', () => {
            expect(shannonCapacity(-Infinity, 20e6)).toBe(0);
        });
    });
});
```

### Test Requirements

- All new features must have tests
- Bug fixes should include a test for the fixed issue
- Maintain or improve code coverage
- Tests should be deterministic (use seeded RNG)

### Running Tests

```bash
# Run all tests
npm run test

# Run tests in watch mode
npm run test:watch

# Run with coverage
npm run test:cov

# Run with UI
npm run test:ui

# Full verification (typecheck + lint + test + build + benchmarks)
npm run test:all
```

---

## Documentation

### When to Update Documentation

- Adding new public API
- Changing existing API behavior
- Adding new modules or features
- Fixing bugs that affect documented behavior

### Documentation Files

| File | Purpose |
|------|---------|
| `README.md` | Project overview |
| `docs/API_REFERENCE.md` | API reference index |
| `CONTRIBUTING.md` | This file |
| `docs/guides/*.md` | Detailed module guides |
| `docs/tasks-user-guide.md` | Tasks module guide |

### Code Examples

- All examples should be runnable
- Use realistic parameter values
- Include expected output in comments

---

## Pull Request Guidelines

### Before Submitting

- [ ] All tests pass (`npm run test:all`)
- [ ] Code follows style guidelines
- [ ] Documentation updated (if applicable)
- [ ] Commit messages are clear
- [ ] No merge conflicts with main branch

### PR Title Format

```
feat: Add new OFDM modulation support
fix: Correct Doppler shift calculation
docs: Update channel model examples
```

### PR Description Template

```markdown
## Description
Brief description of the changes.

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Refactoring
- [ ] Other (please describe)

## Testing
How was this tested?

## Checklist
- [ ] Tests pass
- [ ] Documentation updated
- [ ] Code follows style guidelines
```

### Review Process

1. Submit PR
2. Automated checks run (CI)
3. Code review by maintainer
4. Address feedback if needed
5. Merge when approved

---

## Project Structure

```
symbion/
├── src/
│   ├── core/           # Core framework (Space, Objective, Runner, etc.)
│   ├── models/         # Physics models
│   │   ├── phy/        # Physical layer (channel, modulation, coding)
│   │   ├── robotics/   # Dynamics, control
│   │   ├── planning/   # Path planning, trajectory
│   │   └── sensing/    # Perception, environment
│   ├── tasks/          # Research tasks
│   │   ├── u2u-mcs/    # MCS selection task
│   │   └── isac-trajectory/  # Trajectory optimization task
│   ├── ai/             # AI/RL interface
│   ├── isac/           # ISAC framework
│   └── extras/         # Experimental modules
├── __tests__/          # Unit tests
├── benchmarks/         # Benchmark examples
│   ├── u2u-mcs/
│   └── isac-los-urllc/
├── docs/               # Documentation
│   └── guides/         # Module guides
├── scripts/            # Utility scripts
│   └── test-all.sh     # Unified test runner
├── dist/               # Build output
├── index.ts            # Main entry point
├── package.json
├── tsconfig.json
├── tsup.config.ts
└── vitest.config.ts
```

---

## Questions?

If you have questions or need help:
- Open an issue on GitHub
- Check existing documentation
- Review closed issues/PRs for similar topics

Thank you for contributing!
---

*Last Updated: 2025-12-27 | Version 1.0.0*
