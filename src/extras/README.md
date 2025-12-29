# Symbion Extras

This directory contains experimental, research-focused, or advanced modules that are not part of the core Symbion library.

These modules may have additional dependencies, less stable APIs, or are intended for specific research use cases.

## Module Categories

### Coding (`coding/`)
- `ldpc.ts` - LDPC encoding/decoding
- `polar.ts` - Polar codes implementation  
- `turbo.ts` - Turbo codes

### Networking (`networking/`)
- `aodv.ts` - Ad-hoc On-Demand Distance Vector routing
- `csma.ts` - CSMA/CA MAC protocol
- `tdma.ts` - TDMA MAC protocol

### Modulation (`modulation/`)
- `otfs.ts` - Orthogonal Time Frequency Space modulation
- `fbmc.ts` - Filter Bank Multi-Carrier

## Usage

These modules are not exported from the main `symbion` package. To use them:

```typescript
import { LDPCEncoder } from 'symbion/extras/coding/ldpc';
import { AODVRouter } from 'symbion/extras/networking/aodv';
```

## Stability Warning

**Warning:** These modules are provided AS-IS for research purposes. APIs may change without notice.

## Contributing

If you want to promote a module from `extras` to `core`, please:
1. Add comprehensive unit tests
2. Document the API
3. Ensure backward compatibility
4. Submit a PR for review
