#!/bin/bash
# =============================================================================
# Symbion Library - Unified Test Script
# =============================================================================
# This script runs all verification steps for the Symbion library:
#   1. TypeScript type checking
#   2. ESLint code linting
#   3. Unit tests (Vitest)
#   4. Build (tsup)
#   5. Benchmark examples
#
# Usage:
#   ./scripts/test-all.sh           # Run all tests
#   ./scripts/test-all.sh --quick   # Skip benchmark examples
#   ./scripts/test-all.sh --help    # Show help
# =============================================================================

# Don't use set -e, we handle errors manually

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Counters
PASSED=0
FAILED=0
SKIPPED=0

# Options
QUICK_MODE=false
VERBOSE=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --quick|-q)
            QUICK_MODE=true
            ;;
        --verbose|-v)
            VERBOSE=true
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --quick, -q     Skip benchmark examples (faster)"
            echo "  --verbose, -v   Show detailed output"
            echo "  --help, -h      Show this help message"
            exit 0
            ;;
    esac
done

# Print header
print_header() {
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}${BOLD}           Symbion Library - Unified Test Suite                 ${NC}${CYAN}║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}Started at:${NC} $(date '+%Y-%m-%d %H:%M:%S')"
    if [ "$QUICK_MODE" = true ]; then
        echo -e "${YELLOW}Mode: Quick (skipping benchmark examples)${NC}"
    else
        echo -e "${BLUE}Mode: Full${NC}"
    fi
    echo ""
}

# Print step header
print_step() {
    local step_num=$1
    local step_name=$2
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BOLD}[$step_num] $step_name${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# Print result
print_result() {
    local status=$1
    local message=$2
    local duration=$3
    
    if [ "$status" = "pass" ]; then
        echo -e "${GREEN} PASSED${NC} - $message ${BLUE}(${duration}s)${NC}"
        ((PASSED++))
    elif [ "$status" = "fail" ]; then
        echo -e "${RED} FAILED${NC} - $message"
        ((FAILED++))
    elif [ "$status" = "skip" ]; then
        echo -e "${YELLOW} SKIPPED${NC} - $message"
        ((SKIPPED++))
    fi
}

# Run a test step
run_step() {
    local step_name=$1
    local command=$2
    local start_time=$(date +%s)
    
    if [ "$VERBOSE" = true ]; then
        if eval "$command"; then
            local end_time=$(date +%s)
            local duration=$((end_time - start_time))
            print_result "pass" "$step_name" "$duration"
            return 0
        else
            print_result "fail" "$step_name"
            return 1
        fi
    else
        if eval "$command" > /tmp/symbion_test_output.txt 2>&1; then
            local end_time=$(date +%s)
            local duration=$((end_time - start_time))
            print_result "pass" "$step_name" "$duration"
            return 0
        else
            print_result "fail" "$step_name"
            echo -e "${RED}Error output:${NC}"
            tail -20 /tmp/symbion_test_output.txt
            return 1
        fi
    fi
}

# Print summary
print_summary() {
    local total=$((PASSED + FAILED + SKIPPED))
    local end_time=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}${BOLD}                       Test Summary                             ${NC}${CYAN}║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "  ${GREEN}Passed:${NC}  $PASSED"
    echo -e "  ${RED}Failed:${NC}  $FAILED"
    echo -e "  ${YELLOW}Skipped:${NC} $SKIPPED"
    echo -e "  ${BLUE}Total:${NC}   $total"
    echo ""
    echo -e "${BLUE}Completed at:${NC} $end_time"
    echo ""
    
    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}${BOLD}╔════════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${GREEN}${BOLD}║                     ALL TESTS PASSED!                      ║${NC}"
        echo -e "${GREEN}${BOLD}╚════════════════════════════════════════════════════════════════╝${NC}"
        return 0
    else
        echo -e "${RED}${BOLD}╔════════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${RED}${BOLD}║                     SOME TESTS FAILED                       ║${NC}"
        echo -e "${RED}${BOLD}╚════════════════════════════════════════════════════════════════╝${NC}"
        return 1
    fi
}

# =============================================================================
# Main Script
# =============================================================================

# Change to project root
cd "$(dirname "$0")/.."

print_header

# Step 1: TypeScript Type Check
print_step "1/6" "TypeScript Type Check"
run_step "TypeScript compilation (tsc --noEmit)" "npm run typecheck" || true

# Step 2: ESLint
print_step "2/6" "ESLint Code Linting"
run_step "ESLint check (errors only)" "npm run lint -- --max-warnings 500" || true

# Step 3: Unit Tests
print_step "3/6" "Unit Tests (Vitest)"
run_step "Vitest unit tests" "npm run test" || true

# Step 4: Build
print_step "4/6" "Build (tsup)"
run_step "Build ESM/CJS/DTS" "npm run build" || true

# Step 5: U2U-MCS Benchmark
print_step "5/6" "U2U-MCS Benchmark Example"
if [ "$QUICK_MODE" = true ]; then
    print_result "skip" "U2U-MCS benchmark (quick mode)"
else
    run_step "U2U-MCS simple example" "npx tsx benchmarks/u2u-mcs/simple-example.ts > /dev/null 2>&1" || true
fi

# Step 6: ISAC Trajectory Benchmark
print_step "6/6" "ISAC Trajectory Benchmark Example"
if [ "$QUICK_MODE" = true ]; then
    print_result "skip" "ISAC Trajectory benchmark (quick mode)"
else
    run_step "ISAC Trajectory simple example" "npx tsx benchmarks/isac-los-urllc/simple-example.ts > /dev/null 2>&1" || true
fi

# Print summary
print_summary
exit_code=$?

exit $exit_code
