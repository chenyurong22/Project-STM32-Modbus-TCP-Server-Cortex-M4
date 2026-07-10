/**
 * @file    register_selftest.c
 * @brief   On-device self-test for Modbus register/coil map.
 *
 * Drop this file into `Tests/stm32/` and add it to your build when you
 * want a quick sanity check on the Modbus data model (no TCP required).
 * It exercises the public API from App/include/modbus.h.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "platform_port.h"
#include "modbus.h"
#include "register_selftest.h"

#ifndef MB_ST_ASSERT
#define MB_ST_ASSERT(expr) do { if(!(expr)) return -__LINE__; } while(0)
#endif

// --- Public result type -----------------------------------------------------

// Forward declarations
static int test_holding_registers_basic(void);
static int test_input_registers_basic(void);
static int test_coils_and_discretes(void);
static int test_bounds_behavior(void);
static int test_bulk_patterns(void);

// Optional weak symbol to surface result (LED/UART/etc.)
__attribute__((weak)) void mb_selftest_notify(const MbSelfTestResult *r) {
    (void)r; // user may override to blink LEDs or print to UART
}

// --- Helper: predictable pseudo-random -------------------------------------
static uint16_t prand16(uint32_t seed) {
    // xorshift16-ish (deterministic, tiny)
    uint16_t x = (uint16_t)(seed & 0xFFFFu);
    x ^= (uint16_t)(x << 7);
    x ^= (uint16_t)(x >> 9);
    x ^= (uint16_t)(x << 8);
    return x;
}

// --- Tests ------------------------------------------------------------------

static int test_holding_registers_basic(void) {
    // Write a few values and read back
    mb_set_hreg(0, 0x1234);
    mb_set_hreg(1, 0xABCD);
    mb_set_hreg(2, 0x0);
    MB_ST_ASSERT(mb_get_hreg(0) == 0x1234);
    MB_ST_ASSERT(mb_get_hreg(1) == 0xABCD);
    MB_ST_ASSERT(mb_get_hreg(2) == 0x0000);

    // Overwrite
    mb_set_hreg(1, 0x5555);
    MB_ST_ASSERT(mb_get_hreg(1) == 0x5555);
    return 0;
}

static int test_input_registers_basic(void) {
    // Input regs typically reflect sensors; here we just exercise setters/getters
    mb_set_ireg(0, 10);
    mb_set_ireg(1, 20);
    mb_set_ireg(2, 30);
    MB_ST_ASSERT(mb_get_ireg(0) == 10);
    MB_ST_ASSERT(mb_get_ireg(1) == 20);
    MB_ST_ASSERT(mb_get_ireg(2) == 30);
    return 0;
}

static int test_coils_and_discretes(void) {
    // Pattern: 1,0,1,1,0,0,1,...
    const uint8_t pat[8] = {1,0,1,1,0,0,1,0};
    for (uint16_t i=0;i<8;i++) {
        mb_set_coil(i, pat[i]);
        mb_set_dinput(i, pat[7-i]); // inverse order just to mix it up
    }
    for (uint16_t i=0;i<8;i++) {
        MB_ST_ASSERT(mb_get_coil(i) == pat[i]);
        MB_ST_ASSERT(mb_get_dinput(i) == pat[7-i]);
    }

    // Flip a few coils
    mb_set_coil(3, 0);
    mb_set_coil(4, 1);
    MB_ST_ASSERT(mb_get_coil(3) == 0);
    MB_ST_ASSERT(mb_get_coil(4) == 1);
    return 0;
}

static int test_bounds_behavior(void) {
    // Reads out of range should be safe and return 0 per our implementation
    MB_ST_ASSERT(mb_get_hreg((uint16_t)MB_MAX_HREGS) == 0);
    MB_ST_ASSERT(mb_get_ireg((uint16_t)MB_MAX_IREGS) == 0);
    MB_ST_ASSERT(mb_get_coil((uint16_t)MB_MAX_COILS) == 0);
    MB_ST_ASSERT(mb_get_dinput((uint16_t)MB_MAX_DISCRETE_INPUTS) == 0);

    // Writes out of range should not crash; we cannot easily assert more here
    mb_set_hreg((uint16_t)MB_MAX_HREGS, 0xBEEF);
    mb_set_ireg((uint16_t)MB_MAX_IREGS, 0xBEEF);
    mb_set_coil((uint16_t)MB_MAX_COILS, 1);
    mb_set_dinput((uint16_t)MB_MAX_DISCRETE_INPUTS, 1);

    // Neighbor within range remains writeable
    if (MB_MAX_HREGS > 0) {
        uint16_t last = (uint16_t)(MB_MAX_HREGS - 1);
        mb_set_hreg(last, 0xA5A5);
        MB_ST_ASSERT(mb_get_hreg(last) == 0xA5A5);
    }
    return 0;
}

static int test_bulk_patterns(void) {
    // Fill first N holding regs with deterministic pattern and verify
    const uint16_t N = (MB_MAX_HREGS > 64) ? 64 : (uint16_t)MB_MAX_HREGS;
    for (uint16_t i=0;i<N;i++) {
        uint16_t v = prand16(0xBEEF + i) ^ (uint16_t)(i * 31u);
        mb_set_hreg(i, v);
    }
    for (uint16_t i=0;i<N;i++) {
        uint16_t v = prand16(0xBEEF + i) ^ (uint16_t)(i * 31u);
        MB_ST_ASSERT(mb_get_hreg(i) == v);
    }

    // Coils bulk pattern
    const uint16_t C = (MB_MAX_COILS > 96) ? 96 : (uint16_t)MB_MAX_COILS;
    for (uint16_t i=0;i<C;i++) {
        mb_set_coil(i, (i ^ 0x5A) & 1);
    }
    for (uint16_t i=0;i<C;i++) {
        MB_ST_ASSERT((mb_get_coil(i) & 1u) == ((i ^ 0x5A) & 1u));
    }
    return 0;
}

// --- Public entry points ----------------------------------------------------

void mb_selftest_run(MbSelfTestResult *out) {
    MbSelfTestResult r = {0,0,0,0};

    // Ensure clean map
    mb_init();

    int (*const tests[])(void) = {
        test_holding_registers_basic,
        test_input_registers_basic,
        test_coils_and_discretes,
        test_bounds_behavior,
        test_bulk_patterns,
    };

    const size_t count = sizeof(tests)/sizeof(tests[0]);
    for (size_t i=0; i<count; ++i) {
        r.tests_total++;
        int rc = tests[i]();
        if (rc == 0) {
            r.tests_passed++;
        } else {
            r.tests_failed++;
            if (r.first_error_line == 0) r.first_error_line = rc; // negative line
        }
    }

    if (out) *out = r;
    mb_selftest_notify(&r);
}

// Pretty-printer (optional). Provide your own UART printf/ITM if desired.
void mb_selftest_print(char *buf, size_t len, const MbSelfTestResult *r) {
    if (!buf || !len || !r) return;
    (void)snprintf(buf, len,
        "Modbus SelfTest: total=%lu, passed=%lu, failed=%lu, first_err=%d\n",
        (unsigned long)r->tests_total,
        (unsigned long)r->tests_passed,
        (unsigned long)r->tests_failed,
        r->first_error_line);
}

// Example usage (call from main after init):
//   MbSelfTestResult res; mb_selftest_run(&res);
//   char line[96]; mb_selftest_print(line, sizeof(line), &res);
//   printf("%s", line);
