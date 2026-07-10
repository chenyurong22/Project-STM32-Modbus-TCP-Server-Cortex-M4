#ifndef MODBUS_REGISTER_SELFTEST_H
#define MODBUS_REGISTER_SELFTEST_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t tests_total;
    uint32_t tests_passed;
    uint32_t tests_failed;
    int first_error_line;
} MbSelfTestResult;

void mb_selftest_run(MbSelfTestResult *result);
void mb_selftest_print(char *buffer, size_t length, const MbSelfTestResult *result);
void mb_selftest_notify(const MbSelfTestResult *result);

#endif
