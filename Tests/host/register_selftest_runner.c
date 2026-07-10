#include "register_selftest.h"

#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    MbSelfTestResult result;
    char line[128];

    mb_selftest_run(&result);
    mb_selftest_print(line, sizeof(line), &result);
    fputs(line, stdout);
    return result.tests_failed == 0u ? EXIT_SUCCESS : EXIT_FAILURE;
}
