#include "test_pipeline.h"
#include <stdio.h>

void run_all_tests(void) {
    printf("\r\n====================================\r\n");
    printf(" STARTING AUTOMATED TEST SUITE\r\n");
    printf("====================================\r\n");

    int tests_passed = 0;
    int total_tests = 2;

    if (test_star_id_module()) tests_passed++;
    if (test_quest_module()) tests_passed++;

    printf("====================================\r\n");
    if (tests_passed == total_tests) {
        printf(" [VERDICT] ALL TESTS PASSED (%d/%d)\r\n", tests_passed, total_tests);
        printf(" SYSTEM IS FLIGHT READY.\r\n");
    } else {
        printf(" [VERDICT] PIPELINE FAILURE (%d/%d passed)\r\n", tests_passed, total_tests);
        printf(" DO NOT MERGE CODE.\r\n");
    }
    printf("====================================\r\n\r\n");
}