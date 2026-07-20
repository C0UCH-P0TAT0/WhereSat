#ifndef TEST_PIPELINE_H
#define TEST_PIPELINE_H

#include <stdbool.h>

// Master runner that executes all tests
void run_all_tests(void);

// Individual test modules
bool test_star_id_module(void);
bool test_quest_module(void);

#endif // TEST_PIPELINE_H