// Copyright 2016-2018 California Institute of Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by dfremont on 7/24/17.
//

#ifndef KONTEST_SCENARIOS_H
#define KONTEST_SCENARIOS_H

#include <klee/klee.h>

#include <assert.h>

/*
 * Syntax for scenarios.
 */

#define KT_ASSERT(expr) if (kontest_internal_running && kontest_internal_active) assert(expr)

#define BEGIN_ACTION(name, ...) \
MAKE_IDENTIFIER(name) \
static void name(__VA_ARGS__) { \
	kontest_internal_active = kontest_internal_begin_block(KT_ACTION, IDENTIFIER(name), 0, 0); \
	if (!kontest_internal_active) \
		goto kontest_cleanup; \
	{
#define END_ACTION \
	} \
	kontest_cleanup: \
	kontest_internal_active = kontest_internal_end_block(); \
}

#define BEGIN_SCENARIO(name, ...) \
MAKE_IDENTIFIER(name) \
static void name(__VA_ARGS__) { \
	kontest_internal_active = kontest_internal_begin_block(KT_SCENARIO, IDENTIFIER(name), 0, 0); \
	if (!kontest_internal_active) \
		goto kontest_cleanup; \
	{
#define END_SCENARIO \
	} \
  kontest_cleanup: \
	kontest_internal_active = kontest_internal_end_block(); \
}

#define TEST_SCENARIO(name) \
int main(void) { \
	if (kontest_internal_mode(KT_PARSING)) { \
    kontest_internal_running = 0; \
    name(); \
  } \
	kontest_internal_mode(KT_RUNNING); \
  kontest_internal_running = 1; \
	name(); \
	kontest_internal_mode(KT_DONE); \
	return 0; \
}

#define BEGIN_REPEAT(min, max) { \
  kontest_internal_active = kontest_internal_begin_block(KT_REPEAT, 0, (min), (max)); \
  if (kontest_internal_active) BLOCK_LOOP_START
#define END_REPEAT \
  BLOCK_LOOP_END \
  kontest_internal_active = kontest_internal_end_block(); \
}

#define ANY(...) do { \
	kontest_internal_active = kontest_internal_begin_block(KT_ANY, 0, 0, 0); \
  if (kontest_internal_active) { \
    __VA_ARGS__; \
  } \
	kontest_internal_active = kontest_internal_end_block(); \
} while (0)

#define INTERLEAVE(...) do { \
	kontest_internal_active = kontest_internal_begin_block(KT_INTERLEAVE, 0, 0, 0); \
  if (kontest_internal_active) BLOCK_LOOP(__VA_ARGS__) \
	kontest_internal_active = kontest_internal_end_block(); \
} while (0)

#define REQUIRE_BEFORE(a, b) kontest_internal_require_before(IDENTIFIER(a), IDENTIFIER(b))
#define REQUIRE_NOT_NEXT(a, b) kontest_internal_require_not_next(IDENTIFIER(a), IDENTIFIER(b))
#define REQUIRE_EVENTUALLY(a, b) kontest_internal_require_eventually(IDENTIFIER(a), IDENTIFIER(b))

/*
 * Internal Kontest state, macros, and functions.
 * NOT FOR EXTERNAL USE!
 */

extern int kontest_internal_active;
extern int kontest_internal_running;

// Identical string literals are not guaranteed to have the same addresses;
// so we go through a variable to ensure uniqueness.
#define IDENTIFIER(name) &(IDENTIFIER_NAME(name)[0])
typedef const char KontestIdentifier[];
#define MAKE_IDENTIFIER(name) static const char IDENTIFIER_NAME(name)[] = #name;
#define IDENTIFIER_NAME(name) kontest_id_##name

#define BLOCK_LOOP(...) BLOCK_LOOP_START __VA_ARGS__; BLOCK_LOOP_END
#define BLOCK_LOOP_START while (kontest_internal_loop()) {
#define BLOCK_LOOP_END }

// Block types; these must match the enum in KontestBlock.h
#define KT_ACTION 0
#define KT_SCENARIO 1
#define KT_REPEAT 2
#define KT_ANY 3
#define KT_INTERLEAVE 4

// Modes; these must match the enum in ScenarioHandler.h
#define KT_PARSING 1
#define KT_RUNNING 2
#define KT_DONE 3

int kontest_internal_mode(unsigned mode);
int kontest_internal_begin_block(unsigned type, KontestIdentifier id, int data1, int data2);
int kontest_internal_end_block();
int kontest_internal_loop();

void kontest_internal_require_before(KontestIdentifier a, KontestIdentifier b);
void kontest_internal_require_not_next(KontestIdentifier a, KontestIdentifier b);
void kontest_internal_require_eventually(KontestIdentifier a, KontestIdentifier b);

#endif // KONTEST_SCENARIOS_H
