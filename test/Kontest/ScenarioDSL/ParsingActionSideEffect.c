// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: grep ^once$ %t.log

// The body of an action should not be executed during parsing.

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA)
	printf("once");
END_ACTION

BEGIN_SCENARIO(scene)
	RA();
END_SCENARIO

TEST_SCENARIO(scene)
