// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc

// KT_ASSERTs should not fire during parsing.

#include <stdio.h>
#include <kontest/scenarios.h>

int x = 0;

BEGIN_ACTION(RA)
	x = 1;
END_ACTION

BEGIN_SCENARIO(scene)
	RA();
	KT_ASSERT(x);
END_SCENARIO

TEST_SCENARIO(scene)
