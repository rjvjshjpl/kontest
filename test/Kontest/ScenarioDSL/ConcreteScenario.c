// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > FileCheck %s

// Special case of a scenario with no symbolics and a single skeleton.
// Need to make sure we don't parse twice.

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA, unsigned x)
	printf("RA: %d\n", x);
END_ACTION

BEGIN_ACTION(RB)
	printf("RB\n");
END_ACTION

BEGIN_SCENARIO(scene)
	// CHECK: RA: 4
	RA(4);
	// CHECK: RA: 2
	RA(2);
	// CHECK: RB
	RB();
END_SCENARIO

TEST_SCENARIO(scene)
