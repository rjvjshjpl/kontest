// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: not %klee --output-dir=%t.klee-out --exit-on-error %t.bc 2> %t.log
// RUN: grep "ERROR: actions cannot invoke another action" %t.log

// Invoking an action inside an action should yield an error.

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA)
END_ACTION

BEGIN_ACTION(RB)
	RA();
END_ACTION

BEGIN_SCENARIO(scene)
	RB();
END_SCENARIO

TEST_SCENARIO(scene)
