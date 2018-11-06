// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: not %klee --output-dir=%t.klee-out --exit-on-error %t.bc 2> %t.log
// RUN: grep "ERROR: invoked scenario or action outside of TEST_SCENARIO" %t.log

// Invoking an action or scenario outside TEST_SCENARIO should yield an error.

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RB)
END_ACTION

BEGIN_SCENARIO(scene)
	RB();
END_SCENARIO

int main(void) {
	scene();
	return 0;
}
