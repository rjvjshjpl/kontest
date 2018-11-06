// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: not %klee --output-dir=%t.klee-out --exit-on-error %t.bc 2> %t.log
// RUN: grep "ERROR: invalid range for REPEAT block" %t.log

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RB)
END_ACTION

BEGIN_SCENARIO(scene)
	BEGIN_REPEAT(4, 3)
		RB();
	END_REPEAT
END_SCENARIO

TEST_SCENARIO(scene)
