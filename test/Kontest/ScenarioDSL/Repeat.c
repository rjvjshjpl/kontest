// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: grep ^A$ %t.log
// RUN: grep ^AB$ %t.log
// RUN: grep ^ABB$ %t.log
// RUN: grep ^ABBB$ %t.log

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA)
	printf("A");
END_ACTION

BEGIN_ACTION(RB)
	printf("B");
END_ACTION

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(scene)
	RA();
	BEGIN_REPEAT(0, 3)
		RB();
	END_REPEAT
	end();
END_SCENARIO

TEST_SCENARIO(scene)
