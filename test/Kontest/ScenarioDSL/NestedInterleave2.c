// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log 2> %t.log.err
// RUN: grep "skeletons = 24$" %t.log.err
// RUN: grep "completed paths = 24$" %t.log.err
// RUN: wc -l %t.log | grep "24 "

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RB)
	printf("B");
END_ACTION

BEGIN_ACTION(RC)
	printf("C");
END_ACTION

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(SB)
	INTERLEAVE(RB(), RC());
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(SB(), SB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
