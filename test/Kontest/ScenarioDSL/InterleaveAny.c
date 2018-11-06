// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log 2> %t.log.err
// RUN: grep ^ABC$ %t.log
// RUN: grep ^ACB$ %t.log
// RUN: grep ^CAB$ %t.log
// RUN: grep ^DC %t.log
// RUN: grep ^CD %t.log
// RUN: grep "skeletons = 5$" %t.log.err
// RUN: grep "completed paths = 5$" %t.log.err

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA)
	printf("A");
END_ACTION

BEGIN_ACTION(RB)
	printf("B");
END_ACTION

BEGIN_ACTION(RC)
	printf("C");
END_ACTION

BEGIN_ACTION(RD)
	printf("D");
END_ACTION

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(SA)
	RA();
	RB();
END_SCENARIO

BEGIN_SCENARIO(SB)
	ANY(SA(), RD());
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(SB(), RC());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
