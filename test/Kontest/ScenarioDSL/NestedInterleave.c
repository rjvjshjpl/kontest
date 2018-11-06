// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: not grep AA %t.log
// RUN: not grep BB %t.log
// RUN: not grep CC %t.log
// RUN: not grep ^A$ %t.log
// RUN: not grep ^B$ %t.log
// RUN: not grep ^C$ %t.log
// RUN: grep ^ABC$ %t.log
// RUN: grep ^ACB$ %t.log
// RUN: grep ^BAC$ %t.log
// RUN: grep ^BCA$ %t.log
// RUN: grep ^CAB$ %t.log
// RUN: grep ^CBA$ %t.log

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

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(SB)
	INTERLEAVE(RB(), RC());
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(RA(), SB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
