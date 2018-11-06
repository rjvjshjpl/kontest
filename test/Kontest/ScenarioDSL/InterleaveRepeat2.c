// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: not grep AAA %t.log
// RUN: not grep BBB %t.log
// RUN: not grep ^A$ %t.log
// RUN: not grep ^B$ %t.log
// RUN: grep ^AA$ %t.log
// RUN: grep ^BAA$ %t.log
// RUN: grep ^ABA$ %t.log
// RUN: grep ^AAB$ %t.log
// RUN: grep ^BBAA$ %t.log
// RUN: grep ^BABA$ %t.log
// RUN: grep ^BAAB$ %t.log
// RUN: grep ^ABAB$ %t.log
// RUN: grep ^AABB$ %t.log
// RUN: grep ^ABBA$ %t.log

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

BEGIN_SCENARIO(SA)
	RA();
	RA();
END_SCENARIO

BEGIN_SCENARIO(SB)
	BEGIN_REPEAT(0, 2)
		RB();
	END_REPEAT
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(SA(), SB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
