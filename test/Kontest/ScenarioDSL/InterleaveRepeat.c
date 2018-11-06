// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: not grep AA %t.log
// RUN: not grep BB %t.log
// RUN: not grep CC %t.log
// RUN: not grep -v A %t.log
// RUN: not grep -v B %t.log
// RUN: not grep -v C %t.log
// RUN: grep ^ABC$ %t.log
// RUN: grep ^BAC$ %t.log
// RUN: grep ^BCA$ %t.log
// RUN: grep ^ABCBC$ %t.log
// RUN: grep ^BACBC$ %t.log
// RUN: grep ^BCABC$ %t.log
// RUN: grep ^BCBAC$ %t.log
// RUN: grep ^BCBCA$ %t.log

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

BEGIN_SCENARIO(SA)
	RA();
END_SCENARIO

BEGIN_SCENARIO(SB)
	BEGIN_REPEAT(1, 2)
		RB();
		RC();
	END_REPEAT
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(SA(), SB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
