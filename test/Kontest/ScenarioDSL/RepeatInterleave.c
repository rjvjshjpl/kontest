// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log 2> %t.log.err
// RUN: grep ^ABC$ %t.log
// RUN: grep ^BAC$ %t.log
// RUN: grep ^BCA$ %t.log
// RUN: grep ^ABCABC$ %t.log
// RUN: grep ^ABCBAC$ %t.log
// RUN: grep ^ABCBCA$ %t.log
// RUN: grep ^BACABC$ %t.log
// RUN: grep ^BACBAC$ %t.log
// RUN: grep ^BACBCA$ %t.log
// RUN: grep ^BCAABC$ %t.log
// RUN: grep ^BCABAC$ %t.log
// RUN: grep ^BCABCA$ %t.log
// RUN: grep "skeletons = 12$" %t.log.err
// RUN: grep "completed paths = 12$" %t.log.err

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
  RB();
  RC();
END_SCENARIO

BEGIN_SCENARIO(SB)
	INTERLEAVE(RA(), SA());
END_SCENARIO

BEGIN_SCENARIO(scene)
	BEGIN_REPEAT(1, 2)
		SB();
	END_REPEAT
	end();
END_SCENARIO

TEST_SCENARIO(scene)
