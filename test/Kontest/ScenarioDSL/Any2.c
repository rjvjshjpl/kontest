// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log 2> %t.log.err
// RUN: grep ^A$ %t.log
// RUN: grep ^B$ %t.log
// RUN: grep ^C$ %t.log
// RUN: grep ^CC$ %t.log
// RUN: grep ^CCC$ %t.log
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

BEGIN_SCENARIO(SC)
	RC();
	BEGIN_REPEAT(0, 2)
    RC();
  END_REPEAT
END_SCENARIO

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(scene)
	ANY(RA(), RB(), SC());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
