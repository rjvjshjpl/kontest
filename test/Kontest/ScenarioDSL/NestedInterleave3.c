// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log 2> %t.log.err
// RUN: grep ^ABCD$ %t.log
// RUN: grep ^ACBD$ %t.log
// RUN: grep ^BACD$ %t.log
// RUN: grep ^BCAD$ %t.log
// RUN: grep ^CABD$ %t.log
// RUN: grep ^CBAD$ %t.log
// RUN: grep ^ACDB$ %t.log
// RUN: grep ^BCDA$ %t.log
// RUN: grep ^CADB$ %t.log
// RUN: grep ^CBDA$ %t.log
// RUN: grep ^CDAB$ %t.log
// RUN: grep ^CDBA$ %t.log
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

BEGIN_ACTION(RD)
	printf("D");
END_ACTION

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(SA)
  RC();
  RD();
END_SCENARIO

BEGIN_SCENARIO(SB)
	INTERLEAVE(RB(), SA());
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(RA(), SB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
