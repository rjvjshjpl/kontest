// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: not grep AA %t.log
// RUN: not grep BB %t.log
// RUN: not grep CC %t.log
// RUN: not grep DD %t.log
// RUN: not grep BA %t.log
// RUN: not grep DC %t.log
// RUN: not grep -v A %t.log
// RUN: not grep -v B %t.log
// RUN: not grep -v C %t.log
// RUN: not grep -v D %t.log
// RUN: grep ^ABCD$ %t.log
// RUN: grep ^ACBD$ %t.log
// RUN: grep ^ACDB$ %t.log
// RUN: grep ^CABD$ %t.log
// RUN: grep ^CADB$ %t.log
// RUN: grep ^CDAB$ %t.log

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
	RC();
	RD();
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(SA(), SB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
