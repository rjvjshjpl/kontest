// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: not grep -v ^C %t.log
// RUN: grep ^C$ %t.log
// RUN: grep ^CA$ %t.log
// RUN: grep ^CB$ %t.log
// RUN: grep ^CAA$ %t.log
// RUN: grep ^CAB$ %t.log
// RUN: grep ^CBA$ %t.log
// RUN: grep ^CBB$ %t.log

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

BEGIN_SCENARIO(scene)
	RC();
	BEGIN_REPEAT(0, 2)
		ANY(RA(), RB());
	END_REPEAT
	end();
END_SCENARIO

TEST_SCENARIO(scene)
